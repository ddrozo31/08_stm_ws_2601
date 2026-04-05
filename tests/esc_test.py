#!/usr/bin/env python3
"""
esc_test.py  --  PC-side ESC communication test script
Talks to the STM32G431 ESC firmware over ST-Link VCP (COM3 / USART2).

Protocol
--------
Command frame  (PC -> STM32)  5 bytes:
  [0xAA] [cmd_lo] [cmd_hi] [0x00] [XOR_chk]
   SOF    int16_t u  (-32768=-1.0 / +32767=+1.0)   XOR(bytes 1..3)

Telemetry frame (STM32 -> PC) 15 bytes:
  [0xBB][spd_lo][spd_hi][esc_st][faults][u_lo][u_hi][v_lo][v_hi]
        [iq_lo][iq_hi][id_lo][id_hi][mcsdk_st][XOR_chk]
   SOF   int16 RPM  ESC st  faults  int16 cmd  uint16 V
         int16 iq(mA)  int16 id(mA)  mcsdk_st  XOR(bytes 1..13)

ESC state byte:
  0=BOOT  1=WAIT_NEUTRAL  2=READY  3=FORWARD  4=BRAKE  5=REVERSE  6=FAULT

Usage
-----
  pip install pyserial
  python esc_test.py

  At the prompt, type a value in [-1.0, +1.0] and press Enter.
  Type 'q' to quit (sends neutral before closing).
"""

import serial
import struct
import threading
import time
import sys

# -- Configuration -------------------------------------------------------------
PORT      = '/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066BFF494970535067242339-if02'   # RPi5: ST-Link VCP; Windows: 'COM4'
BAUDRATE  = 1843200
CMD_HZ    = 10          # command send rate (Hz) -- must be faster than 500 ms timeout

# -- Protocol constants --------------------------------------------------------
CMD_SOF       = 0xAA
TLM_SOF       = 0xBB
CMD_FRAME_LEN = 5
TLM_FRAME_LEN = 15

# ESC_State_t enum values (from mc_app_hooks.c)
STATE_NAMES = {
    0: 'BOOT',
    1: 'WAIT_NEUTRAL',
    2: 'READY',
    3: 'FORWARD',
    4: 'BRAKE',
    5: 'REVERSE',
    6: 'FAULT',
}

# Fault bitmask (lower byte of MC_GetCurrentFaultsMotor1)
FAULT_BITS = {
    0x02: 'OVER_VOLT',
    0x04: 'UNDER_VOLT',
    0x08: 'OVER_TEMP',
    0x10: 'START_UP',
    0x20: 'SPEED_FDBK',
    0x40: 'OVER_CURR',
    0x80: 'SW_ERROR',
}


# CFOC_State_t (custom_foc.h) — sent in telemetry mc_st byte
CFOC_STATE_NAMES = {
    0: 'IDLE',
    1: 'ALIGNMENT',
    2: 'OPEN_LOOP',
    3: 'CROSSFADE',
    4: 'CLOSED_LOOP',
    5: 'FAULT',
}

def decode_cfoc_state(b: int) -> str:
    return CFOC_STATE_NAMES.get(b, f'CFOC_{b}')
# -- Frame builders / decoders -------------------------------------------------

def build_command(u: float) -> bytes:
    """Encode normalised command u in [-1.0, +1.0] into a 5-byte frame."""
    u   = max(-1.0, min(1.0, u))
    raw = int(u * 32767)
    lo  = raw & 0xFF
    hi  = (raw >> 8) & 0xFF
    chk = lo ^ hi ^ 0x00
    return bytes([CMD_SOF, lo, hi, 0x00, chk])

def decode_state(b: int) -> str:
    return STATE_NAMES.get(b, f'STATE_{b}')

def decode_faults(b: int) -> str:
    if b == 0:
        return 'none'
    return ' | '.join(name for bit, name in FAULT_BITS.items() if b & bit) or f'0x{b:02X}'

def parse_telemetry(frame: bytes):
    """Return (speed_rpm, state_str, fault_str, u_float, vbus_v, iq_a, id_a, mcsdk_str)
    or None on bad checksum."""
    if len(frame) != TLM_FRAME_LEN or frame[0] != TLM_SOF:
        return None
    chk = 0
    for i in range(1, 14):
        chk ^= frame[i]
    if chk != frame[14]:
        return None
    speed  = struct.unpack_from('<h', frame, 1)[0]   # int16 RPM
    cmd_r  = struct.unpack_from('<h', frame, 5)[0]   # int16 raw command
    vbus   = struct.unpack_from('<H', frame, 7)[0]   # uint16 Volts
    iq_ma  = struct.unpack_from('<h', frame, 9)[0]   # int16 milliAmps
    id_ma  = struct.unpack_from('<h', frame, 11)[0]  # int16 milliAmps
    u_val  = cmd_r / 32767.0
    return (speed, decode_state(frame[3]), decode_faults(frame[4]),
            u_val, vbus, iq_ma / 1000.0, id_ma / 1000.0,
            decode_cfoc_state(frame[13]))

# -- Background threads --------------------------------------------------------

_current_u = 0.0
_u_lock    = threading.Lock()

def get_u() -> float:
    with _u_lock:
        return _current_u

def set_u(val: float):
    global _current_u
    with _u_lock:
        _current_u = max(-1.0, min(1.0, val))


def sender_thread(ser: serial.Serial, stop: threading.Event):
    """Sends the current command at CMD_HZ -- keeps the watchdog alive."""
    interval = 1.0 / CMD_HZ
    while not stop.is_set():
        frame = build_command(get_u())
        try:
            ser.write(frame)
        except serial.SerialException:
            break
        time.sleep(interval)


def reader_thread(ser: serial.Serial, stop: threading.Event):
    """Reads incoming bytes, syncs on 0xBB, prints decoded telemetry."""
    buf = bytearray()
    while not stop.is_set():
        try:
            chunk = ser.read(ser.in_waiting or 1)
        except serial.SerialException:
            break
        if not chunk:
            continue
        buf.extend(chunk)

        while len(buf) >= TLM_FRAME_LEN:
            if buf[0] != TLM_SOF:
                buf.pop(0)
                continue
            frame = bytes(buf[:TLM_FRAME_LEN])
            result = parse_telemetry(frame)
            if result is None:
                buf.pop(0)
                continue
            speed, state, faults, u_val, vbus, iq_a, id_a, cfoc_st = result
            print(f"\r  [TLM]  spd={speed:6d} RPM  state={state:<14}"
                  f"  u={u_val:+.3f}  vbus={vbus:3d} V"
                  f"  iq={iq_a:+.3f}A  id={id_a:+.3f}A"
                  f"  cfoc={cfoc_st}  faults={faults}",
                  flush=True)
            buf = buf[TLM_FRAME_LEN:]

# -- Main ----------------------------------------------------------------------

def main():
    print(f"\nOpening {PORT} at {BAUDRATE} baud ...")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    print("Port open.\n")
    print("-" * 66)
    print("  ESC test  |  command range: -1.0 (full rev) .. +1.0 (full fwd)")
    print("  Firmware timeout: 500 ms  |  commands sent at 10 Hz")
    print("-" * 66)
    print("  Type a value [-1.0 .. +1.0] + Enter  to set command")
    print("  'q' + Enter  to quit")
    print("-" * 66)
    print("  IMPORTANT: firmware starts in WAIT_NEUTRAL state.")
    print("  Send u=0.0 first -- motor won't move until neutral is confirmed.")
    print("  Negative commands reverse direction (via BRAKE state).\n")

    stop = threading.Event()

    t_send = threading.Thread(target=sender_thread, args=(ser, stop), daemon=True)
    t_recv = threading.Thread(target=reader_thread, args=(ser, stop), daemon=True)
    t_send.start()
    t_recv.start()

    try:
        while True:
            try:
                raw = input(f"\n  cmd u={get_u():.3f} > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if raw.lower() == 'q':
                break

            if raw == '':
                continue

            try:
                val = float(raw)
                set_u(val)
                print(f"  -> u={get_u():.4f}  raw={int(get_u()*32767):6d}  "
                      f"frame={build_command(get_u()).hex(' ').upper()}")
            except ValueError:
                print("  [!] Invalid input. Enter a number in [-1.0, +1.0] or 'q'.")

    finally:
        print("\n  Sending neutral and closing port ...")
        set_u(0.0)
        time.sleep(0.2)
        stop.set()
        time.sleep(0.1)
        ser.close()
        print("  Done.\n")


if __name__ == '__main__':
    main()
