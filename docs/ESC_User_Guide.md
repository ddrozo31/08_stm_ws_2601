# ESC User Guide — B-G431B-ESC1 Firmware

> **Audience:** Engineers integrating the STM32 motor controller into a host system (e.g. Raspberry Pi 5 or any UART-capable controller).
> **Firmware:** Release build (`BUILD_ESC` defined), ST Motor Control SDK v6.4.1.

---

## 1. Overview

The B-G431B-ESC1 board runs sensorless Field-Oriented Control (FOC) using an STM32G431. Once flashed with the **Release** firmware it acts as a dumb, safe motor node:

- **Receives** normalized drive commands `u ∈ [-1.0, +1.0]` over UART.
- **Controls** motor torque (q-axis current, Iq) proportional to `u`.
- **Streams** telemetry back at ~10 Hz: speed, state, faults, bus voltage, currents.

No tuning GUI, no USB, no wireless. Just a serial line and a motor spinning.

---

## 2. Hardware & Electrical

| Item | Detail |
|------|--------|
| MCU | STM32G431CBU (Cortex-M4, 170 MHz) |
| Motor type | 3-phase BLDC / PMSM, sensorless |
| Motor params | Pole pairs = 2, Rs = 0.1 Ω, Ls = 10 µH |
| PWM frequency | 25 kHz, center-aligned |
| Current limit (Iq) | 12 A peak (`ESC_MAX_IQ_A`) |
| UART port | USART2 header on the ESC board |
| Baud rate | **1 843 200 bps**, 8N1 |
| Logic level | 3.3 V |

> **Important:** The UART is 3.3 V logic. If your host (e.g. RPi5) is also 3.3 V, connect directly. Do **not** use a 5 V UART adapter without a level shifter.

---

## 3. UART Communication Protocol

### 3.1 Physical link

```
Raspberry Pi 5 (or any host)          STM32 ESC
        TX ──────────────────────────▶ RX (USART2)
        RX ◀────────────────────────── TX (USART2)
       GND ─────────────────────────── GND
```

Settings: `1843200 8N1`, no flow control.

---

### 3.2 Command Frame: Host → ESC (5 bytes)

The host sends one frame per control cycle to drive the motor.

```
Byte 0  : 0xAA          (Start-of-Frame marker)
Byte 1  : cmd_lo        (low  byte of int16_t command)
Byte 2  : cmd_hi        (high byte of int16_t command)
Byte 3  : 0x00          (reserved, always zero)
Byte 4  : XOR checksum  (byte1 ^ byte2 ^ byte3)
```

#### Command value encoding

`cmd` is a **signed 16-bit integer** that maps linearly to `u ∈ [-1.0, +1.0]`:

| Desired `u` | `cmd` (int16_t) | Bytes [1,2] (little-endian) |
|-------------|-----------------|------------------------------|
| +1.0 (full forward) | +32767 | `0xFF 0x7F` |
| +0.5 (half forward) | +16384 | `0x00 0x40` |
|  0.0 (neutral)      |      0 | `0x00 0x00` |
| -0.5 (half reverse) | -16384 | `0x00 0xC0` |
| -1.0 (full reverse) | -32768 | `0x00 0x80` |

**Conversion formula:**
```
cmd = (int16_t)(u * 32767.0f)     # float u in [-1.0, +1.0]
```

#### Checksum

XOR of bytes 1, 2, and 3 (the reserved byte is included):
```
checksum = cmd_lo ^ cmd_hi ^ 0x00
         = cmd_lo ^ cmd_hi
```

Frames that fail the checksum are silently dropped. The parser is self-synchronizing on the `0xAA` SOF byte.

#### Python example

```python
import serial
import struct

ser = serial.Serial('/dev/ttyAMA0', baudrate=1843200, timeout=0.1)

def send_command(u: float):
    """Send a normalized drive command u in [-1.0, +1.0]."""
    u = max(-1.0, min(1.0, u))          # clamp
    cmd = int(u * 32767.0)
    cmd_bytes = struct.pack('<h', cmd)   # little-endian int16
    reserved = 0x00
    checksum = cmd_bytes[0] ^ cmd_bytes[1] ^ reserved
    frame = bytes([0xAA, cmd_bytes[0], cmd_bytes[1], reserved, checksum])
    ser.write(frame)

# Send neutral to unlock the ESC after boot
send_command(0.0)

# Drive forward at 70% throttle
send_command(0.7)
```

---

### 3.3 Telemetry Frame: ESC → Host (15 bytes)

The ESC sends one frame every ~100 ms automatically (no polling needed).

```
Byte  0  : 0xBB           (Start-of-Frame marker)
Byte  1  : spd_lo         (low  byte of int16_t speed in RPM)
Byte  2  : spd_hi         (high byte of int16_t speed in RPM)
Byte  3  : esc_state      (ESC state machine state, see §4)
Byte  4  : faults         (lower byte of MCSDK fault bitmask)
Byte  5  : u_lo           (low  byte of active int16_t command)
Byte  6  : u_hi           (high byte of active int16_t command)
Byte  7  : vbus_lo        (low  byte of uint16_t DC bus voltage in Volts)
Byte  8  : vbus_hi        (high byte of uint16_t DC bus voltage in Volts)
Byte  9  : iq_lo          (low  byte of int16_t Iq in milliAmps)
Byte 10  : iq_hi          (high byte of int16_t Iq in milliAmps)
Byte 11  : id_lo          (low  byte of int16_t Id in milliAmps)
Byte 12  : id_hi          (high byte of int16_t Id in milliAmps)
Byte 13  : mcsdk_state    (internal MCSDK state, see §4.2)
Byte 14  : XOR checksum   (byte1 ^ byte2 ^ ... ^ byte13)
```

#### Field summary

| Field | Type | Range / Notes |
|-------|------|---------------|
| `speed` | `int16_t` | Signed RPM. Positive = forward, negative = reverse. |
| `esc_state` | `uint8_t` | ESC state (0–6), see §4.1 |
| `faults` | `uint8_t` | Lower byte of MCSDK fault bitmask. 0 = no fault. |
| `u` (cmd_raw) | `int16_t` | Last validated command. Divide by 32767 for float. |
| `vbus` | `uint16_t` | DC bus voltage in whole Volts (e.g. 12 = 12 V). |
| `iq` | `int16_t` | Actual q-axis current in mA. Positive = forward torque. |
| `id` | `int16_t` | Actual d-axis current in mA. Should be ~0 in normal RUN. |
| `mcsdk_state` | `uint8_t` | Internal MCSDK state (0–19+), see §4.2 |

#### Checksum validation

```python
expected = frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ \
           frame[6] ^ frame[7] ^ frame[8] ^ frame[9] ^ frame[10] ^ \
           frame[11] ^ frame[12] ^ frame[13]
valid = (expected == frame[14])
```

#### Python parser example

```python
import serial
import struct

ser = serial.Serial('/dev/ttyAMA0', baudrate=1843200, timeout=0.5)

ESC_STATE_NAMES = {
    0: 'BOOT', 1: 'WAIT_NEUTRAL', 2: 'READY',
    3: 'FORWARD', 4: 'BRAKE', 5: 'REVERSE', 6: 'FAULT'
}

MCSDK_STATE_NAMES = {
    0: 'IDLE', 4: 'START', 6: 'RUN',
    10: 'FAULT_NOW', 11: 'FAULT_OVER', 19: 'SWITCH_OVER'
}

def read_telemetry(ser):
    """Block until a valid telemetry frame is received. Returns a dict."""
    buf = bytearray()
    while True:
        byte = ser.read(1)
        if not byte:
            return None
        if byte[0] == 0xBB:
            rest = ser.read(14)
            if len(rest) < 14:
                continue
            frame = bytearray([0xBB]) + rest
            # Validate checksum
            chk = 0
            for b in frame[1:14]:
                chk ^= b
            if chk != frame[14]:
                continue  # bad frame, keep scanning
            speed   = struct.unpack_from('<h', frame, 1)[0]
            esc_st  = frame[3]
            faults  = frame[4]
            cmd_raw = struct.unpack_from('<h', frame, 5)[0]
            vbus    = struct.unpack_from('<H', frame, 7)[0]
            iq_ma   = struct.unpack_from('<h', frame, 9)[0]
            id_ma   = struct.unpack_from('<h', frame, 11)[0]
            mc_st   = frame[13]
            return {
                'speed_rpm':  speed,
                'esc_state':  ESC_STATE_NAMES.get(esc_st, f'UNKNOWN({esc_st})'),
                'faults':     faults,
                'cmd_u':      cmd_raw / 32767.0,
                'vbus_v':     vbus,
                'iq_ma':      iq_ma,
                'id_ma':      id_ma,
                'mcsdk_state': MCSDK_STATE_NAMES.get(mc_st, f'MCSDK({mc_st})')
            }
```

---

## 4. State Machines

### 4.1 ESC State Machine (Application Layer)

This is what the **host sees** in the `esc_state` telemetry byte. It describes the high-level motor status.

```
                     [Power on]
                          │
                          ▼
                        BOOT (0)
                          │ immediate
                          ▼
                   WAIT_NEUTRAL (1) ◀──────────────────────────┐
                          │                                     │
                   |u| < 0.02 received                   (fault cleared,
                          │                               u is neutral)
                          ▼
                        READY (2)
                       /       \
              u > 0.02           u < -0.02
              (new cmd)          (new cmd)
                 /                    \
                ▼                      ▼
           FORWARD (3)            REVERSE (5)
           │       │              │       │
      |u|<0.02   u<-0.02      |u|<0.02  u>0.02
      timeout     (reversal)  timeout   (reversal)
           │           │          │          │
           ▼           ▼          ▼          ▼
         READY       BRAKE (4)  READY      BRAKE (4)
                       │                    │
                  MCSDK=IDLE           MCSDK=IDLE
                  + u < -0.02         + u > 0.02
                       │                    │
                       ▼                    ▼
                   REVERSE              FORWARD

            ──── Any MCSDK fault ────▶ FAULT (6)
            ◀─── FAULT_OVER + ack ────
```

| State | `esc_state` byte | Meaning |
|-------|-----------------|---------|
| `BOOT` | 0 | One-shot init. Never seen in telemetry in practice. |
| `WAIT_NEUTRAL` | 1 | **ESC is locked.** Must receive a neutral command (`u ≈ 0`) to unlock. This is the power-on state and also the post-fault state. |
| `READY` | 2 | Motor stopped. Waiting for a drive command. |
| `FORWARD` | 3 | Motor spinning forward. Torque = `u × 12 A`. |
| `BRAKE` | 4 | Motor coasting to stop. Triggered by direction reversal. |
| `REVERSE` | 5 | Motor spinning reverse. Torque = `u × 12 A` (u is negative). |
| `FAULT` | 6 | MCSDK hardware fault. Motor stopped. Clears automatically on `FAULT_OVER`. |

### 4.2 MCSDK Internal State (`mcsdk_state` byte)

This is the low-level FOC state machine inside the ST Motor Control SDK.

| Value | Name | Meaning |
|-------|------|---------|
| 0 | `IDLE` | Motor stopped. Ready to start. |
| 4 | `START` | Open-loop rev-up in progress (~7 s). |
| 6 | `RUN` | Closed-loop FOC active. Torque mode engaged. |
| 10 | `FAULT_NOW` | Active hardware fault. |
| 11 | `FAULT_OVER` | Fault condition gone; ESC auto-acknowledges. |
| 19 | `SWITCH_OVER` | 100 ms transition from open-loop to closed-loop. |

---

## 5. Motor Startup Sequence

Understanding the startup sequence is critical — the motor does not spin instantly.

### 5.1 Step-by-step what happens when you send a forward command

```
Host sends u > 0.02
        │
        ▼ (ESC: READY → FORWARD)
MCSDK: IDLE → ICLWAIT → OFFSET_CALIB → ALIGNMENT → START
        │
        │   ~7 seconds of open-loop rev-up
        │   (motor hums and accelerates to 2500 RPM target)
        │
        ▼ (MCSDK_STATE = 19: SWITCH_OVER)
STO observer locks onto back-EMF signal
        │   100 ms blend window; speed may dip 400–600 RPM
        │
        ▼ (MCSDK_STATE = 6: RUN)
ESC activates torque mode:
  Iq = u × 12 A  (programmed every ~1 ms)
Motor speed now determined by load, not a fixed target.
```

> **Key rule:** The motor **will not** respond to throttle changes during `START` (MCSDK state 4).
> Only after `RUN` (MCSDK state 6) does `u` directly command torque.

### 5.2 Minimum operating speed

The sensorless observer requires **at least ±2500 RPM** to maintain lock. Below this speed the BEMF signal is too weak. This means:

- The motor cannot be used for very slow creep — it will either be in open-loop rev-up or running at ≥2500 RPM closed-loop.
- If the motor is physically stalled (wheel blocked), the observer loses lock → FAULT.

### 5.3 Direction reversal (BRAKE state)

You cannot reverse direction instantaneously. The sequence is:

1. Host sends opposite-sign command (e.g. was forward, now `u < -0.02`).
2. ESC stops the motor → enters `BRAKE` state.
3. ESC waits for MCSDK to reach `IDLE` (motor fully coasted).
4. ESC starts rev-up in the new direction → `REVERSE` state.
5. ~7 s later, closed-loop torque mode resumes.

**Total reversal time: approximately 7–15 seconds** depending on motor deceleration and rev-up duration.

---

## 6. Host Software Requirements

### 6.1 Startup handshake (mandatory)

At power-on, the ESC **initialises `esc_cmd_value` to `0x7FFF` (non-neutral)** and waits in `WAIT_NEUTRAL`. The motor will **not** run until the host sends at least one valid neutral frame.

**Your host must send `u = 0.0` at startup before anything else.**

```python
# Mandatory startup sequence
ser = serial.Serial('/dev/ttyAMA0', baudrate=1843200, timeout=0.1)
send_command(0.0)   # unlock the ESC from WAIT_NEUTRAL
time.sleep(0.1)     # give the ESC one telemetry cycle to confirm READY
```

### 6.2 Command rate

- Send commands at **10–50 Hz** (every 20–100 ms).
- The ESC has a **500 ms timeout watchdog**: if no valid frame arrives for 500 ms, the motor stops and ESC returns to `READY`.
- At 10 Hz you have a comfortable 50% margin. At 50 Hz you have 25× margin.

### 6.3 Fault recovery

Faults are self-clearing: the ESC auto-acknowledges `FAULT_OVER` and transitions back. The host does not need to send a special reset frame.

However after a fault the ESC goes to `WAIT_NEUTRAL` (if joystick is neutral) — so the host must send `u = 0.0` again before driving.

**Recommended host logic:**
```python
if telemetry['esc_state'] == 'WAIT_NEUTRAL':
    send_command(0.0)   # re-send neutral to unlock
```

---

## 7. Telemetry Interpretation Reference

### 7.1 Normal forward drive sequence

| Time | `esc_state` | `mcsdk_state` | `speed_rpm` | `iq_ma` |
|------|------------|---------------|-------------|---------|
| t=0 (boot) | WAIT_NEUTRAL | IDLE | 0 | 0 |
| t=0.1 (after neutral) | READY | IDLE | 0 | 0 |
| t=0.2 (after u=0.7) | FORWARD | START | 0→500 | ~2000 |
| t=7 (rev-up peak) | FORWARD | SWITCH_OVER | ~2100 | ~3000 |
| t=7.1 (locked) | FORWARD | RUN | ~2500 | ~8400 (0.7×12A) |

### 7.2 Fault byte decoding (MCSDK bitmask, lower byte)

| Bit | Fault |
|-----|-------|
| 0 | Duration (rev-up timeout) |
| 1 | Over-voltage |
| 2 | Under-voltage |
| 3 | Over-temperature |
| 4 | Overcurrent (hardware comparator) |
| 5 | Speed feedback loss (observer unlock) |
| 6 | Software overcurrent |
| 7 | BEMF inconsistency |

The most common fault in sensorless operation is **bit 5 (speed feedback loss)** — the observer lost lock, usually due to stall or very low speed.

---

## 8. Operational Limits

These limits are hardware-verified on smooth hard floor (2026-03-12):

| Parameter | Value | Notes |
|-----------|-------|-------|
| Min closed-loop speed | ±2500 RPM | Below this: observer cannot maintain lock |
| Max Iq command | 12 A | `|u|=1.0` maps to 12 A |
| Rev-up duration | ~7 s | Must hold throttle through the full open-loop phase |
| SWITCH_OVER speed dip | 400–600 RPM | Normal during observer handoff |
| Communication timeout | 500 ms | Motor stops if no command received |
| Telemetry rate | ~10 Hz | One 15-byte frame every 100 ms |

---

## 9. Minimal Host Implementation Checklist

To successfully control the ESC your host code must:

- [ ] Open serial port at **1843200 baud, 8N1**, no flow control.
- [ ] On startup, **send `u = 0.0`** to pass the WAIT_NEUTRAL gate.
- [ ] Send command frames at **10–50 Hz** continuously while driving.
- [ ] **Hold throttle** through the full ~7 s rev-up (do not drop to neutral during START).
- [ ] Parse telemetry frames (SOF = `0xBB`, 15 bytes, XOR checksum on bytes 1–13).
- [ ] Monitor `esc_state` — react to `WAIT_NEUTRAL` by re-sending neutral.
- [ ] Monitor `faults` byte — log or surface any non-zero value.
- [ ] Gracefully handle direction reversal delay (~7–15 s total).
- [ ] Do **not** send commands faster than 200 Hz — the UART FIFO is 8 bytes and the ESC processes one frame per ~1 ms task cycle.

---

## 10. Quick Reference: Frame Byte Tables

### Command Frame (5 bytes, Host → ESC)

| Byte | Value | Description |
|------|-------|-------------|
| 0 | `0xAA` | SOF |
| 1 | `cmd & 0xFF` | int16 command, low byte |
| 2 | `(cmd >> 8) & 0xFF` | int16 command, high byte |
| 3 | `0x00` | Reserved |
| 4 | `byte1 ^ byte2 ^ byte3` | XOR checksum |

### Telemetry Frame (15 bytes, ESC → Host)

| Byte | Value | Type | Description |
|------|-------|------|-------------|
| 0 | `0xBB` | — | SOF |
| 1–2 | `speed_rpm` | `int16_t LE` | Signed RPM |
| 3 | `esc_state` | `uint8_t` | ESC state (0–6) |
| 4 | `faults` | `uint8_t` | MCSDK fault bitmask (lower byte) |
| 5–6 | `cmd_raw` | `int16_t LE` | Active command (−32768..+32767) |
| 7–8 | `vbus_v` | `uint16_t LE` | DC bus voltage in Volts |
| 9–10 | `iq_ma` | `int16_t LE` | Actual Iq in milliAmps |
| 11–12 | `id_ma` | `int16_t LE` | Actual Id in milliAmps |
| 13 | `mcsdk_state` | `uint8_t` | MCSDK internal state |
| 14 | checksum | `uint8_t` | XOR of bytes 1–13 |
