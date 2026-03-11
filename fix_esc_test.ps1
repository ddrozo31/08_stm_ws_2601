$f = 'c:/01_Fixed/08_stm_ws_2601/tests/esc_test.py'
$c = [System.IO.File]::ReadAllText($f)

# Frame length
$c = $c.Replace('TLM_FRAME_LEN = 10', 'TLM_FRAME_LEN = 15')

# MCSDK state names
$mcsdk_names = @'

# MCSDK internal state (MCI_State_t)
MCSDK_STATE_NAMES = {
    0:  'IDLE',
    4:  'START',
    6:  'RUN',
    10: 'FAULT_NOW',
    11: 'FAULT_OVER',
    12: 'ICLWAIT',
}

def decode_mcsdk_state(b: int) -> str:
    return MCSDK_STATE_NAMES.get(b, f'MC_{b}')

'@

$c = $c.Replace(
    '# -- Frame builders / decoders -------------------------------------------------',
    $mcsdk_names + '# -- Frame builders / decoders -------------------------------------------------'
)

# parse_telemetry: new checksum and return value
$c = $c.Replace(
'def parse_telemetry(frame: bytes):
    """Return (speed_rpm, state_str, fault_str, u_float, vbus_v) or None on bad checksum."""
    if len(frame) != TLM_FRAME_LEN or frame[0] != TLM_SOF:
        return None
    chk = frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5] ^ frame[6] ^ frame[7] ^ frame[8]
    if chk != frame[9]:
        return None
    speed  = struct.unpack_from(''<h'', frame, 1)[0]   # int16 RPM
    cmd_r  = struct.unpack_from(''<h'', frame, 5)[0]   # int16 raw command
    vbus   = struct.unpack_from(''<H'', frame, 7)[0]   # uint16 Volts
    u_val  = cmd_r / 32767.0
    return speed, decode_state(frame[3]), decode_faults(frame[4]), u_val, vbus',
'def parse_telemetry(frame: bytes):
    """Return (speed_rpm, state_str, fault_str, u_float, vbus_v, iq_a, id_a, mcsdk_str)
    or None on bad checksum."""
    if len(frame) != TLM_FRAME_LEN or frame[0] != TLM_SOF:
        return None
    chk = 0
    for i in range(1, 14):
        chk ^= frame[i]
    if chk != frame[14]:
        return None
    speed  = struct.unpack_from(''<h'', frame, 1)[0]   # int16 RPM
    cmd_r  = struct.unpack_from(''<h'', frame, 5)[0]   # int16 raw command
    vbus   = struct.unpack_from(''<H'', frame, 7)[0]   # uint16 Volts
    iq_ma  = struct.unpack_from(''<h'', frame, 9)[0]   # int16 milliAmps
    id_ma  = struct.unpack_from(''<h'', frame, 11)[0]  # int16 milliAmps
    u_val  = cmd_r / 32767.0
    return (speed, decode_state(frame[3]), decode_faults(frame[4]),
            u_val, vbus, iq_ma / 1000.0, id_ma / 1000.0,
            decode_mcsdk_state(frame[13]))'
)

# reader_thread print
$c = $c.Replace(
'            speed, state, faults, u_val, vbus = result
            print(f"\r  [TLM]  spd={speed:6d} RPM  state={state:<14}"
                  f"  u={u_val:+.3f}  vbus={vbus:3d} V  faults={faults}",
                  flush=True)',
'            speed, state, faults, u_val, vbus, iq_a, id_a, mc_st = result
            print(f"\r  [TLM]  spd={speed:6d} RPM  state={state:<14}"
                  f"  u={u_val:+.3f}  vbus={vbus:3d} V"
                  f"  iq={iq_a:+.3f}A  id={id_a:+.3f}A"
                  f"  mc={mc_st}  faults={faults}",
                  flush=True)'
)

[System.IO.File]::WriteAllText($f, $c)
Write-Host 'esc_test.py updated'