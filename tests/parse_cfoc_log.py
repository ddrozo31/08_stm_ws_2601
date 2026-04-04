#!/usr/bin/env python3
"""
Parse a custom FOC debug log dumped from STM32 RAM.

Usage:
  1. In STM32CubeIDE debugger, pause execution after test.
  2. Open Memory view, go to address of `cfoc_log` (find via Expressions).
  3. Right-click → Export Memory…
     - Start: address of cfoc_log
     - Length: cfoc_log_idx × 17  (each entry is 17 bytes)
     - Format: Raw Binary
     - Save as: cfoc_dump.bin
  4. Run:  python3 tests/parse_cfoc_log.py cfoc_dump.bin

Alternative — paste hex from debugger:
  python3 tests/parse_cfoc_log.py --hex cfoc_hex.txt
"""

import struct
import sys
import argparse
import csv
import os

# Must match CFOC_LogEntry_t (packed)
#   uint16_t tick_ms
#   uint8_t  state
#   int16_t  Iq_x100
#   int16_t  Id_x100
#   int16_t  Vq_x100
#   int16_t  Vd_x100
#   int16_t  theta_x10
#   int16_t  ekf_theta_x10
#   int16_t  ekf_rpm
ENTRY_FMT = '<HBhhhhhhh'  # little-endian
ENTRY_SIZE = struct.calcsize(ENTRY_FMT)  # 17 bytes

STATE_NAMES = {
    0: 'IDLE',
    1: 'ALIGNMENT',
    2: 'OPEN_LOOP',
    3: 'CROSSFADE',
    4: 'CLOSED_LOOP',
    5: 'FAULT',
}


def parse_binary(data):
    """Parse raw binary dump into list of dicts."""
    entries = []
    n = len(data) // ENTRY_SIZE
    for i in range(n):
        chunk = data[i * ENTRY_SIZE:(i + 1) * ENTRY_SIZE]
        tick, state, iq, id_, vq, vd, theta, ekf_theta, ekf_rpm = struct.unpack(ENTRY_FMT, chunk)
        entries.append({
            'tick_ms':  tick,
            'state':    STATE_NAMES.get(state, f'?{state}'),
            'Iq_A':     iq / 100.0,
            'Id_A':     id_ / 100.0,
            'Vq_V':     vq / 100.0,
            'Vd_V':     vd / 100.0,
            'theta_deg': theta / 10.0,
            'ekf_deg':  ekf_theta / 10.0,
            'ekf_rpm':  ekf_rpm,
        })
    return entries


def parse_hex_file(path):
    """Parse a text file containing hex bytes (space/newline separated)."""
    with open(path, 'r') as f:
        text = f.read()
    # Strip common prefixes like "0x", addresses, colons
    import re
    # Remove address prefixes like "0x20000100:"
    text = re.sub(r'0x[0-9A-Fa-f]+:\s*', '', text)
    # Extract hex byte pairs
    hex_bytes = re.findall(r'[0-9A-Fa-f]{2}', text)
    data = bytes(int(b, 16) for b in hex_bytes)
    return parse_binary(data)


def print_table(entries):
    """Print entries as a formatted table."""
    header = f"{'tick':>6}  {'state':<12} {'Iq(A)':>7} {'Id(A)':>7} {'Vq(V)':>7} {'Vd(V)':>7} {'θ(°)':>8} {'EKFθ(°)':>8} {'EKF RPM':>8}"
    print(header)
    print('-' * len(header))
    for e in entries:
        print(f"{e['tick_ms']:>6}  {e['state']:<12} {e['Iq_A']:>7.2f} {e['Id_A']:>7.2f} "
              f"{e['Vq_V']:>7.2f} {e['Vd_V']:>7.2f} {e['theta_deg']:>8.1f} "
              f"{e['ekf_deg']:>8.1f} {e['ekf_rpm']:>8}")


def save_csv(entries, path):
    """Save entries to CSV."""
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=entries[0].keys())
        writer.writeheader()
        writer.writerows(entries)
    print(f"Saved {len(entries)} entries to {path}")


def print_summary(entries):
    """Print diagnostic summary."""
    print(f"\n{'='*60}")
    print(f"CFOC Log Summary — {len(entries)} samples")
    print(f"{'='*60}")

    # State transitions
    print("\nState transitions:")
    prev_state = None
    for e in entries:
        if e['state'] != prev_state:
            print(f"  t={e['tick_ms']:>5} ms  →  {e['state']}")
            prev_state = e['state']

    # Peak values
    if entries:
        max_iq = max(abs(e['Iq_A']) for e in entries)
        max_id = max(abs(e['Id_A']) for e in entries)
        max_vq = max(abs(e['Vq_V']) for e in entries)
        max_vd = max(abs(e['Vd_V']) for e in entries)
        print(f"\nPeak |Iq| = {max_iq:.2f} A")
        print(f"Peak |Id| = {max_id:.2f} A")
        print(f"Peak |Vq| = {max_vq:.2f} V")
        print(f"Peak |Vd| = {max_vd:.2f} V")

    # Angle range in OPEN_LOOP
    ol_entries = [e for e in entries if e['state'] == 'OPEN_LOOP']
    if ol_entries:
        thetas = [e['theta_deg'] for e in ol_entries]
        print(f"\nOpen-loop angle range: {min(thetas):.1f}° .. {max(thetas):.1f}°")
        duration = ol_entries[-1]['tick_ms'] - ol_entries[0]['tick_ms']
        print(f"Open-loop duration: {duration} ms")
        ekf_rpms = [e['ekf_rpm'] for e in ol_entries if e['ekf_rpm'] != 0]
        if ekf_rpms:
            print(f"EKF speed during OL: {min(ekf_rpms)}..{max(ekf_rpms)} RPM")
    else:
        print("\n⚠ Never reached OPEN_LOOP state!")

    align_entries = [e for e in entries if e['state'] == 'ALIGNMENT']
    if align_entries:
        duration = align_entries[-1]['tick_ms'] - align_entries[0]['tick_ms']
        peak_id = max(abs(e['Id_A']) for e in align_entries)
        print(f"\nAlignment duration: {duration} ms, peak |Id| = {peak_id:.2f} A")

    # Crossfade info
    xf_entries = [e for e in entries if e['state'] == 'CROSSFADE']
    if xf_entries:
        duration = xf_entries[-1]['tick_ms'] - xf_entries[0]['tick_ms']
        print(f"\nCrossfade duration: {duration} ms")
        # Angle difference at start and end
        start = xf_entries[0]
        end = xf_entries[-1]
        print(f"  Start: θ_foc={start['theta_deg']:.1f}°, θ_ekf={start['ekf_deg']:.1f}° (diff={start['ekf_deg']-start['theta_deg']:.1f}°)")
        print(f"  End:   θ_foc={end['theta_deg']:.1f}°, θ_ekf={end['ekf_deg']:.1f}° (diff={end['ekf_deg']-end['theta_deg']:.1f}°)")
    else:
        print("\n⚠ Never reached CROSSFADE state!")

    # Closed-loop info
    cl_entries = [e for e in entries if e['state'] == 'CLOSED_LOOP']
    if cl_entries:
        duration = cl_entries[-1]['tick_ms'] - cl_entries[0]['tick_ms']
        rpms = [e['ekf_rpm'] for e in cl_entries]
        iqs = [abs(e['Iq_A']) for e in cl_entries]
        print(f"\nClosed-loop duration: {duration} ms")
        print(f"  Speed: {min(rpms)}..{max(rpms)} RPM (mean={sum(rpms)/len(rpms):.0f})")
        print(f"  |Iq|: {min(iqs):.2f}..{max(iqs):.2f} A")
    else:
        print("\n⚠ Never reached CLOSED_LOOP state!")


def main():
    parser = argparse.ArgumentParser(description='Parse CFOC debug log from STM32 RAM dump')
    parser.add_argument('input', help='Binary dump (.bin) or hex text file')
    parser.add_argument('--hex', action='store_true', help='Input is hex text, not binary')
    parser.add_argument('--csv', metavar='FILE', help='Save to CSV file')
    parser.add_argument('--all', action='store_true', help='Print all entries (default: summary + first/last 20)')
    args = parser.parse_args()

    if args.hex:
        entries = parse_hex_file(args.input)
    else:
        with open(args.input, 'rb') as f:
            data = f.read()
        entries = parse_binary(data)

    if not entries:
        print("No entries found!")
        sys.exit(1)

    print_summary(entries)

    if args.all:
        print(f"\nAll {len(entries)} entries:")
        print_table(entries)
    else:
        n = min(20, len(entries))
        print(f"\nFirst {n} entries:")
        print_table(entries[:n])
        if len(entries) > 40:
            print(f"\n  ... ({len(entries) - 40} entries omitted) ...\n")
            print(f"Last 20 entries:")
            print_table(entries[-20:])
        elif len(entries) > n:
            print(f"\nLast {len(entries) - n} entries:")
            print_table(entries[n:])

    if args.csv:
        save_csv(entries, args.csv)


if __name__ == '__main__':
    main()
