#!/usr/bin/env python3
"""
analyze_rosbag.py — ESC rosbag analysis tool.

Usage:
    python3 tests/analyze_rosbag.py [bag_path]

If bag_path is omitted, uses the most recent bag in bag_file/.
Prints:
  - MCSDK state transitions (with timestamps and durations)
  - Speed timeline summary
  - Per-START block: duration, Phase 5 speed stats, Iq stats
  - Fault events
  - Whether RUN was ever reached

Update this script when new telemetry fields or analysis needs arise.
"""

import sys
import os
import glob
import collections
from pathlib import Path

# ROS 2 Python path
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Int16, String, Float32


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
BAG_ROOT = Path(__file__).parent.parent / 'bag_file'

TOPIC_TYPES = {
    '/esc/mcsdk_state': String,
    '/esc/speed_rpm':   Int16,
    '/esc/iq_ma':       Int16,
    '/esc/id_ma':       Int16,
    '/esc/vbus_v':      Float32,
    '/esc/command':     Float32,
    '/esc/state':       String,
    '/esc/faults':      String,
}

# MCSDK state numbers of interest
MCSDK_SWITCHOVER = 'MCSDK_19'
MCSDK_RUN        = 'RUN'
MCSDK_START      = 'START'

# Rev-up phase durations (ms) — for Phase 5 start estimate
PHASE_DURATIONS_MS = [1200, 1200, 1200, 1800, 3000]  # phases 1-5
REVUP_TOTAL_MS     = sum(PHASE_DURATIONS_MS)
PHASE5_DURATION_S  = PHASE_DURATIONS_MS[4] / 1000.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_latest_bag():
    bags = sorted(BAG_ROOT.glob('rosbag2_*'))
    if not bags:
        raise FileNotFoundError(f'No bags found in {BAG_ROOT}')
    return bags[-1]


def load_bag(bag_path: Path) -> dict:
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id='mcap'),
                ConverterOptions('', ''))

    available = {t.name for t in reader.get_all_topics_and_types()}
    data = collections.defaultdict(list)
    t0 = None

    while reader.has_next():
        topic, raw, ts = reader.read_next()
        if topic not in TOPIC_TYPES:
            continue
        if t0 is None:
            t0 = ts
        t_s = (ts - t0) * 1e-9
        msg_type = TOPIC_TYPES[topic]
        msg = deserialize_message(raw, msg_type)
        data[topic].append((t_s, msg.data))

    return dict(data)


def state_transitions(data):
    series = data.get('/esc/mcsdk_state', [])
    transitions = []
    prev_val = None
    prev_t = None
    for t, v in series:
        if v != prev_val:
            transitions.append((t, prev_val, v))
            prev_val = v
            prev_t = t
    return transitions


def extract_start_blocks(data):
    blocks = []
    in_start = False
    sb = None
    for t, v in data.get('/esc/mcsdk_state', []):
        if v == MCSDK_START and not in_start:
            in_start = True
            sb = t
        elif v != MCSDK_START and in_start:
            in_start = False
            blocks.append((sb, t, v))  # (begin, end, exit_state)
    if in_start:
        last_t = data['/esc/mcsdk_state'][-1][0]
        blocks.append((sb, last_t, '(recording ended)'))
    return blocks


def stats(values):
    if not values:
        return None
    return dict(n=len(values), avg=sum(values)/len(values),
                mn=min(values), mx=max(values))


def window(data, topic, t_start, t_end):
    return [v for t, v in data.get(topic, []) if t_start <= t <= t_end]


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyze(bag_path: Path):
    print(f'\n{"="*60}')
    print(f'Bag: {bag_path.name}')
    print(f'{"="*60}')

    data = load_bag(bag_path)

    if not data:
        print('No recognised topics found.')
        return

    total_t = data['/esc/mcsdk_state'][-1][0] if '/esc/mcsdk_state' in data else 0
    print(f'Duration: {total_t:.1f} s\n')

    # -- State transitions -------------------------------------------------
    transitions = state_transitions(data)
    print('--- MCSDK state transitions ---')
    run_reached = False
    switchover_reached = False
    for t, frm, to in transitions:
        marker = ''
        if to == MCSDK_RUN:
            marker = '  *** RUN REACHED ***'
            run_reached = True
        if to == MCSDK_SWITCHOVER:
            marker = '  *** SWITCH_OVER ***'
            switchover_reached = True
        print(f'  t={t:6.2f}s  {frm} -> {to}{marker}')

    # -- Fault events ------------------------------------------------------
    faults = [(t, v) for t, v in data.get('/esc/faults', []) if v != 'none']
    if faults:
        print('\n--- Faults ---')
        prev_f = None
        for t, v in faults:
            if v != prev_f:
                print(f'  t={t:.2f}s  {v}')
                prev_f = v

    # -- Per-START block analysis ------------------------------------------
    blocks = extract_start_blocks(data)
    print(f'\n--- START block analysis ({len(blocks)} attempts) ---')
    for i, (sb, se, exit_st) in enumerate(blocks):
        dur = se - sb
        # Estimate Phase 5 window (last PHASE5_DURATION_S of the block)
        p5_begin = se - PHASE5_DURATION_S
        p5_end   = se

        spd_p5  = window(data, '/esc/speed_rpm', p5_begin, p5_end)
        iq_p5   = window(data, '/esc/iq_ma',     p5_begin, p5_end)
        spd_all = window(data, '/esc/speed_rpm', sb, se)

        spd_p5_st  = stats(spd_p5)
        iq_p5_st   = stats(iq_p5)
        spd_all_st = stats(spd_all)

        print(f'\n  Attempt #{i+1}: START {sb:.2f}s -> {se:.2f}s  (dur={dur:.2f}s, exit={exit_st})')
        if spd_all_st:
            print(f'    Speed (full block): avg={spd_all_st["avg"]:.0f}  '
                  f'min={spd_all_st["mn"]}  max={spd_all_st["mx"]} RPM')
        if spd_p5_st:
            print(f'    Speed (Phase 5 est): avg={spd_p5_st["avg"]:.0f}  '
                  f'min={spd_p5_st["mn"]}  max={spd_p5_st["mx"]} RPM  (n={spd_p5_st["n"]})')
        if iq_p5_st:
            print(f'    Iq   (Phase 5 est): avg={iq_p5_st["avg"]/1000:.2f}  '
                  f'min={iq_p5_st["mn"]/1000:.2f}  max={iq_p5_st["mx"]/1000:.2f} A')

    # -- Summary -----------------------------------------------------------
    print('\n--- Summary ---')
    print(f'  SWITCH_OVER reached: {switchover_reached}')
    print(f'  RUN reached:         {run_reached}')
    if not run_reached:
        print('  -> Motor never entered closed-loop RUN.')
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) > 1:
        bag_path = Path(sys.argv[1])
    else:
        bag_path = find_latest_bag()

    analyze(bag_path)
