#!/usr/bin/env python3
"""
analyze_rosbag.py — ESC rosbag analysis tool (custom FOC + IMU).

Usage:
    python3 tests/analyze_rosbag.py [bag_path]

If bag_path is omitted, uses the most recent bag in bag_file/.
Supports both MCSDK and custom FOC telemetry.

Update this script when new telemetry fields or analysis needs arise.
"""

import sys
import os
import math
import collections
from pathlib import Path

# ROS 2 Python path
sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from std_msgs.msg import Int16, String, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
BAG_ROOT = Path(__file__).parent.parent / 'bag_file'

TOPIC_TYPES = {
    '/esc/speed_rpm':   Int16,
    '/esc/iq_ma':       Int16,
    '/esc/id_ma':       Int16,
    '/esc/vbus_v':      Float32,
    '/esc/command':     Float32,
    '/esc/state':       String,
    '/esc/faults':      String,
    '/esc/cfoc_state':  String,
    '/esc/mcsdk_state': String,
    '/imu/data_raw':    Imu,
    '/cmd_vel_joy':     TwistStamped,
}


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

        if topic == '/imu/data_raw':
            # Extract gyro Z (yaw rate) and accel X (forward accel)
            data[topic].append((t_s, msg.angular_velocity.z,
                                msg.linear_acceleration.x,
                                msg.linear_acceleration.y,
                                msg.linear_acceleration.z))
        elif topic == '/cmd_vel_joy':
            data[topic].append((t_s, msg.twist.linear.x, msg.twist.angular.z))
        else:
            data[topic].append((t_s, msg.data))

    return dict(data)


def stats(values):
    if not values:
        return None
    return dict(n=len(values), avg=sum(values)/len(values),
                mn=min(values), mx=max(values))


def window_vals(data, topic, t_start, t_end, idx=1):
    """Extract values from topic within a time window. idx=column index in tuple."""
    return [row[idx] for row in data.get(topic, []) if t_start <= row[0] <= t_end]


def state_transitions(series):
    """Return list of (time, from_state, to_state) from a state series."""
    transitions = []
    prev_val = None
    for t, v in series:
        if v != prev_val:
            transitions.append((t, prev_val, v))
            prev_val = v
    return transitions


def find_motion_segments(data):
    """Find segments where motor is running (CFOC not IDLE and speed != 0).
    Returns list of (t_start, t_end, direction)."""
    cfoc = data.get('/esc/cfoc_state', [])
    esc = data.get('/esc/state', [])

    segments = []
    seg_start = None
    seg_dir = None

    for t, v in esc:
        if v in ('FORWARD', 'REVERSE') and seg_start is None:
            seg_start = t
            seg_dir = v
        elif v not in ('FORWARD', 'REVERSE') and seg_start is not None:
            segments.append((seg_start, t, seg_dir))
            seg_start = None
            seg_dir = None

    if seg_start is not None:
        last_t = esc[-1][0]
        segments.append((seg_start, last_t, seg_dir))

    return segments


# ---------------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------------

def analyze(bag_path: Path):
    print(f'\n{"="*70}')
    print(f'Bag: {bag_path.name}')
    print(f'{"="*70}')

    data = load_bag(bag_path)

    if not data:
        print('No recognised topics found.')
        return

    # Determine mode: custom FOC or MCSDK
    is_cfoc = '/esc/cfoc_state' in data
    is_mcsdk = '/esc/mcsdk_state' in data
    has_imu = '/imu/data_raw' in data

    # Duration
    all_times = []
    for series in data.values():
        if series:
            all_times.append(series[-1][0])
    total_t = max(all_times) if all_times else 0
    print(f'Duration: {total_t:.1f} s')
    print(f'Mode: {"Custom FOC" if is_cfoc else "MCSDK"}')
    print(f'IMU: {"Yes" if has_imu else "No"}')

    # -- Topic sample counts ---------------------------------------------------
    print(f'\n--- Topic counts ---')
    for topic in sorted(data.keys()):
        print(f'  {topic}: {len(data[topic])} samples')

    # -- ESC state transitions -------------------------------------------------
    esc_series = data.get('/esc/state', [])
    if esc_series:
        esc_trans = state_transitions(esc_series)
        print(f'\n--- ESC state transitions ({len(esc_trans)}) ---')
        for t, frm, to in esc_trans:
            print(f'  t={t:7.2f}s  {frm} -> {to}')

    # -- CFOC state transitions ------------------------------------------------
    cfoc_series = data.get('/esc/cfoc_state', [])
    if cfoc_series:
        cfoc_trans = state_transitions(cfoc_series)
        print(f'\n--- CFOC state transitions ({len(cfoc_trans)}) ---')
        cl_reached = False
        for t, frm, to in cfoc_trans:
            marker = ''
            if to == 'CLOSED_LOOP':
                marker = '  *** CLOSED_LOOP ***'
                cl_reached = True
            elif to == 'FAULT':
                marker = '  *** FAULT ***'
            print(f'  t={t:7.2f}s  {frm} -> {to}{marker}')
        print(f'  CLOSED_LOOP reached: {cl_reached}')

    # -- Fault events ----------------------------------------------------------
    faults = [(t, v) for t, v in data.get('/esc/faults', []) if v != 'none']
    if faults:
        print(f'\n--- Faults ({len(faults)} samples with faults) ---')
        prev_f = None
        for t, v in faults:
            if v != prev_f:
                print(f'  t={t:.2f}s  {v}')
                prev_f = v
    else:
        print(f'\n--- Faults: NONE ---')

    # -- Command overview ------------------------------------------------------
    cmd = data.get('/esc/command', [])
    if cmd:
        cmd_vals = [v for _, v in cmd]
        nonzero = [v for v in cmd_vals if abs(v) > 0.02]
        print(f'\n--- Command (u) ---')
        print(f'  Total samples: {len(cmd_vals)}')
        print(f'  Non-zero commands: {len(nonzero)}')
        if nonzero:
            print(f'  Range: [{min(nonzero):+.3f}, {max(nonzero):+.3f}]')
            print(f'  Avg (non-zero): {sum(nonzero)/len(nonzero):+.3f}')

    # -- Motion segments -------------------------------------------------------
    segments = find_motion_segments(data)
    if segments:
        print(f'\n--- Motion segments ({len(segments)}) ---')
        for i, (ts, te, direction) in enumerate(segments):
            dur = te - ts
            spd = window_vals(data, '/esc/speed_rpm', ts, te)
            iq = window_vals(data, '/esc/iq_ma', ts, te)
            cmd_seg = window_vals(data, '/esc/command', ts, te)
            spd_st = stats(spd)
            iq_st = stats(iq)
            cmd_st = stats(cmd_seg)

            # Find time to reach CLOSED_LOOP within this segment
            cl_time = None
            for t, v in cfoc_series:
                if ts <= t <= te and v == 'CLOSED_LOOP':
                    cl_time = t - ts
                    break

            print(f'\n  Segment #{i+1}: {direction} [{ts:.2f}s - {te:.2f}s] ({dur:.2f}s)')
            if cl_time is not None:
                print(f'    Startup time (→CLOSED_LOOP): {cl_time:.2f}s')
            if cmd_st:
                print(f'    Command u:  avg={cmd_st["avg"]:+.3f}  [{cmd_st["mn"]:+.3f}, {cmd_st["mx"]:+.3f}]')
            if spd_st:
                print(f'    Speed RPM:  avg={spd_st["avg"]:+.0f}  [{spd_st["mn"]:+}, {spd_st["mx"]:+}]')
            if iq_st:
                print(f'    Iq (mA):    avg={iq_st["avg"]:+.0f}  [{iq_st["mn"]:+}, {iq_st["mx"]:+}]')

            # Id (should be near zero in FOC)
            id_seg = window_vals(data, '/esc/id_ma', ts, te)
            id_st = stats(id_seg)
            if id_st:
                print(f'    Id (mA):    avg={id_st["avg"]:+.0f}  [{id_st["mn"]:+}, {id_st["mx"]:+}]')

            # IMU during this segment
            if has_imu:
                gyro_z = window_vals(data, '/imu/data_raw', ts, te, idx=1)
                accel_x = window_vals(data, '/imu/data_raw', ts, te, idx=2)
                accel_y = window_vals(data, '/imu/data_raw', ts, te, idx=3)
                gz_st = stats(gyro_z)
                ax_st = stats(accel_x)
                ay_st = stats(accel_y)
                if gz_st:
                    print(f'    Gyro Z (rad/s): avg={gz_st["avg"]:+.3f}  [{gz_st["mn"]:+.3f}, {gz_st["mx"]:+.3f}]')
                if ax_st:
                    print(f'    Accel X (m/s²): avg={ax_st["avg"]:+.2f}  [{ax_st["mn"]:+.2f}, {ax_st["mx"]:+.2f}]')
                if ay_st:
                    print(f'    Accel Y (m/s²): avg={ay_st["avg"]:+.2f}  [{ay_st["mn"]:+.2f}, {ay_st["mx"]:+.2f}]')

            # Closed-loop steady-state (after startup, if we have enough data)
            if cl_time is not None and dur > cl_time + 1.0:
                ss_start = ts + cl_time + 0.5  # skip 0.5s after CL entry
                spd_ss = window_vals(data, '/esc/speed_rpm', ss_start, te)
                iq_ss = window_vals(data, '/esc/iq_ma', ss_start, te)
                cmd_ss = window_vals(data, '/esc/command', ss_start, te)
                if spd_ss and len(spd_ss) > 3:
                    spd_ss_st = stats(spd_ss)
                    iq_ss_st = stats(iq_ss)
                    cmd_ss_st = stats(cmd_ss)
                    print(f'    --- Steady-state (after CL+0.5s) ---')
                    if cmd_ss_st:
                        print(f'    Command u:  avg={cmd_ss_st["avg"]:+.3f}  [{cmd_ss_st["mn"]:+.3f}, {cmd_ss_st["mx"]:+.3f}]')
                    if spd_ss_st:
                        print(f'    Speed RPM:  avg={spd_ss_st["avg"]:+.0f}  [{spd_ss_st["mn"]:+}, {spd_ss_st["mx"]:+}]')
                    if iq_ss_st:
                        print(f'    Iq (mA):    avg={iq_ss_st["avg"]:+.0f}  [{iq_ss_st["mn"]:+}, {iq_ss_st["mx"]:+}]')

    # -- Speed vs Command proportionality check --------------------------------
    if cmd and '/esc/speed_rpm' in data:
        print(f'\n--- Speed vs Command mapping ---')
        # Collect pairs where CFOC is in CLOSED_LOOP
        cl_windows = []
        in_cl = False
        cl_start = None
        for t, v in cfoc_series:
            if v == 'CLOSED_LOOP' and not in_cl:
                in_cl = True
                cl_start = t
            elif v != 'CLOSED_LOOP' and in_cl:
                in_cl = False
                cl_windows.append((cl_start, t))
        if in_cl:
            cl_windows.append((cl_start, cfoc_series[-1][0]))

        # Build time-aligned pairs
        speed_dict = {}
        for t, v in data['/esc/speed_rpm']:
            speed_dict[round(t, 1)] = v
        cmd_dict = {}
        for t, v in data['/esc/command']:
            cmd_dict[round(t, 1)] = v

        # Collect by command bucket
        buckets = collections.defaultdict(list)
        for tw_start, tw_end in cl_windows:
            for t_key in speed_dict:
                if tw_start + 0.5 <= t_key <= tw_end and t_key in cmd_dict:
                    u = cmd_dict[t_key]
                    if abs(u) > 0.02:
                        # Bucket by u rounded to 0.1
                        bucket = round(u, 1)
                        buckets[bucket].append(speed_dict[t_key])

        if buckets:
            print(f'  {"u":>6s}  {"avg_RPM":>8s}  {"min_RPM":>8s}  {"max_RPM":>8s}  {"n":>4s}  {"expected":>9s}  {"error%":>7s}')
            for u_val in sorted(buckets.keys()):
                speeds = buckets[u_val]
                avg_s = sum(speeds) / len(speeds)
                expected = u_val * 5000
                error_pct = ((avg_s - expected) / expected * 100) if expected != 0 else 0
                print(f'  {u_val:+6.1f}  {avg_s:+8.0f}  {min(speeds):+8}  {max(speeds):+8}  {len(speeds):4d}  {expected:+9.0f}  {error_pct:+6.1f}%')

    # -- IMU overview ----------------------------------------------------------
    if has_imu:
        imu = data['/imu/data_raw']
        gyro_z_all = [row[1] for row in imu]
        accel_x_all = [row[2] for row in imu]
        accel_y_all = [row[3] for row in imu]
        accel_z_all = [row[4] for row in imu]
        print(f'\n--- IMU overview ({len(imu)} samples) ---')
        gz = stats(gyro_z_all)
        ax = stats(accel_x_all)
        ay = stats(accel_y_all)
        az = stats(accel_z_all)
        if gz:
            print(f'  Gyro Z  (rad/s): avg={gz["avg"]:+.4f}  [{gz["mn"]:+.3f}, {gz["mx"]:+.3f}]')
        if ax:
            print(f'  Accel X (m/s²):  avg={ax["avg"]:+.2f}  [{ax["mn"]:+.2f}, {ax["mx"]:+.2f}]')
        if ay:
            print(f'  Accel Y (m/s²):  avg={ay["avg"]:+.2f}  [{ay["mn"]:+.2f}, {ay["mx"]:+.2f}]')
        if az:
            print(f'  Accel Z (m/s²):  avg={az["avg"]:+.2f}  [{az["mn"]:+.2f}, {az["mx"]:+.2f}]')

        # Detect significant motion events (large accel or yaw)
        motion_events = []
        for t, gz_v, ax_v, ay_v, az_v in imu:
            if abs(gz_v) > 0.5 or abs(ax_v) > 3.0 or abs(ay_v) > 3.0:
                motion_events.append((t, gz_v, ax_v, ay_v))
        if motion_events:
            print(f'\n  Significant motion events (|gyro_z|>0.5 or |accel_xy|>3.0): {len(motion_events)}')
            # Group consecutive events
            groups = []
            grp_start = motion_events[0][0]
            grp_end = motion_events[0][0]
            for t, gz_v, ax_v, ay_v in motion_events[1:]:
                if t - grp_end < 1.0:  # within 1s = same group
                    grp_end = t
                else:
                    groups.append((grp_start, grp_end))
                    grp_start = t
                    grp_end = t
            groups.append((grp_start, grp_end))
            print(f'  Motion event groups: {len(groups)}')
            for gi, (gs, ge) in enumerate(groups):
                evts = [(t, gz_v, ax_v, ay_v) for t, gz_v, ax_v, ay_v in motion_events if gs <= t <= ge]
                max_gz = max(abs(e[1]) for e in evts)
                max_ax = max(abs(e[2]) for e in evts)
                max_ay = max(abs(e[3]) for e in evts)
                print(f'    Group {gi+1}: [{gs:.2f}s - {ge:.2f}s] ({ge-gs:.2f}s)  '
                      f'peak |gyro_z|={max_gz:.2f} |accel_x|={max_ax:.2f} |accel_y|={max_ay:.2f}')

    # -- Summary ---------------------------------------------------------------
    print(f'\n--- Summary ---')
    if is_cfoc:
        cl_count = sum(1 for _, _, to in state_transitions(cfoc_series) if to == 'CLOSED_LOOP')
        fault_count = sum(1 for _, _, to in state_transitions(cfoc_series) if to == 'FAULT')
        print(f'  CLOSED_LOOP entries: {cl_count}')
        print(f'  FAULT entries: {fault_count}')
    print(f'  Motion segments: {len(segments)}')
    if not faults:
        print(f'  Fault-free run: YES')
    else:
        print(f'  Fault-free run: NO ({len(faults)} fault samples)')
    print()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if len(sys.argv) > 1:
        bag_path = Path(sys.argv[1])
    else:
        bag_path = find_latest_bag()

    # If a directory is given, analyze it; if it contains multiple test bags, analyze all
    if bag_path.is_dir() and (bag_path / 'metadata.yaml').exists():
        analyze(bag_path)
    elif bag_path.is_dir():
        # Directory of bags — analyze all
        bags = sorted(bag_path.glob('rosbag2_*'))
        if not bags:
            print(f'No bags found in {bag_path}')
        for b in bags:
            if (b / 'metadata.yaml').exists():
                analyze(b)
    else:
        print(f'Not a valid bag: {bag_path}')
