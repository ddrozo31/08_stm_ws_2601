#!/usr/bin/env python3
"""
analyze_cl_transition.py — Correlate RPM, Iq, cmd, IMU around each CL entry.

For each CROSSFADE→CLOSED_LOOP event, prints a ±2.5s window showing:
  time | cfoc_state | cmd_u | speed_rpm | iq_ma | accel_x | accel_y | gyro_z
Helps diagnose grinding/lurch at EKF lock.
"""

import sys
import struct
from pathlib import Path
from mcap.reader import make_reader
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

WINDOW_PRE  = 2.0   # seconds before CL entry to show
WINDOW_POST = 2.5   # seconds after CL entry to show

CFOC_NAMES = {0:'IDLE',1:'ALIGN',2:'OPEN_LOOP',3:'XFADE',4:'CL',5:'FAULT'}

def find_mcap(bag_path):
    p = Path(bag_path)
    if p.is_dir():
        files = list(p.glob('*.mcap'))
        if files: return str(files[0])
    return bag_path

def load_all(bag_path):
    """Load all messages into per-topic lists of (t_sec, msg)."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    TYPE_MAP = {
        'std_msgs/msg/Int16':   'std_msgs/msg/Int16',
        'std_msgs/msg/Float32': 'std_msgs/msg/Float32',
        'std_msgs/msg/String':  'std_msgs/msg/String',
        'sensor_msgs/msg/Imu':  'sensor_msgs/msg/Imu',
        'geometry_msgs/msg/TwistStamped': 'geometry_msgs/msg/TwistStamped',
    }

    data = {}
    mcap_file = find_mcap(bag_path)

    with open(mcap_file, 'rb') as f:
        reader = make_reader(f)
        schema_map = {}
        for schema in reader.get_summary().schemas.values():
            schema_map[schema.id] = schema.name

        channel_map = {}
        for ch in reader.get_summary().channels.values():
            channel_map[ch.id] = (ch.topic, schema_map.get(ch.schema_id,''))

        t0 = None
        for schema, channel, message in reader.iter_messages():
            topic, msg_type = channel_map.get(channel.id, ('',''))
            t_ns = message.log_time
            t_s  = t_ns * 1e-9
            if t0 is None: t0 = t_s
            t_rel = t_s - t0

            try:
                msg_class = get_message(msg_type)
                msg = deserialize_message(message.data, msg_class)
            except Exception:
                continue

            if topic not in data:
                data[topic] = []
            data[topic].append((t_rel, msg))

    return data

def interpolate(series, t, field):
    """Get value of field at time t by nearest-neighbor."""
    if not series: return None
    best = min(series, key=lambda x: abs(x[0]-t))
    msg = best[1]
    if field == 'data':
        return getattr(msg, 'data', None)
    if field == 'linear_x':
        return msg.twist.linear.x
    if field == 'accel_x':
        return msg.linear_acceleration.x
    if field == 'accel_y':
        return msg.linear_acceleration.y
    if field == 'gyro_z':
        return msg.angular_velocity.z
    return None

def get_nearest(series, t, tol=0.2):
    """Return (t_actual, msg) nearest to t within tol seconds, or None."""
    if not series: return None
    best = min(series, key=lambda x: abs(x[0]-t))
    if abs(best[0]-t) > tol: return None
    return best

def analyze_transitions(bag_path):
    print(f"\n{'='*72}")
    print(f"Transition analysis: {Path(bag_path).name}")
    print(f"{'='*72}")

    data = load_all(bag_path)

    cfoc  = data.get('/esc/cfoc_state', [])
    speed = data.get('/esc/speed_rpm', [])
    iq    = data.get('/esc/iq_ma', [])
    cmd   = data.get('/esc/command', [])
    imu   = data.get('/imu/data_raw', [])

    # Find all CROSSFADE→CLOSED_LOOP transitions
    cl_entries = []
    for i in range(1, len(cfoc)):
        prev_name = cfoc[i-1][1].data
        curr_name = cfoc[i][1].data
        if prev_name == 'CROSSFADE' and curr_name == 'CLOSED_LOOP':
            cl_entries.append(cfoc[i][0])

    if not cl_entries:
        print("  No CROSSFADE→CLOSED_LOOP transitions found.")
        return

    print(f"  Found {len(cl_entries)} CL entries: "
          + ", ".join(f"t={t:.2f}s" for t in cl_entries))

    for entry_idx, t_cl in enumerate(cl_entries):
        print(f"\n{'─'*72}")
        print(f"  CL Entry #{entry_idx+1} at t={t_cl:.2f}s  "
              f"(window: {t_cl-WINDOW_PRE:.2f}s → {t_cl+WINDOW_POST:.2f}s)")
        print(f"{'─'*72}")
        header = (f"  {'t_rel':>7} {'Δt_CL':>7} {'state':<10} "
                  f"{'cmd_u':>7} {'RPM':>7} {'Iq_A':>7} "
                  f"{'accel_x':>8} {'accel_y':>8} {'gyro_z':>8}")
        print(header)
        print(f"  {'-'*7} {'-'*7} {'-'*10} {'-'*7} {'-'*7} {'-'*7} "
              f"{'-'*8} {'-'*8} {'-'*8}")

        # Build unified timeline from all available topics in window
        t_start = t_cl - WINDOW_PRE
        t_end   = t_cl + WINDOW_POST

        # Collect all unique timestamps from speed topic (10 Hz, good cadence)
        times = sorted(set(
            t for t, _ in speed
            if t_start <= t <= t_end
        ))

        for t in times:
            # state
            cfoc_pt = get_nearest(cfoc, t)
            state   = cfoc_pt[1].data if cfoc_pt else '?'

            # speed
            spd_pt  = get_nearest(speed, t)
            rpm_val = spd_pt[1].data if spd_pt else 0

            # iq
            iq_pt   = get_nearest(iq, t)
            iq_a    = iq_pt[1].data / 1000.0 if iq_pt else 0.0

            # cmd
            cmd_pt  = get_nearest(cmd, t)
            cmd_val = cmd_pt[1].data if cmd_pt else 0.0

            # imu
            imu_pt  = get_nearest(imu, t, tol=0.15)
            ax = imu_pt[1].linear_acceleration.x if imu_pt else 0.0
            ay = imu_pt[1].linear_acceleration.y if imu_pt else 0.0
            gz = imu_pt[1].angular_velocity.z    if imu_pt else 0.0

            delta_t = t - t_cl
            marker = ' ◀ CL' if abs(delta_t) < 0.06 else ''

            print(f"  {t:7.2f} {delta_t:+7.2f} {state:<10} "
                  f"{cmd_val:+7.3f} {rpm_val:+7.0f} {iq_a:+7.2f} "
                  f"{ax:+8.2f} {ay:+8.2f} {gz:+8.3f}{marker}")

    # Summary: for each CL entry, measure RPM drop and Iq jump in first 500ms
    print(f"\n{'─'*72}")
    print("  Per-transition summary (0→+500ms window):")
    print(f"  {'#':>3} {'t_CL':>7} {'RPM@CL':>8} {'RPM+0.5':>8} "
          f"{'ΔRPM':>7} {'Iq@CL':>7} {'Iq+0.5':>7} {'ΔIq':>7}")
    print(f"  {'-'*3} {'-'*7} {'-'*8} {'-'*8} {'-'*7} {'-'*7} {'-'*7} {'-'*7}")

    for i, t_cl in enumerate(cl_entries):
        spd_at  = get_nearest(speed, t_cl, tol=0.15)
        spd_p5  = get_nearest(speed, t_cl+0.5, tol=0.2)
        iq_at   = get_nearest(iq,    t_cl, tol=0.15)
        iq_p5   = get_nearest(iq,    t_cl+0.5, tol=0.2)

        rpm0 = spd_at[1].data if spd_at else 0
        rpm5 = spd_p5[1].data if spd_p5 else 0
        iq0  = (iq_at[1].data/1000.0) if iq_at else 0.0
        iq5  = (iq_p5[1].data/1000.0) if iq_p5 else 0.0

        print(f"  {i+1:3} {t_cl:7.2f} {rpm0:+8.0f} {rpm5:+8.0f} "
              f"{rpm5-rpm0:+7.0f} {iq0:+7.2f} {iq5:+7.2f} {iq5-iq0:+7.2f}")


if __name__ == '__main__':
    bags = sys.argv[1:] if len(sys.argv) > 1 else []
    if not bags:
        print("Usage: python3 analyze_cl_transition.py <bag_dir> [bag_dir2 ...]")
        sys.exit(1)
    for bag in bags:
        analyze_transitions(bag)
