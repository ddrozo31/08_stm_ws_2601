"""
read_bag2.py  --  Parse ESC rosbag2: ESC signals + IMU linear acceleration.
"""

import struct, math, collections
from mcap.reader import make_reader

BAG = (r"C:\Users\ddroz\Downloads\mrad_ws_2601_zulu-feat_rc_car"
       r"\src\zulu_esc\rosbag2_2026_03_11-12_01_42"
       r"\rosbag2_2026_03_11-12_01_42_0.mcap")

TOPICS = {
    "/esc/command", "/esc/speed_rpm", "/esc/state",
    "/esc/faults",  "/esc/vbus_v",   "/imu/data_raw",
}

# ---- CDR helpers -------------------------------------------------------------

def cdr_float32(data):
    return struct.unpack_from("<f", data, 4)[0]

def cdr_int16(data):
    return struct.unpack_from("<h", data, 4)[0]

def cdr_string(data):
    length = struct.unpack_from("<I", data, 4)[0]
    return data[8:8+length].rstrip(b'\x00').decode()

def cdr_imu(data):
    """Extract linear_acceleration (x,y,z) from sensor_msgs/msg/Imu CDR.
    Layout (little-endian CDR):
      [4]  CDR header
      [4]  stamp.sec
      [4]  stamp.nanosec
      [4]  frame_id length (n)
      [n padded to 4] frame_id
      [32] orientation quat  (4 × float64)
      [72] orientation_cov   (9 × float64)
      [24] angular_velocity  (3 × float64)
      [72] angular_vel_cov   (9 × float64)
      [24] linear_accel      (3 × float64)  ← we want this
    """
    off = 4  # skip CDR header
    off += 4 + 4  # stamp sec + nanosec
    fid_len = struct.unpack_from("<I", data, off)[0]
    off += 4
    off += (fid_len + 3) & ~3  # padded to 4-byte boundary
    # orientation (4 float64) + cov (9 float64)
    off += (4 + 9) * 8
    # angular_velocity (3 float64) + cov (9 float64)
    off += (3 + 9) * 8
    # linear_acceleration (3 float64)
    ax, ay, az = struct.unpack_from("<ddd", data, off)
    return ax, ay, az

DESER = {
    "/esc/command":   cdr_float32,
    "/esc/speed_rpm": cdr_int16,
    "/esc/state":     cdr_string,
    "/esc/faults":    cdr_string,
    "/esc/vbus_v":    cdr_float32,
    "/imu/data_raw":  cdr_imu,
}

records = collections.defaultdict(list)

with open(BAG, "rb") as f:
    reader = make_reader(f)
    for schema, channel, message in reader.iter_messages(topics=list(TOPICS)):
        topic = channel.topic
        t_sec = message.log_time / 1e9
        try:
            val = DESER[topic](message.data)
        except Exception as e:
            continue
        records[topic].append((t_sec, val))

# Normalise time to t=0
t0 = min(v[0][0] for v in records.values() if v)
for topic in records:
    records[topic] = [(t - t0, v) for t, v in records[topic]]

dur = max(v[-1][0] for v in records.values() if v)
print(f"\n=== Bag summary  duration={dur:.1f} s  (ROS2 Jazzy / MCAP) ===\n")

# ---- IMU analysis ------------------------------------------------------------
imu = records.get("/imu/data_raw", [])
if imu:
    ax_vals = [v[0] for _, v in imu]
    ay_vals = [v[1] for _, v in imu]
    az_vals = [v[2] for _, v in imu]
    norm_vals = [math.sqrt(x**2 + y**2 + z**2) for x,y,z in [v for _,v in imu]]

    print(f"IMU linear_acceleration  n={len(imu)}")
    print(f"  ax: min={min(ax_vals):7.3f}  max={max(ax_vals):7.3f}  m/s²")
    print(f"  ay: min={min(ay_vals):7.3f}  max={max(ay_vals):7.3f}  m/s²")
    print(f"  az: min={min(az_vals):7.3f}  max={max(az_vals):7.3f}  m/s²  (gravity axis)")
    print(f"  |a|: min={min(norm_vals):7.3f}  max={max(norm_vals):7.3f}  mean={sum(norm_vals)/len(norm_vals):.3f}")

    # Find gravity axis (highest mean absolute value when stationary)
    print(f"\n  Gravity ≈ {sum(az_vals)/len(az_vals):.2f} m/s² on az → horizontal plane is ax/ay")

    # Lateral (ax) and longitudinal (ay) acceleration timeline
    print("\n=== Longitudinal acceleration ay (car forward/back, sampled) ===\n")
    step = max(1, len(imu) // 40)
    for t, (ax, ay, az) in imu[::step]:
        bar_len = int(abs(ay) / 0.5)
        sign = "+" if ay >= 0 else "-"
        bar = sign + "#" * bar_len
        print(f"  t={t:6.2f}s  ay={ay:+7.3f} m/s²  {bar}")

    # Find peak acceleration events
    print("\n=== Peak acceleration events (|ay| > 1.5 m/s²) ===\n")
    for t, (ax, ay, az) in imu:
        if abs(ay) > 1.5:
            print(f"  t={t:6.2f}s  ay={ay:+7.3f}  ax={ax:+7.3f}  |a|={math.sqrt(ax**2+ay**2+az**2):.3f}")

# ---- ESC + IMU combined timeline ---------------------------------------------
print("\n=== Combined timeline: command / speed / state / ay ===\n")
print(f"  {'t(s)':>6}  {'u':>6}  {'spd RPM':>8}  {'state':<14}  {'ay m/s²':>8}")
print("  " + "-"*60)

# Build aligned timeline from speed (10 Hz reference)
spd_data  = records.get("/esc/speed_rpm", [])
cmd_data  = records.get("/esc/command",   [])
state_data= records.get("/esc/state",     [])
imu_data  = records.get("/imu/data_raw",  [])

def nearest(series, t):
    if not series:
        return None
    return min(series, key=lambda x: abs(x[0]-t))[1]

for t, spd in spd_data:
    u   = nearest(cmd_data,   t)
    st  = nearest(state_data, t)
    imu_val = nearest(imu_data, t)
    ay  = imu_val[1] if imu_val else 0.0
    if isinstance(u, float) and isinstance(st, str):
        print(f"  {t:6.2f}  {u:+6.3f}  {spd:8d}  {st:<14}  {ay:+8.3f}")

print()