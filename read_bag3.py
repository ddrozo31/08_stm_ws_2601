"""
read_bag3.py  --  Parse ESC rosbag2 with extended telemetry (iq_ma, id_ma, mcsdk_state).
Usage: edit BAG path below or pass as arg.
"""

import struct, math, collections, sys
from mcap.reader import make_reader

BAG = sys.argv[1] if len(sys.argv) > 1 else (
    r"C:\01_Fixed\mrad_ws_2601_zulu\src\zulu_esc\bag_file"
    r"\rosbag2_2026_03_11-15_24_29\rosbag2_2026_03_11-15_24_29_0.mcap"
)

TOPICS = {
    "/esc/command", "/esc/speed_rpm", "/esc/state", "/esc/mcsdk_state",
    "/esc/faults",  "/esc/vbus_v",   "/esc/iq_ma", "/esc/id_ma",
    "/imu/data_raw",
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
    off = 4
    off += 4 + 4
    fid_len = struct.unpack_from("<I", data, off)[0]
    off += 4
    off += (fid_len + 3) & ~3
    off += (4 + 9) * 8
    off += (3 + 9) * 8
    ax, ay, az = struct.unpack_from("<ddd", data, off)
    return ax, ay, az

DESER = {
    "/esc/command":    cdr_float32,
    "/esc/speed_rpm":  cdr_int16,
    "/esc/state":      cdr_string,
    "/esc/mcsdk_state":cdr_string,
    "/esc/faults":     cdr_string,
    "/esc/vbus_v":     cdr_float32,
    "/esc/iq_ma":      cdr_int16,
    "/esc/id_ma":      cdr_int16,
    "/imu/data_raw":   cdr_imu,
}

records = collections.defaultdict(list)

with open(BAG, "rb") as f:
    reader = make_reader(f)
    for schema, channel, message in reader.iter_messages(topics=list(TOPICS)):
        topic = channel.topic
        t_sec = message.log_time / 1e9
        try:
            val = DESER[topic](message.data)
        except Exception:
            continue
        records[topic].append((t_sec, val))

t0 = min(v[0][0] for v in records.values() if v)
for topic in records:
    records[topic] = [(t - t0, v) for t, v in records[topic]]

dur = max(v[-1][0] for v in records.values() if v)
print(f"\n=== Bag: {BAG.split(chr(92))[-1]}  duration={dur:.1f} s ===")
for t in sorted(records):
    print(f"  {t:<25}  n={len(records[t])}")

def nearest(series, t):
    if not series:
        return None
    return min(series, key=lambda x: abs(x[0]-t))[1]

spd_data   = records.get("/esc/speed_rpm",   [])
cmd_data   = records.get("/esc/command",     [])
state_data = records.get("/esc/state",       [])
mc_data    = records.get("/esc/mcsdk_state", [])
iq_data    = records.get("/esc/iq_ma",       [])
id_data    = records.get("/esc/id_ma",       [])
imu_data   = records.get("/imu/data_raw",    [])
flt_data   = records.get("/esc/faults",      [])

# ---- IMU summary -------------------------------------------------------------
if imu_data:
    ay_vals = [v[1] for _, v in imu_data]
    ax_vals = [v[0] for _, v in imu_data]
    print(f"\n=== IMU linear_acceleration (n={len(imu_data)}) ===")
    print(f"  ax: min={min(ax_vals):+7.3f}  max={max(ax_vals):+7.3f}  m/s^2")
    print(f"  ay: min={min(ay_vals):+7.3f}  max={max(ay_vals):+7.3f}  m/s^2")
    print(f"  ay > 1.0 m/s^2 count: {sum(1 for v in ay_vals if abs(v)>1.0)}"
          f"  ({100*sum(1 for v in ay_vals if abs(v)>1.0)/len(ay_vals):.1f}%)")

# ---- Iq summary (exclude IDLE garbage values >5000 mA) ----------------------
if iq_data:
    iq_motor = [(t, v) for t, v in iq_data if abs(v) < 5000]
    iq_vals  = [v for _, v in iq_motor]
    id_vals  = [v for _, v in id_data if abs(v) < 5000] if id_data else []
    print(f"\n=== Iq/Id while motor active (<5A filter, n={len(iq_motor)}) ===")
    if iq_vals:
        print(f"  iq_ma: min={min(iq_vals):6d}  max={max(iq_vals):6d}  mean={sum(iq_vals)/len(iq_vals):.0f} mA")
    if id_vals:
        print(f"  id_ma: min={min(id_vals):6d}  max={max(id_vals):6d}  mean={sum(id_vals)/len(id_vals):.0f} mA")

# ---- MCSDK state transitions -------------------------------------------------
print(f"\n=== MCSDK state transitions ===")
prev_mc = None
for t, mc in mc_data:
    if mc != prev_mc:
        spd = nearest(spd_data, t)
        iq  = nearest(iq_data,  t)
        u   = nearest(cmd_data, t)
        print(f"  t={t:6.2f}s  {prev_mc or '---':12} -> {mc:<12}  spd={spd:5d} RPM  iq={iq:6d} mA  u={u:+.3f}")
        prev_mc = mc

# ---- ESC state transitions ---------------------------------------------------
print(f"\n=== ESC state transitions ===")
prev_st = None
for t, st in state_data:
    if st != prev_st:
        spd = nearest(spd_data, t)
        iq  = nearest(iq_data,  t)
        u   = nearest(cmd_data, t)
        mc  = nearest(mc_data,  t)
        flt = nearest(flt_data, t)
        print(f"  t={t:6.2f}s  {prev_st or '---':14} -> {st:<14}  spd={spd:5d}  iq={iq:6d} mA  u={u:+.3f}  mc={mc}  faults={flt}")
        prev_st = st

# ---- Fault events ------------------------------------------------------------
faults_seen = set(v for _, v in flt_data)
print(f"\n=== Faults seen: {faults_seen} ===")
prev_flt = None
for t, flt in flt_data:
    if flt != prev_flt and flt != 'none':
        spd = nearest(spd_data, t)
        iq  = nearest(iq_data,  t)
        u   = nearest(cmd_data, t)
        mc  = nearest(mc_data,  t)
        print(f"  t={t:6.2f}s  FAULT: {flt:<20}  spd={spd:5d}  iq={iq:6d} mA  u={u:+.3f}  mc={mc}")
        prev_flt = flt

# ---- Did observer ever reach RUN? -------------------------------------------
run_entries = [(t, v) for t, v in mc_data if v == "RUN"]
print(f"\n=== RUN entries: {len(run_entries)} ===")
for t, _ in run_entries[:10]:
    spd = nearest(spd_data, t)
    iq  = nearest(iq_data, t)
    u   = nearest(cmd_data, t)
    print(f"  t={t:.2f}s  spd={spd} RPM  iq={iq} mA  u={u:+.3f}")

# ---- Speed histogram during START state --------------------------------------
start_speeds = [nearest(spd_data, t) for t, mc in mc_data if mc == "START"]
start_speeds = [s for s in start_speeds if s is not None]
if start_speeds:
    print(f"\n=== Speed samples while in START state: n={len(start_speeds)} ===")
    buckets = {}
    for s in start_speeds:
        b = (abs(s) // 500) * 500
        buckets[b] = buckets.get(b, 0) + 1
    for b in sorted(buckets):
        bar = "#" * (buckets[b] // 2)
        print(f"  {b:5d}-{b+499} RPM: {buckets[b]:4d}  {bar}")

# ---- Combined timeline -------------------------------------------------------
print(f"\n=== Combined timeline (every 5th sample) ===")
print(f"  {'t':>6}  {'u':>6}  {'spd':>6}  {'esc_state':<14}  {'mc_state':<12}  {'iq_ma':>7}  {'ay':>7}")
print("  " + "-"*72)
step = max(1, len(spd_data) // 60)
for t, spd in spd_data[::step]:
    u   = nearest(cmd_data,   t)
    st  = nearest(state_data, t)
    mc  = nearest(mc_data,    t)
    iq  = nearest(iq_data,    t)
    imu_val = nearest(imu_data, t)
    ay  = imu_val[1] if imu_val else 0.0
    print(f"  {t:6.2f}  {u:+6.3f}  {spd:6d}  {str(st):<14}  {str(mc):<12}  {iq:7d}  {ay:+7.3f}")

print()