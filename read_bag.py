"""
read_bag.py  --  Parse ESC rosbag2 MCAP and print summary + per-topic data.
Usage: python read_bag.py
"""

import struct, collections
from mcap.reader import make_reader

BAG = (r"C:\Users\ddroz\Downloads\mrad_ws_2601_zulu-feat_rc_car"
       r"\src\zulu_esc\rosbag2_2026_03_11-12_01_42"
       r"\rosbag2_2026_03_11-12_01_42_0.mcap")

TOPICS = {
    "/esc/command",
    "/esc/speed_rpm",
    "/esc/state",
    "/esc/faults",
    "/esc/vbus_v",
}

# CDR deserialisers (all ROS2 CDR: 4-byte header then payload)
def cdr_float32(data):
    return struct.unpack_from("<f", data, 4)[0]

def cdr_int16(data):
    return struct.unpack_from("<h", data, 4)[0]

def cdr_string(data):
    # 4-byte header + 4-byte string length + chars
    length = struct.unpack_from("<I", data, 4)[0]
    return data[8:8+length].rstrip(b'\x00').decode()

DESER = {
    "/esc/command":   cdr_float32,
    "/esc/speed_rpm": cdr_int16,
    "/esc/state":     cdr_string,
    "/esc/faults":    cdr_string,
    "/esc/vbus_v":    cdr_float32,
}

records = collections.defaultdict(list)  # topic -> [(t_sec, value)]

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

# Normalise time to t=0
t0 = min(v[0][0] for v in records.values() if v)
for topic in records:
    records[topic] = [(t - t0, v) for t, v in records[topic]]

# ---- Summary -----------------------------------------------------------------
dur = max(v[-1][0] for v in records.values() if v)
print(f"\n=== Bag summary  duration={dur:.1f} s ===\n")

for topic, rows in sorted(records.items()):
    vals = [v for _, v in rows if isinstance(v, (int, float))]
    if vals:
        print(f"  {topic:<22}  n={len(rows):4d}  "
              f"min={min(vals):8.2f}  max={max(vals):8.2f}  mean={sum(vals)/len(vals):8.2f}")
    else:
        unique = sorted(set(v for _, v in rows))
        print(f"  {topic:<22}  n={len(rows):4d}  values={unique}")

# ---- State transitions -------------------------------------------------------
print("\n=== ESC state timeline ===\n")
prev_state = None
for t, s in records.get("/esc/state", []):
    s = s.strip()
    if s != prev_state:
        print(f"  t={t:6.2f}s  {s}")
        prev_state = s

# ---- Fault events ------------------------------------------------------------
print("\n=== Fault events ===\n")
prev_fault = None
for t, s in records.get("/esc/faults", []):
    s = s.strip()
    if s != prev_fault:
        print(f"  t={t:6.2f}s  {s}")
        prev_fault = s

# ---- Speed trace (sampled every ~2 s) ----------------------------------------
print("\n=== Speed RPM (sampled) ===\n")
spd = records.get("/esc/speed_rpm", [])
step = max(1, len(spd) // 30)
for t, v in spd[::step]:
    bar = "#" * (abs(v) // 200)
    print(f"  t={t:6.2f}s  {v:6d} RPM  {bar}")

# ---- Command trace (sampled) -------------------------------------------------
print("\n=== ESC command u (sampled) ===\n")
cmd = records.get("/esc/command", [])
step = max(1, len(cmd) // 30)
for t, v in cmd[::step]:
    bar = "#" * int(abs(v) * 20)
    sign = "+" if v >= 0 else "-"
    print(f"  t={t:6.2f}s  u={v:+.3f}  {sign}{bar}")

print()