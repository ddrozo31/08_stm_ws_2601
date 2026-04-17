"""
analyze_lock_confidence.py -- Step 8-B S1 calibration analyzer.

Compares /esc/lock_kappa and /esc/lock_residual distributions:
  * per CFOC state (so the CL band is isolated from OL/CROSSFADE)
  * per bag (bench vs ground)
  * bucketed into HEALTHY_CL vs STALL_CL subsets within CLOSED_LOOP using
    speed/Iq heuristics, so we can see if the two populations separate.

Usage: python3 analyze_lock_confidence.py <bag.mcap> [<bag.mcap> ...]
"""

import struct, collections, statistics, sys
from pathlib import Path
from mcap.reader import make_reader

TOPICS = {
    "/esc/cfoc_state", "/esc/command", "/esc/speed_rpm",
    "/esc/iq_ma", "/esc/id_ma", "/esc/innov_a",
    "/esc/lock_kappa", "/esc/lock_residual",
    "/esc/faults", "/esc/state", "/imu/data_raw",
}

def cdr_f32(d): return struct.unpack_from("<f", d, 4)[0]
def cdr_i16(d): return struct.unpack_from("<h", d, 4)[0]
def cdr_str(d):
    n = struct.unpack_from("<I", d, 4)[0]
    return d[8:8+n].rstrip(b'\x00').decode()

def cdr_imu(d):
    off = 4
    off += 4 + 4
    fid_len = struct.unpack_from("<I", d, off)[0]
    off += 4
    off += (fid_len + 3) & ~3
    off += (4 + 9) * 8
    off += (3 + 9) * 8
    ax, ay, az = struct.unpack_from("<ddd", d, off)
    return (ax, ay, az)

DESER = {
    "/esc/cfoc_state": cdr_str, "/esc/command": cdr_f32,
    "/esc/speed_rpm": cdr_i16, "/esc/iq_ma": cdr_i16, "/esc/id_ma": cdr_i16,
    "/esc/innov_a": cdr_f32, "/esc/lock_kappa": cdr_f32,
    "/esc/lock_residual": cdr_f32, "/esc/faults": cdr_str, "/esc/state": cdr_str,
    "/imu/data_raw": cdr_imu,
}

def load(bag_path):
    rec = collections.defaultdict(list)
    with open(bag_path, "rb") as f:
        for schema, ch, msg in make_reader(f).iter_messages(topics=list(TOPICS)):
            try:
                rec[ch.topic].append((msg.log_time/1e9, DESER[ch.topic](msg.data)))
            except Exception:
                pass
    if not rec: return rec
    # Anchor time to cfoc_state start (ESC telemetry) so transitions line up with
    # what read_bag3 and raw inspection print; IMU often starts 5+ seconds earlier
    # and would otherwise shift every event.
    anchor = rec["/esc/cfoc_state"][0][0] if rec.get("/esc/cfoc_state") else \
             min(v[0][0] for v in rec.values() if v)
    for t in rec: rec[t] = [(x-anchor, y) for x, y in rec[t]]
    return rec

def nearest(series, t):
    if not series: return None
    return min(series, key=lambda x: abs(x[0]-t))[1]

def pct(xs, p):
    if not xs: return float("nan")
    xs = sorted(xs)
    k = max(0, min(len(xs)-1, int(round(p/100.0*(len(xs)-1)))))
    return xs[k]

def summarize(label, xs, fmt="{:7.4f}"):
    if not xs:
        print(f"  {label:<32} n=0")
        return
    p = [pct(xs, q) for q in (1, 5, 25, 50, 75, 95, 99)]
    s = "  ".join(fmt.format(v) for v in p)
    print(f"  {label:<32} n={len(xs):5d}  "
          f"p01 p05 p25 p50 p75 p95 p99 = {s}  mean={statistics.mean(xs):.4f}")

def analyze(bag_path):
    name = Path(bag_path).parent.name
    print("="*96)
    print(f"BAG: {name}")
    print("="*96)

    r = load(bag_path)
    if not r.get("/esc/lock_kappa"):
        print("  (no lock_kappa topic — skipping)"); return

    kappa = r["/esc/lock_kappa"]
    resid = r["/esc/lock_residual"]
    cfoc  = r.get("/esc/cfoc_state", [])
    spd   = r.get("/esc/speed_rpm", [])
    iq    = r.get("/esc/iq_ma", [])
    u     = r.get("/esc/command", [])

    # Align κ and r samples to nearest cfoc_state / speed / Iq / u
    buckets = collections.defaultdict(list)   # state -> list of (t, kappa, resid, spd, iq, u)
    for (t, k), (_, res) in zip(kappa, resid):
        st  = nearest(cfoc, t) or "UNK"
        sp  = nearest(spd, t) or 0
        iqv = nearest(iq, t) or 0
        uv  = nearest(u, t) or 0.0
        buckets[st].append((t, k, res, sp, iqv, uv))

    print(f"\n--- Samples per CFOC state ---")
    for st in sorted(buckets):
        print(f"  {st:<14} n={len(buckets[st]):5d}")

    print(f"\n--- κ = |Iq|/|ω_e|  (kappa)  per state ---")
    for st in ("OPEN_LOOP", "CROSSFADE", "CLOSED_LOOP"):
        if st in buckets:
            summarize(st, [x[1] for x in buckets[st]])

    print(f"\n--- r = |Vq - (Rs·Iq + Ψf·ω_e)|  (residual)  per state ---")
    for st in ("OPEN_LOOP", "CROSSFADE", "CLOSED_LOOP"):
        if st in buckets:
            summarize(st, [x[2] for x in buckets[st]])

    # Within CLOSED_LOOP, split "likely healthy" vs "likely stall" heuristically:
    #   healthy : |spd| >= 1500 and |iq| < 15000 (locked, not saturated)
    #   stall   : |spd| <  1000 and |iq| > 12000 (torque but no speed = hallucination)
    cl = buckets.get("CLOSED_LOOP", [])
    healthy = [(k, res) for _, k, res, s, i, _ in cl if abs(s) >= 1500 and abs(i) < 15000]
    stall   = [(k, res) for _, k, res, s, i, _ in cl if abs(s) <  1000 and abs(i) > 12000]

    print(f"\n--- CLOSED_LOOP partitioned heuristically ---")
    print(f"  HEALTHY (|spd|>=1500, |iq|<15000): n={len(healthy)}")
    print(f"  STALL   (|spd|< 1000, |iq|>12000): n={len(stall)}")
    if healthy:
        summarize("  kappa HEALTHY", [x[0] for x in healthy])
        summarize("  resid HEALTHY", [x[1] for x in healthy])
    if stall:
        summarize("  kappa STALL",   [x[0] for x in stall])
        summarize("  resid STALL",   [x[1] for x in stall])

    # Every CL -> IDLE event, enriched with IMU motion + full CL-dwell κ/r window.
    imu = r.get("/imu/data_raw", [])
    def window(series, t_lo, t_hi):
        return [(tt, vv) for tt, vv in series if t_lo <= tt <= t_hi]
    def stats(xs):
        if not xs: return (float("nan"),)*4
        return (min(xs), max(xs), statistics.mean(xs),
                statistics.pstdev(xs) if len(xs) > 1 else 0.0)

    # Build state-transition list (contiguous runs only)
    transitions = []
    prev = None
    for t, st in cfoc:
        if st != prev:
            transitions.append((t, st))
            prev = st

    # Find each CL->IDLE event; mark as WRONG_ANGLE_RETRY if next transition
    # is IDLE->ALIGNMENT within 200 ms (auto-retry pattern).
    events = []
    for i, (t, st) in enumerate(transitions):
        if i > 0 and transitions[i-1][1] == "CLOSED_LOOP" and st == "IDLE":
            # Find CL entry time (previous CLOSED_LOOP run start)
            cl_entry = transitions[i-1][0]
            next_tr = transitions[i+1] if i+1 < len(transitions) else (None, None)
            retry = (next_tr[1] == "ALIGNMENT"
                     and next_tr[0] is not None
                     and next_tr[0] - t < 0.2)
            events.append((cl_entry, t, retry))

    print(f"\n--- CL→IDLE events ({len(events)} total) ---")
    print(f"    {'cl_in':>6}  {'cl_out':>6}  {'dwell_s':>7}  {'u@out':>6}  "
          f"{'|ay|max':>8}  {'ay_std':>7}  "
          f"{'κ@out':>6}  {'κ_med_CL':>8}  {'κ_p95_CL':>8}  "
          f"{'r@out':>6}  {'r_med_CL':>8}  {'r_p95_CL':>8}  label")
    for cl_in, t_out, retry in events:
        uv  = nearest(u, t_out) or 0.0
        kv  = nearest(kappa, t_out) or 0.0
        rv  = nearest(resid, t_out) or 0.0
        dwell = t_out - cl_in

        # IMU pre-event window (last 500ms of CL)
        imu_w = window(imu, max(cl_in, t_out-0.5), t_out+0.05)
        ay_vals = [v[1] for _, v in imu_w]
        ay_abs_max = max((abs(a) for a in ay_vals), default=0.0)
        ay_std = stats(ay_vals)[3]

        # κ/r distribution across the ENTIRE CL dwell (up to 50ms before exit
        # to avoid mixing in the transition tail)
        k_w = [v for _, v in window(kappa, cl_in+0.05, t_out-0.05)]
        r_w = [v for _, v in window(resid, cl_in+0.05, t_out-0.05)]
        k_med, k_p95 = (pct(k_w, 50), pct(k_w, 95)) if k_w else (float("nan"), float("nan"))
        r_med, r_p95 = (pct(r_w, 50), pct(r_w, 95)) if r_w else (float("nan"), float("nan"))

        motion = (ay_abs_max > 0.5) or (ay_std > 0.3)
        # Wrong-angle retry requires both the fast IDLE->ALIGNMENT transition AND
        # a non-zero command at exit (firmware only re-commits if user is still
        # asking for motion; u=0 followed by quick re-tap is user behavior).
        if retry and abs(uv) >= 0.05:
            label = "WRONG_ANGLE_RETRY"
        elif motion and abs(uv) < 0.05:
            label = "USER_RELEASE"
        elif motion and abs(uv) >= 0.05:
            label = "OBSERVER_LOSS"
        elif (not motion) and abs(uv) >= 0.05:
            label = "STALL_HALLUCINATION"
        else:
            label = "AMBIGUOUS"
        print(f"    {cl_in:6.2f}  {t_out:6.2f}  {dwell:7.2f}  {uv:+6.3f}  "
              f"{ay_abs_max:8.3f}  {ay_std:7.3f}  "
              f"{kv:6.4f}  {k_med:8.4f}  {k_p95:8.4f}  "
              f"{rv:6.3f}  {r_med:8.3f}  {r_p95:8.3f}  {label}")


if __name__ == "__main__":
    bags = sys.argv[1:] or []
    if not bags:
        print("usage: analyze_lock_confidence.py <bag.mcap> [...]"); sys.exit(1)
    for b in bags:
        analyze(b)
        print()
