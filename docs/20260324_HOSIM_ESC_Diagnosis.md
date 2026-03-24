# HOSIM ESC Diagnosis — 2026-03-24

## Context

Same STM32G431 ESC and motor used on the Zulu robot is now installed in a HOSIM RC car
("HOSIM POWER IN YOUR HANDS"). The motor and drivetrain do not respond with the same
ROS 2 node parameters that worked on Zulu.

- ROS 2 node: `esc_node_trigger.py` (workspace: `~/mrad_ws_2601_zulu`)
- Rosbag analyzed: `rosbag2_2026_03_24-13_57_55` (41s, 2 rev-up attempts)

---

## Rosbag Analysis

### Attempt 1 (t = 8.4 – 11.7 s)
- RB held for **3.3 seconds**
- Motor entered START, Iq reached ~6000 mA
- Reported RPM climbed to 762 (open-loop reference, not actual)
- RB released → `MCSDK_8` (ANY_STOP) → IDLE

### Attempt 2 (t = 23.5 – 30.3 s)
- RB held for **6.6 seconds**
- Motor entered START, Iq sustained at 4000–6000 mA throughout
- Reported RPM climbed to 2388 (open-loop reference profile)
- RB released → `MCSDK_8` → IDLE
- **Never transitioned to RUN**

---

## Root Cause: Rotor Stall During Open-Loop Startup

### Key evidence

| Observation | What it means |
|---|---|
| Iq stays at 4000–6000 mA at all reported RPMs | No back-EMF developing — rotor is not actually spinning |
| RPM climbs in a smooth profile (0→30→78→114…→2388) | This is the firmware's internal open-loop reference, not measured shaft speed |
| No sound from motor | Stalled open-loop field: rotor locked, field spinning past it |
| Drivetrain does not move | Confirmed stall |

### Explanation

During START, the MCSDK firmware runs a fixed **5-phase open-loop ramp** regardless of the
ROS command value. It rotates the magnetic field at a programmed speed profile using **5 A**
of current (reduced from 6 A during Zulu development to eliminate grinding noise).

If the drivetrain static friction exceeds the electromagnetic torque at 5 A, the rotor
cannot follow the rotating field. The firmware continues reporting its *reference* speed
as if the motor is spinning, but the shaft is physically stalled.

### Firmware phase profile (current settings for Zulu)

| Phase | Target speed | Duration | Current |
|---|---|---|---|
| 3 | 1000 RPM | 1500 ms | 5 A |
| 4 | 2000 RPM | 1800 ms | 5 A |
| 5 | 2800 RPM | 2000 ms | 5 A |

Total rev-up time: ~7.7 s. SWITCH_OVER (START → RUN) requires ~2400 RPM BEMF lock.

The HOSIM drivetrain is stiffer/heavier than Zulu. 5 A is insufficient to move the rotor.

---

## What Was Ruled Out

- **ROS parameters** (`u_revup`, `u_min_start`, etc.): Not the issue. During START the
  firmware ignores the torque command and runs the phase profile internally.
- **Too-short RB hold time**: Attempt 2 lasted 6.6 s (needed 7.7 s), but even if held
  longer the rotor was stalled — SWITCH_OVER would still fail.
- **Wiring fault**: Iq is high and MCSDK enters START correctly, so the electrical
  connection is intact.

---

## Code Fix Applied

Added missing MCSDK state names to `esc_node_trigger.py` (previously showed as `MCSDK_8`,
`MCSDK_19` in logs/topics):

```python
# Before
MCSDK_STATE_NAMES = {
    0:  'IDLE',
    4:  'START',
    6:  'RUN',
    10: 'FAULT_NOW',
    11: 'FAULT_OVER',
}

# After
MCSDK_STATE_NAMES = {
    0:  'IDLE',
    4:  'START',
    6:  'RUN',
    8:  'ANY_STOP',
    10: 'FAULT_NOW',
    11: 'FAULT_OVER',
    19: 'SWITCH_OVER',
}
```

---

## Required Fix: STM32 Firmware

The fix must be made in the STM32 firmware (`mc_app_hooks.c`), not in the ROS node.

**Increase the open-loop startup current** from **5 A → 6 A or 7 A** in the phase profile.
Note: 6 A was the original value on Zulu — it worked but caused grinding noise.
For HOSIM the higher current is necessary to overcome drivetrain stiction.

---

## Diagnostics Before Reflashing

Do these in order to confirm the root cause:

1. **Free-spin test (motor off):** Spin the HOSIM wheels by hand. If stiff or catching,
   the drivetrain has binding that must be resolved regardless of firmware changes.

2. **Lift test:** Suspend the car so wheels are free. Attempt rev-up. If the motor now
   spins and makes sound → drivetrain friction is the issue. If still silent → check
   motor phase wiring.

3. **Phase wiring check:** Verify all 3 motor phase wires are securely seated in the
   ESC connector. A broken/missing phase causes high Iq with zero torque.

---

## Next Steps

| Priority | Action |
|---|---|
| 1 | Run free-spin and lift tests to isolate mechanical vs. electrical |
| 2 | If mechanical OK: reflash STM32 with startup current 6–7 A |
| 3 | Once RUN is reached: tune `max_speed` and `u_min_run` for HOSIM gear ratio |
