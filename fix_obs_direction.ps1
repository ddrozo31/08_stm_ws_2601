$fh = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/drive_parameters.h'
$fc = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Src/mc_app_hooks.c'

# ---- drive_parameters.h: increase TRANSITION_DURATION 200 -> 500 ms --------
$h = [System.IO.File]::ReadAllText($fh)
$h = $h.Replace(
    '#define TRANSITION_DURATION                 200 /* Switch over duration, ms */',
    '#define TRANSITION_DURATION                 500 /* Switch over duration, ms — increased for angle convergence */'
)
[System.IO.File]::WriteAllText($fh, $h)
Write-Host 'drive_parameters.h: TRANSITION_DURATION 200->500'

# ---- mc_app_hooks.c: add direction sanity check in ESC_FORWARD and ESC_REVERSE ----
$c = [System.IO.File]::ReadAllText($fc)

# ESC_FORWARD: replace the torque-mode block
$c = $c.Replace(
'      else if (mci_st == RUN)
      {
        /* Observer locked, closed-loop active: torque mode.
         * Joystick maps directly to Iq — speed is set by the load. */
        (void)MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS);
      }
      /* Still in START (rev-up): speed ramp set at entry, no update needed. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_BRAKE:',
'      else if (mci_st == RUN)
      {
        /* Direction sanity: STO observer can lock at 180-deg wrong angle.
         * If estimated speed sign disagrees with the forward command, the
         * observer has the wrong solution — stop cleanly back to READY so
         * the user can retry immediately without sending neutral first.
         * Otherwise apply torque mode: Iq proportional to joystick. */
        if (speed < -50.0f)
        {
          (void)MC_StopMotor1();
          esc_state = ESC_READY;
        }
        else
        {
          (void)MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS);
        }
      }
      /* Still in START (rev-up): speed ramp set at entry, no update needed. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_BRAKE:'
)

# ESC_REVERSE: replace the torque-mode block
$c = $c.Replace(
'      else if (mci_st == RUN)
      {
        /* Observer locked, closed-loop active: torque mode.
         * u is negative here — negative Iq — reverse torque. */
        (void)MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS);
      }
      /* Still in START (rev-up): speed ramp set at entry, no update needed. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_FAULT:',
'      else if (mci_st == RUN)
      {
        /* Direction sanity: same as FORWARD — if observer locked at wrong
         * angle, estimated speed will be positive despite reverse command.
         * Stop cleanly back to READY for immediate retry. */
        if (speed > 50.0f)
        {
          (void)MC_StopMotor1();
          esc_state = ESC_READY;
        }
        else
        {
          /* u is negative here — negative Iq — reverse torque. */
          (void)MC_ProgramTorqueRampMotor1_F(u * ESC_MAX_IQ_A, ESC_TORQUE_RAMP_MS);
        }
      }
      /* Still in START (rev-up): speed ramp set at entry, no update needed. */
      break;

    /* ---------------------------------------------------------------------- */
    case ESC_FAULT:'
)

[System.IO.File]::WriteAllText($fc, $c)
Write-Host 'mc_app_hooks.c: direction sanity check added to ESC_FORWARD and ESC_REVERSE'