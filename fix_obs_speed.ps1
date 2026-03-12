$fh = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/drive_parameters.h'
$h = [System.IO.File]::ReadAllText($fh)

# 1. TRANSITION_DURATION: 500 -> 200 (the 500ms was causing PLL to diverge wildly)
$h = $h.Replace(
    '#define TRANSITION_DURATION                 500 /* Switch over duration, ms',
    '#define TRANSITION_DURATION                 200 /* Switch over duration, ms'
)

# 2. OBS_MINIMUM_SPEED_RPM: 1500 -> 2000
#    Force switch-over at higher speed where BEMF is stronger -> better angle convergence
$h = $h.Replace(
    '#define OBS_MINIMUM_SPEED_RPM               1500  /* Lowered 3000->1500: motor runs ~2000 RPM under drivetrain load */',
    '#define OBS_MINIMUM_SPEED_RPM               2000  /* Switch-over at 2000 RPM: more BEMF for reliable angle convergence */'
)

[System.IO.File]::WriteAllText($fh, $h)
Write-Host 'drive_parameters.h: TRANSITION_DURATION 500->200, OBS_MINIMUM_SPEED_RPM 1500->2000'