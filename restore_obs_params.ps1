$f = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/drive_parameters.h'
$c = [System.IO.File]::ReadAllText($f)

# 1. VARIANCE_THRESHOLD: 0.99 -> 0.80
$c = $c.Replace(
    '#define VARIANCE_THRESHOLD                  0.99 /*!< Max tolerance: fault only on total observer divergence under drivetrain load */',
    '#define VARIANCE_THRESHOLD                  0.80 /*!< Max tolerance: default proven value */'
)

# 2. GAIN1: -16000 -> -9830 (stable value, same effective gain as -19661 at F1=16384)
$c = $c.Replace(
    '#define GAIN1                               -16000  /* Increased from -9830: faster BEMF tracking at low speed under load */',
    '#define GAIN1                               -9830  /* Original stable value (equiv. -19661 at F1=16384) */'
)

# 3. PLL_KP_GAIN: 1500 -> 638
$c = $c.Replace(
    '#define PLL_KP_GAIN                         1500  /* Increased from 638: faster angle tracking at ~2000 RPM BEMF level */',
    '#define PLL_KP_GAIN                         638  /* Original stable value */'
)

# 4. PLL_KI_GAIN: 60 -> 18
$c = $c.Replace(
    '#define PLL_KI_GAIN                         60  /* Increased from 18: faster angle lock under drivetrain load */',
    '#define PLL_KI_GAIN                         18  /* Original stable value */'
)

# 5. Phase currents: 4.0 -> 1.8 (all 5 phases)
$c = $c.Replace('#define PHASE1_FINAL_CURRENT_A              4.0', '#define PHASE1_FINAL_CURRENT_A              1.8')
$c = $c.Replace('#define PHASE2_FINAL_CURRENT_A              4.0', '#define PHASE2_FINAL_CURRENT_A              1.8')
$c = $c.Replace('#define PHASE3_FINAL_CURRENT_A              4.0', '#define PHASE3_FINAL_CURRENT_A              1.8')
$c = $c.Replace('#define PHASE4_FINAL_CURRENT_A              4.0', '#define PHASE4_FINAL_CURRENT_A              1.8')
$c = $c.Replace('#define PHASE5_FINAL_CURRENT_A              4.0', '#define PHASE5_FINAL_CURRENT_A              1.8')

[System.IO.File]::WriteAllText($f, $c)
Write-Host 'drive_parameters.h restored to stable observer baseline'