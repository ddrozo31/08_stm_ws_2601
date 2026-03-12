$fh = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/drive_parameters.h'
$fp = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/pmsm_motor_parameters.h'

# ---- drive_parameters.h ------------------------------------------------
$h = [System.IO.File]::ReadAllText($fh)

# PHASE1_DURATION: 1200 -> 300 ms  (alignment still happens, just 4x shorter)
$h = $h.Replace(
    '#define PHASE1_DURATION                     1200 /*milliseconds */',
    '#define PHASE1_DURATION                     300 /*milliseconds */'
)

[System.IO.File]::WriteAllText($fh, $h)
Write-Host 'drive_parameters.h: PHASE1_DURATION 1200->300ms'

# ---- pmsm_motor_parameters.h: educated-guess Rs/Ls ----------------------
# 3100 KV RC motor typical: Rs ~40-60 mOhm, Ls ~2-4 uH
# Stability check: C1/F1 = Rs/(Ls*ISR) = 0.050/(3e-6*25000) = 0.67 < 1 -> STABLE
# This is the main reason Ls was inflated to 5uH (Rs=0.1 made it unstable).
# If true Rs is 50mOhm, Ls=3uH is stable and more accurate.
$p = [System.IO.File]::ReadAllText($fp)

$p = $p.Replace(
    '#define RS                                  0.1 /* Ohm */  /* measured: 100mΩ placeholder; true value likely 30-60mΩ for 3100KV motor */',
    '#define RS                                  0.050 /* Ohm */  /* educated guess: 50mOhm typical for 3100KV RC motor */'
)

$p = $p.Replace(
    '#define LS                                  0.000005 /* H */   /* 5uH inflated from true ~1uH: required to keep C1/F1=RS/(LS*ISR)<1 stable */',
    '#define LS                                  0.000003 /* H */   /* 3uH educated guess: stable with Rs=50mOhm (C1/F1=0.67<1) */'
)

[System.IO.File]::WriteAllText($fp, $p)
Write-Host 'pmsm_motor_parameters.h: Rs 0.1->0.050, Ls 5uH->3uH'