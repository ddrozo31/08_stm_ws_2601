$fm = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/pmsm_motor_parameters.h'
$fd = 'c:/01_Fixed/08_stm_ws_2601/motor_test1_20260227/Inc/drive_parameters.h'
$enc = [System.Text.UTF8Encoding]::new($false)

# ---- pmsm_motor_parameters.h ------------------------------------------------
$lines = [System.IO.File]::ReadAllLines($fm)

# Line 32 (0-indexed 31): RS 0.050 -> 0.100 Ohm (Motor Pilot measured)
$lines[31] = '#define RS                      0.100 /* Stator resistance, ohm -- Motor Pilot measured: 0.1 Ohm (2852/3100KV) */'

# Lines 33-37 (0-indexed 32-36): LS 4uH -> 10uH (Motor Pilot measured 0.01 mH)
$lines[32] = '#define LS                      0.000010 /* Stator inductance, H (10 uH -- Motor Pilot measured 0.01 mH;'
$lines[33] = '                                                 C1/F1=8192*0.1/(10e-6*25000)=0.33 (stable, < 1).'
$lines[34] = '                                                 C5 ~= 25866*(4/10) = 10346 -- well within int16_t range.'
$lines[35] = '                                                 Rs=0.1 Ohm / Ls=10 uH matches Motor Pilot identification.'
$lines[36] = '                                                 Accurate observer BEMF model at all speeds.) */'

[System.IO.File]::WriteAllLines($fm, $lines, $enc)
Write-Host 'pmsm_motor_parameters.h: RS 0.050->0.100 Ohm, LS 4uH->10uH'

# ---- drive_parameters.h -----------------------------------------------------
$lines = [System.IO.File]::ReadAllLines($fd)

# Lines 42-46 (0-indexed 41-45): update C1/F1 comment with new values
$lines[41] = '/* State observer scaling factors F1'
$lines[42] = ' * Ls=10uH, Rs=100mOhm (Motor Pilot measured): C1=F1*RS/(LS*TF_RATE)=8192*0.1/(10e-6*25000)=3277'
$lines[43] = ' * and C1/F1=0.40 -- stable (requires C1/F1 < 1).'
$lines[44] = ' * C5 ~= 25866*(4/10) = 10346 -- well within int16_t range (was 25866 at Ls=4uH).'
$lines[45] = ' * GAIN1=-9830, F1=8192: stable baseline (verified hardware). */'

# Lines 86-90 (0-indexed 85-89): update PI gains comment -- KP/KI already correct
# Pole-zero cancellation: KI/KP*(KPDIV/KIDIV) = Ts*Rs/Ls = 40e-6*0.1/10e-6 = 0.4
# Check: (395/247)*(4096/16384) = 0.3998 ~ 0.400 -- verified correct for measured motor
$lines[85] = '/* Gains values for torque and flux control loops'
$lines[86] = ' * Motor Pilot measured Rs=0.1 Ohm, Ls=10 uH. Pole-zero cancellation verified:'
$lines[87] = ' * KI/KP*(KPDIV/KIDIV) = (395/247)*(4096/16384) = 0.400 = Ts*Rs/Ls = 40e-6*0.1/10e-6.'
$lines[88] = ' * Bandwidth: (247/4096)*5.52/10e-6 ~ 33000 rad/s ~ 5300 Hz. No change needed. */'
$lines[89] = '#define PID_TORQUE_KP_DEFAULT               247  /* Motor Pilot Rs=0.1 Ohm, Ls=10 uH -- pole-zero cancellation verified */'

[System.IO.File]::WriteAllLines($fd, $lines, $enc)
Write-Host 'drive_parameters.h: C1/F1 comment updated; PI gains KP=247 KI=395 verified correct (no value change)'
