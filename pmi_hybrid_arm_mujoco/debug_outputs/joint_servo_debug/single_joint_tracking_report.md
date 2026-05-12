# Single Joint Tracking Report

- Can each joint track +/-0.2 and +/-0.5? see per-case errors below.
- jnt1 step +0.20: final=0.25667, steady=0.25750, sat=0, wrong_dir=True
- jnt1 step -0.20: final=-0.14339, steady=-0.14256, sat=0, wrong_dir=False
- jnt1 step +0.50: final=0.55658, steady=0.55754, sat=89, wrong_dir=True
- jnt1 step -0.50: final=-0.44186, steady=-0.44258, sat=0, wrong_dir=False
- jnt2 step +0.20: final=0.01470, steady=0.01495, sat=0, wrong_dir=False
- jnt2 step -0.20: final=0.00000, steady=0.00000, sat=0, wrong_dir=False
- jnt2 step +0.50: final=0.20333, steady=0.20324, sat=0, wrong_dir=False
- jnt2 step -0.50: final=0.00000, steady=0.00000, sat=3, wrong_dir=False
- jnt3 step +0.20: final=0.04506, steady=0.04531, sat=0, wrong_dir=False
- jnt3 step -0.20: final=0.00035, steady=0.00139, sat=362, wrong_dir=False
- jnt3 step +0.50: final=0.33315, steady=0.33314, sat=0, wrong_dir=False
- jnt3 step -0.50: final=0.00015, steady=0.00131, sat=444, wrong_dir=False
- jnt4 step +0.20: final=0.01720, steady=0.01316, sat=457, wrong_dir=False
- jnt4 step -0.20: final=0.00000, steady=-0.00000, sat=0, wrong_dir=False
- jnt4 step +0.50: final=0.01734, steady=0.01336, sat=485, wrong_dir=False
- jnt4 step -0.50: final=-0.00527, steady=-0.00507, sat=0, wrong_dir=False

- largest steady-state error: jnt1 step 0.5 (|e_ss|=0.55754)
- wrong direction cases: 2
- jnt1 failing while others work? yes
