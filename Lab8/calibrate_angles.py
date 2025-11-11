# ---------- calibrate_angles.py ----------
# Sweep each servo 0→180° to record measured angles.

import time
from machine import Pin, PWM

# ===== Configuration =====
SERVO_FREQ_HZ = 50
SERVO_MIN_US, SERVO_MAX_US = 500, 2500
NEUTRAL_SHOULDER, NEUTRAL_ELBOW = 90.0, 90.0

# ===== PWM setup =====
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(SERVO_FREQ_HZ)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(SERVO_FREQ_HZ)