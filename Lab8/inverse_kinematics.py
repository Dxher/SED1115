# ---------- main.py ----------
import math, time
from machine import Pin, PWM

# ===== USER EDITS =====
ArmSE = 155 # Length of shoulder-to-elbow link (mm)
ArmEW = 155 # Length of elbow-to-wrist link (mm)
ShoulderX, ShoulderY = -50, 139.5  # shoulder offset from paper origin

ServoFreq = 50 # servo PWM frequency (Hz)
ServoMinPulse = 500    # pulse at 0 degrees (typical 500–600 µs)
ServoMaxPulse = 2500   # pulse at 180 degrees (typical 2400–2500 µs)

# ===== Servo Setup =====
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)

pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# ===== Inverse kinematics =====
def inverse_kinematics(Cx, Cy):
    AC = math.sqrt((ShoulderX - Cx)**2 + (ShoulderY - Cy)**2)
    AbaseC = math.sqrt((ShoulderX - Cx)**2 + Cy**2)
    Angle_BAC = math.acos((ArmSE**2 + AC**2 - ArmEW**2) / (2 * ArmSE * AC))
    Angle_ACB = math.asin((ArmSE * math.sin(Angle_BAC)) / ArmEW)
    Angle_YAC = math.acos((ShoulderY**2 + AC**2 - AbaseC**2) / (2 * ShoulderY * AC))
    alpha = math.degrees(Angle_YAC + Angle_BAC)
    beta = math.degrees(Angle_ACB + Angle_BAC)
    return alpha, beta

# ===== Servo angle mapping =====
def servo_angles(alpha, beta):
    shoulder_angle = alpha - math.degrees(75)
    elbow_angle = math.degrees(150) - (beta)
    return shoulder_angle, elbow_angle
