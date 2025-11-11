# ---------- main.py ----------
import math, time
from machine import Pin, PWM, I2C, ADC
from ads1x15 import ADS1015

# ===== CONSTANTS =====
ArmSE = 155 # Length of shoulder-to-elbow link (mm)
ArmEW = 155 # Length of elbow-to-wrist link (mm)
ShoulderX, ShoulderY = -50, 139.5  # shoulder offset from paper origin

ServoFreq = 50 # servo PWM frequency (Hz)
ServoMinPulse = 500    # pulse at 0 degrees (typical 500–600 µs)
ServoMaxPulse = 2500   # pulse at 180 degrees (typical 2400–2500 µs)

paper_x = 214
paper_y = 278
y_divider = 234
x_divider = 304

# ===== Servo Setup =====
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)

pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# Potentiometers
pot_x = ADC(0)
pot_y = ADC(1)

def read_potentiometer(pot):
     duty = pot.read_u16()    # read potentionmeter value (0–65535)
     return duty

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

def translate(angle):
	"""
	Converts an angle in degrees to the corresponding input
	for the duty_u16 method of the servo class
	"""
	MIN = 1638 # 0 degrees
	MAX = 8192 # 180 degrees
	DEG = (MAX - MIN) / 180 # value per degree

	# clamp angle to be between 0 and 180
	angle = max(0, min(180, angle))

	return int(angle * DEG + MIN)

def set_servo_deg(pwm, angle_deg):
    pwm.duty_u16(translate(angle_deg))

while True:
    set_servo_deg(pwm_shoulder, 179)
    set_servo_deg(pwm_elbow, 1)

# while True:

#     cx = read_potentiometer(pot_x)
#     cy = read_potentiometer(pot_y)

#     cx = cx / x_divider
#     cy = cy / y_divider
#     alpha, beta = inverse_kinematics(cx,cy)

#     s_angle, e_angle = servo_angles(alpha,beta)

#     set_servo_deg(pwm_shoulder, s_angle)
#     set_servo_deg(pwm_elbow, e_angle)

