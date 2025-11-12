import math, time
from machine import Pin, PWM, I2C, ADC
from ads1x15 import ADS1015

JIG_ID = "JIG123"                                   # <-- set to your jig ID
CALIB_PATH = "/calibration_data_%s.txt" % JIG_ID    # file produced by calibrate_angles.py
USE_CALIBRATION = True                              # flip to False to compare raw vs compensated


# Constants
ArmSE = 155 # Length of shoulder-to-elbow link (mm)
ArmEW = 155 # Length of elbow-to-wrist link (mm)
ShoulderX, ShoulderY = -50, 139.5  # shoulder offset from paper origin

ServoFreq = 50 # servo PWM frequency (Hz)
ServoMinPulse = 500    # pulse at 0 degrees (typical 500–600 µs)
ServoMaxPulse = 2500   # pulse at 180 degrees (typical 2400–2500 µs)

paper_x_max = 214
paper_y_max = 278
y_divider = 234
x_divider = 304

# Servo setup
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)

pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# Potentiometers
pot_x = ADC(27)
pot_y = ADC(26)

def read_potentiometer(pot):
     duty = pot.read_u16()    # read potentionmeter value (0–65535)
     return duty

def inverse_kinematics(Cx, Cy):
    AC = math.sqrt((ShoulderX - Cx)**2 + (ShoulderY - Cy)**2)
    AbaseC = math.sqrt((ShoulderX - Cx)**2 + Cy**2)
    Angle_BAC = math.acos((ArmSE**2 + AC**2 - ArmEW**2) / (2 * ArmSE * AC))
    Angle_ACB = math.asin((ArmSE * math.sin(Angle_BAC)) / ArmEW)
    Angle_YAC = math.acos((ShoulderY**2 + AC**2 - AbaseC**2) / (2 * ShoulderY * AC))
    alpha = math.degrees(Angle_YAC + Angle_BAC)
    beta = math.degrees(Angle_ACB + Angle_BAC)
    return alpha, beta

def servo_angles(alpha, beta):
    shoulder_angle = alpha - 75
    elbow_angle = 150 - (beta)
    if elbow_angle < 0:
        elbow_angle = 0
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

def calibration_setup(jig_id=JIG_ID, path_template=CALIB_PATH):
    cal = {"shoulder": {"deg": [], "err": []}, "elbow": {"deg": [], "err": []}}
    current = None
    path = path_template
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("CALIBRATION DATA") or line.startswith("step_deg") or line.startswith("columns"):
                    continue
                if line.startswith("[") and line.endswith("]"):
                    name = line[1:-1].strip().lower()
                    if name in cal:
                        current = name
                    else:
                        current = None
                    continue
                if current is None:
                    continue
                # expected row: "<desired>  <actual|NA>  <error|NA>  <volts|NA>"
                parts = line.split()
                if len(parts) < 4:
                    continue
                try:
                    d = int(parts[0])
                    e = None if parts[2].upper() == "NA" else float(parts[2])
                except Exception:
                    continue
                if e is None:
                    continue  # skip rows without an error value
                cal[current]["deg"].append(d)
                cal[current]["err"].append(e)
        # sort by degree just in case
        for joint in ("shoulder", "elbow"):
            z = list(zip(cal[joint]["deg"], cal[joint]["err"]))
            z.sort()
            cal[joint]["deg"] = [d for d, _ in z]
            cal[joint]["err"] = [e for _, e in z]
        # sanity: must have at least two points per joint for interpolation
        for joint in ("shoulder", "elbow"):
            if len(cal[joint]["deg"]) < 2:
                cal[joint] = {"deg": [], "err": []}  # disable this joint’s cal if insufficient
        return cal
    except OSError:
        # File not found or unreadable — return empty tables (calibration disabled)
        return {"shoulder": {"deg": [], "err": []}, "elbow": {"deg": [], "err": []}}

def interpolate(cal, joint, desired_deg):
    """Piecewise-linear interpolation of error at desired_deg. Returns 0.0 if table empty."""
    tbl = cal.get(joint, {"deg": [], "err": []})
    D, E = tbl["deg"], tbl["err"]
    if not D:
        return 0.0
    # clamp outside range to nearest endpoint error
    if desired_deg <= D[0]:
        return E[0]
    if desired_deg >= D[-1]:
        return E[-1]
    # find bracketing points
    # (small tables -> linear scan is fine)
    for i in range(1, len(D)):
        if desired_deg <= D[i]:
            d0, d1 = D[i-1], D[i]
            e0, e1 = E[i-1], E[i]
            # linear interpolation
            t = (desired_deg - d0) / (d1 - d0)
            return e0 + t * (e1 - e0)
    return 0.0  # fallback

def send_compensated_angle(pwm, joint, desired_deg, cal):
    """Look up error at desired_deg, add it, clamp, send."""
    err = interpolate(cal, joint, desired_deg)
    corrected = desired_deg + err
    corrected = max(0.0, min(180.0, corrected))
    set_servo_deg(pwm, corrected)
    return corrected, err


CAL = calibration_setup()

set_servo_deg(pwm_shoulder, 90)
set_servo_deg(pwm_elbow, 90)
time.sleep_ms(600)

while True:

    cx = read_potentiometer(pot_x)
    cy = read_potentiometer(pot_y)

    Cx = cx / x_divider
    Cy = cy / y_divider
    
    alpha, beta = inverse_kinematics(Cx,Cy)
    s_angle, e_angle = servo_angles(alpha,beta)
    
    if USE_CALIBRATION and (CAL["shoulder"]["deg"] and CAL["elbow"]["deg"]):
        s_sent, s_err = send_compensated_angle(pwm_shoulder, "shoulder", s_angle, CAL)
        e_sent, e_err = send_compensated_angle(pwm_elbow,    "elbow",    e_angle, CAL)
        # quick debug: desired -> corrected (+err)
        print("S: desired=%.1f, +err=%.2f => sent=%.1f   |   E: desired=%.1f, +err=%.2f => sent=%.1f"
              % (s_angle, s_err, s_sent, e_angle, e_err, e_sent))
    else:
        # raw (uncalibrated) send
        set_servo_deg(pwm_shoulder, s_angle)
        set_servo_deg(pwm_elbow,    e_angle)
        print("RAW S,E:", s_angle, e_angle)
    
    time.sleep_ms(100)

