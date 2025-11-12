import json, time
from machine import Pin, PWM, I2C
from ads1x15 import ADS1015


# X Config
OUT_PATH = "/calibration_data_JIG123.txt"

# Constants
ServoFreq = 50 # Hz

# Sweep settings
STEP_DEG   = 5      # angle step for sweep
NSAMPLES   = 6      # samples per point

# Servos
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# Feedback
i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
adc = ADS1015(i2c, address=0x48, gain=1)

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
    # Give the servo (pwm) an angle (angle_deg) to go to
    pwm.duty_u16(translate(angle_deg))

def read_feedback_volts():
    # Get the feedback from the servos to compare with our values
    raw0 = adc.read(rate=4, channel1=0)  # AIN0 = shoulder
    raw1 = adc.read(rate=4, channel1=1)  # AIN1 = elbow
    return adc.raw_to_v(raw0), adc.raw_to_v(raw1)

def average(samples):
    # Return the average of a group of data
    return sum(samples) / len(samples) if samples else 0.0

def volts_to_angle(volts):
    """Convert feedback volts to actual angle (linear model per channel)."""
    v0   = 0.51
    v180 = 2.79
    if abs(v180 - v0) < 1e-6:
        return 0.0  # avoid div-by-zero; update CAL!
    slope = 180.0 / (v180 - v0)
    return max(0.0, min(180.0, slope * (volts - v0)))

# --------------- Sweep & record ----------------
def sweep_joint(joint_name, pwm_this, pwm_other, AIN):
    """
    Sweep desired angle 0..180 and record (desired, actual_from_volts, error).
    AIN: 0 for AIN0 (shoulder), 1 for AIN1 (elbow)
    """
    print("\n[Calibrate] Sweeping", joint_name)
    # Park the other joint at neutral
    set_servo_deg(pwm_other, 90)
    time.sleep_ms(400)

    rows = []  # list of dicts per angle

    for desired in range(0, 181, STEP_DEG):
        set_servo_deg(pwm_this, desired)
        time.sleep_ms(60)

        # average N samples from selected channel
        samples = []
        for _ in range(NSAMPLES):
            v0, v1 = read_feedback_volts()
            samples.append(v0 if AIN == 0 else v1)
            time.sleep_ms(10)
        mean_v = average(samples)

        actual = volts_to_angle(mean_v)
        error  = actual - desired

        rows.append({
            "desired_deg": desired,
            "actual_deg":  round(actual, 3),
            "error_deg":   round(error, 3),
            "mean_volts":  round(mean_v, 5),
        })
        print("%s: %3ddeg -> %.3f V -> actual %.2f deg  error %+6.2f deg"
            % (joint_name[0].upper(), desired, mean_v, actual, error))


    return {
        "name": joint_name,
        "sweep_step_deg": STEP_DEG,
        "samples_per_point": NSAMPLES,
        "rows": rows
    }

def write_text_file(shoulder, elbow):
    with open(OUT_PATH, "w") as f:
        # Header
        f.write("CALIBRATION DATA — Jig123\n")

        def section(name, rows):
            f.write("[%s]\n" % name)
            f.write("[Calibrate] desired_deg  actual_deg  error_deg  mean_volts\n")
            for r in rows:
                d = r["desired_deg"]
                a = r["actual_deg"]
                e = r["error_deg"]
                v = r["mean_volts"]
                a_txt = "NA" if a is None else f"{a:.3f}"
                e_txt = "NA" if e is None else f"{e:.3f}"
                v_txt = "NA" if v is None else f"{v:.5f}"
                f.write(f"{d:3d}  {a_txt:>7}  {e_txt:>7}  {v_txt}\n")
            f.write("\n")

        # pass the *row lists*, not the dicts
        section("shoulder", shoulder["rows"])
        section("elbow",    elbow["rows"])

    print("\nSaved %s" % OUT_PATH)

def main():
    # Move to safe neutral
    set_servo_deg(pwm_shoulder, 90)
    set_servo_deg(pwm_elbow, 90)
    time.sleep_ms(600)

    # Test all angles for shoulder and elbow
    shoulder = sweep_joint("shoulder", pwm_shoulder, pwm_elbow, AIN=0)
    elbow    = sweep_joint("elbow", pwm_elbow, pwm_shoulder, AIN=1)

    write_text_file(shoulder, elbow)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
