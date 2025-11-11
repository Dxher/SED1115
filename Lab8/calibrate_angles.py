import json, time
from machine import Pin, PWM, I2C
from ads1x15 import ADS1015

# Constants
ServoFreq = 50 # Hz
ServoMinPulse = 500 # µs at 0°
ServoMaxPulse = 2500 # µs at 180°

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
adc = ADS1015(i2c, address=0x48, gain=1) # index (1 => ±4.096V in your driver)

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

def sweep_joint(joint_name, pwm_this, pwm_other, AIN):
    """
    joint_name: "shoulder" | "elbow"
    pwm_this: PWM for the joint being swept
    pwm_other: PWM for the other joint (held at 90)
    AIN: 0 -> use AIN0; 1 -> use AIN1
    """
    print("\n[Calibrate] Sweeping", joint_name)
    # Park other joint
    set_servo_deg(pwm_other, 90)
    time.sleep_ms(400)

    angles = []
    volts  = []

    # 0 -> 180 sweep
    for angle in range(0, 181, STEP_DEG):
        set_servo_deg(pwm_this, angle)
        time.sleep_ms(60)
        # take multiple samples and average
        v = []
        for _ in range(NSAMPLES):
            v0, v1 = read_feedback_volts()
            v.append(v0 if AIN == 0 else v1)
            time.sleep_ms(10)
        vavg = average(v)
        angles.append(angle)
        volts.append(vavg)
        print("%s: %3ddegrees  ->  %.3f V" % (joint_name[0].upper(), angle, vavg))

    # basic stats
    v_min = min(volts); v_max = max(volts)
    # If voltage generally rises with angle, slope > 0; else it’s reversed
    # Compute simple correlation sign using end points
    reverse = (volts[-1] - volts[0]) < 0

    return {
        "name": joint_name,
        "reverse": reverse,
        "v_min": 0.52,
        "v_max": 2.48,
        "sweep_step_deg": STEP_DEG,
        "samples_per_point": NSAMPLES,
        "points": [{"angle": a, "v": round(v, 5)} for a, v in zip(angles, volts)]
    }

def write_file(data):
    with open("/calibration.json", "w") as f:
        f.write(json.dumps(data))
    print("\nSaved /calibration.json")

def main():
    # Move to safe neutral
    set_servo_deg(pwm_shoulder, 90)
    set_servo_deg(pwm_elbow, 90)
    time.sleep_ms(600)

    # Test all angles for shoulder and elbow
    shoulder = sweep_joint("shoulder", pwm_shoulder, pwm_elbow, AIN=0)
    elbow    = sweep_joint("elbow", pwm_elbow, pwm_shoulder, AIN=1)

    # Write the values to json file
    data = {
        "shoulder":  shoulder,
        "elbow":     elbow,
    }
    write_file(data)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
