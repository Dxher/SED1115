#done
import time
from machine import Pin, PWM, I2C
from ads1x15 import ADS1015

# Constants
ServoFreq = 50  # Hz

# Converting voltage to actual angle - These will be updated when the code first runs
# Voltage at 0 degrees
shoulder_min_volts = 0.0
elbow_min_volts = 0.0
# Voltage at 180 degrees
shoulder_max_volts = 2.5
elbow_max_volts = 2.5

# Sweep settings
STEP_DEG = 10 # Angle degree step (10-degree increments)

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

def voltage_to_angle(voltage, v_min, v_max):
    """
    Convert feedback voltage to actual angle (0-180 degrees)
    *The calculation was retrieved from GPT*
    **Assumes linear relationship between voltage and angle**
    """
    if v_max - v_min == 0:
        raise Exception("Invalid voltage range (v_max == v_min)")
    else:
        angle = (voltage - v_min) / (v_max - v_min) * 180.0
        # Clamp to valid range
        return max(0.0, min(180.0, angle))

def read_actual_angles():
    """
    Read feedback voltages and convert to actual angles
    Returns (shoulder_angle, elbow_angle) in degrees
    """
    v_shoulder, v_elbow = read_feedback_volts()
    
    angle_shoulder = voltage_to_angle(v_shoulder, shoulder_min_volts, shoulder_max_volts)
    angle_elbow = voltage_to_angle(v_elbow, elbow_min_volts, elbow_max_volts)
    
    return angle_shoulder, angle_elbow

def calibrate_servo(servo_name, pwm):
    """
    Calibrate a single servo across its range
    Returns list of desired_angle, actual_angle and error
    """
    print(f"\nCalibrating {servo_name} servo...")
    print("Desired\tActual\tError")
    
    results = []
    set_servo_deg(pwm, 0)
    time.sleep(1)
    for desired_angle in range(0, 181, STEP_DEG):
        # Set the servo to desired angle
        set_servo_deg(pwm, desired_angle)
        
        # Wait for servo to reach position
        time.sleep_ms(200)
        
        # Read the actual angle (averaged)
        if servo_name == "shoulder":
            actual_angle, _ = read_actual_angles()
        else:  # elbow
            _, actual_angle = read_actual_angles()
        
        # Calculate error (actual - desired)
        error = actual_angle - desired_angle
        
        # Create list containing all pertinent data
        results.append((desired_angle, actual_angle, error))
        print(desired_angle, actual_angle, error)
    
    return results

def save_calibration_data(jig_id, shoulder_data, elbow_data):
    """
    Save calibration data to a text file
    """
    filename = f"calibration_data_{jig_id}.txt"
    
    with open(filename, 'w') as f:
        f.write(f"# Calibration data for test jig_{jig_id}\n")
        f.write(f"# Format: desired_angle, actual_angle, error\n")
        
        f.write("[SHOULDER]\n")
        for desired, actual, error in shoulder_data:
            f.write(f"{desired},{actual:.2f},{error:.2f}\n")
        
        f.write("[ELBOW]\n")
        for desired, actual, error in elbow_data:
            f.write(f"{desired},{actual:.2f},{error:.2f}\n")
    print("Saved as", filename)

def run_calibration():
    """
    Main calibration routine. This routine takes an input to get the jig_id, calibrates the min and max volts for each servo,
    calibrate the shoulder, calibrates the elbow then saves the data into a file.
    """
    # Get the jig ID from user
    jig_id = input("Enter your test jig ID (e.g., 1, 2, A, B): ").strip()
    if not jig_id:
        jig_id = "default"
    
    print(f"\nStarting calibration for test jig: {jig_id}")
    
    # First, update voltage constraints
    print("\nStep 1: Determining voltage ranges...")
    global shoulder_min_volts, shoulder_max_volts, elbow_min_volts, elbow_max_volts
    
    # Voltage at 0 degrees
    set_servo_deg(pwm_shoulder, 0)
    set_servo_deg(pwm_elbow, 0)
    time.sleep(1)
    shoulder_min_volts, elbow_min_volts = read_feedback_volts()
    print(f"At 0°: Shoulder={shoulder_min_volts:.3f}V, Elbow={elbow_min_volts:.3f}V")
   
    # Voltage at 180 degrees
    set_servo_deg(pwm_shoulder, 180)
    time.sleep(1.5)
    set_servo_deg(pwm_elbow, 180)
    time.sleep(1)
    shoulder_max_volts, elbow_max_volts = read_feedback_volts()
    print(f"At 180°: Shoulder={shoulder_max_volts:.3f}V, Elbow={elbow_max_volts:.3f}V")
    
    
    # Second, calibrate the shoulder
    print("\nStep 2: Calibrating shoulder servo...")
    # Return elbow to neutral position during shoulder calibration
    set_servo_deg(pwm_elbow, 90)
    time.sleep(1.5)
    shoulder_data = calibrate_servo("shoulder", pwm_shoulder)
    
    # Third, calibrate the elbow
    print("\nStep 3: Calibrating elbow servo...")
    # Return shoulder to neutral position during elbow calibration
    set_servo_deg(pwm_shoulder, 90)
    time.sleep(1)
    elbow_data = calibrate_servo("elbow", pwm_elbow)
    
    # Save the calibration data
    save_calibration_data(jig_id, shoulder_data, elbow_data)
    
    # Return servos to neutral position
    set_servo_deg(pwm_shoulder, 90)
    set_servo_deg(pwm_elbow, 90)
    
    print("\nCalibration complete!")
    
    """
    The following code block was used to manually test the voltage to angle conversion
    
    x1 = voltage_to_angle(2.7840002, shoulder_min_volts, shoulder_max_volts)
    y1 = voltage_to_angle(2.456, elbow_min_volts, elbow_max_volts)
    x2 = voltage_to_angle(1.4200001, shoulder_min_volts, shoulder_max_volts)
    y2 = voltage_to_angle(0.92800004, elbow_min_volts, elbow_max_volts)
    x3 = voltage_to_angle(0.614, shoulder_min_volts, shoulder_max_volts)
    y3 = voltage_to_angle(0.94000008, elbow_min_volts, elbow_max_volts)
    x4 = voltage_to_angle(0.744, shoulder_min_volts, shoulder_max_volts)
    y4 = voltage_to_angle(2.4780002, elbow_min_volts, elbow_max_volts)
    x5 = voltage_to_angle(1.746, shoulder_min_volts, shoulder_max_volts)
    y5 = voltage_to_angle(2.456, elbow_min_volts, elbow_max_volts)
    print(x1, y1)
    set_servo_deg(pwm_shoulder, x1)
    set_servo_deg(pwm_elbow, y1)
    time.sleep(1.5)
    set_servo_deg(pwm_shoulder, x2)
    set_servo_deg(pwm_elbow, y2)
    print(x2, y2)
    time.sleep(1.5)
    set_servo_deg(pwm_shoulder, x3)
    set_servo_deg(pwm_elbow, y3)
    print(x3, y3)
    time.sleep(1.5)
    set_servo_deg(pwm_shoulder, x4)
    set_servo_deg(pwm_elbow, y4)
    print(x4, y4)
    time.sleep(1.5)
    set_servo_deg(pwm_shoulder, x5)
    set_servo_deg(pwm_elbow, y5)
    print(x5, y5)
    time.sleep(1.5)
    """


if __name__ == "__main__":
    try:
        
        run_calibration()
        
        # Lines used to manually test voltage to angle conversion
        # x1, y1 = read_feedback_volts()
        # print(x1, y1)


    except:
        print("\nThe porgram has been forcefully stopped")

# Voltage of each corner (x is shoulder servo, y is elbow servo)
# x1 = 2.7840002 y1 = 2.456       bottom left
# x2 = 1.4200001 y2 = 0.92800004  bottom right
# x3 = 0.614 y3 = 0.94000008      top right
# x4 = 0.744 y4 = 2.4780002       top left
# x5 = 1.746 y5 = 2.456           middle

# Angles of each corner and the middle
# s14.432075  e36.929884    bottom left
# s105.567928 e138.42067   bottom right
# s159.42093  e137.62361    top right
# s150.73497  e35.468628    top left
# s83.786192  e36.929884    mid