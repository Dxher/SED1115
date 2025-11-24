from machine import Pin, PWM
import math, time

# Constants
ServoFreq = 50  # Hz

# Robot arm dimensions (in mm)
Lab = 155  # shoulder to elbow
Lbc = 155  # elbow to pen

# Shoulder base position (in mm)
ShoulderX = -50
ShoulderY = 139.5

# Constants that calbrate geometric angles to servo angles
SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

# Servos
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

def translate(angle):
    """
    Converts an angle in degrees to the corresponding input
    for the duty_u16 method of the servo class
    """
    MIN = 1638  # 0 degrees
    MAX = 8192  # 180 degrees
    DEG = (MAX - MIN) / 180  # value per degree

    # clamp angle to be between 0 and 180
    angle = max(0, min(180, angle))

    return int(angle * DEG + MIN)

def set_servo_deg(pwm, angle_deg):
    """Give the servo (pwm) an angle (angle_deg) to go to"""
    pwm.duty_u16(translate(angle_deg))

def inverse_kinematics(Cx, Cy):
    """
    Given a target point C (Cx, Cy) in mm, compute 
    shoulder_servo_deg and elbow_servo_deg
    """

    # 1) AC length and reachability check
    dx = Cx - ShoulderX
    dy = Cy - ShoulderY
    Lac = math.sqrt(dx*dx + dy*dy)

    max_reach = Lab + Lbc
    if Lac == 0 or Lac > max_reach:
        raise ValueError("Point C is unreachable for this arm")

    # 2) Geometric elbow angle (theta2), elbow-up solution ---
    # Law of cosines for elbow angle
    cos_t2 = (dx*dx + dy*dy - Lab*Lab - Lbc*Lbc) / (2 * Lab * Lbc)
    # Clamp for numerical safety
    cos_t2 = min(1, max(-1,cos_t2))

    # Elbow-up → positive sin
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2*cos_t2))
    theta2 = math.atan2(sin_t2, cos_t2)  # radians

    # --- 3) Geometric shoulder angle (theta1) ---
    k1 = Lab + Lbc * cos_t2
    k2 = Lbc * sin_t2
    theta1 = math.atan2(dy, dx) - math.atan2(k2, k1)  # radians

    # Convert to degrees
    shoulder_geom_deg = math.degrees(theta1)
    elbow_geom_deg    = math.degrees(theta2)

    # --- 4) Map geometric angles → servo angles using calibration ---
    shoulder_servo = SHOULDER_A * shoulder_geom_deg + SHOULDER_B
    elbow_servo    = ELBOW_A    * elbow_geom_deg    + ELBOW_B

    # Clamp to 0–180 for safety
    servoA = max(0, min(180, shoulder_servo))
    servoB = max(0, min(180, elbow_servo))

    return servoA, servoB

def calibration_setup(jig_id):
    """
    Read calibration file and return parsed error table
    
    Returns:
        Dictionary with 'shoulder' and 'elbow' keys, each containing
        a list of desired_angle and error
    """
    filename = f"calibration_data_{jig_id}.txt"
    
    calibration_table = {
        'shoulder': [],
        'elbow': []
    }
    
    current_servo = None
    
    try:
        with open(filename, 'r') as calibrated_file:
            for line in calibrated_file:
                line = line.strip()
                
                # Skip empty lines and comments
                if not line or line.startswith('#'):
                    continue
                
                # Check for section headers
                if line == '[SHOULDER]':
                    current_servo = 'shoulder'
                    continue
                elif line == '[ELBOW]':
                    current_servo = 'elbow'
                    continue
                
                # Parse data lines
                if current_servo and ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 3:
                        desired_angle = float(parts[0])
                        # actual_angle = float(parts[1]) Not needed
                        error = float(parts[2])
                        calibration_table[current_servo].append((desired_angle, error))
        
        # Ensure sort by first element (desired_angle)
        calibration_table['shoulder'].sort(key=lambda x: x[0])
        calibration_table['elbow'].sort(key=lambda x: x[0])
        
    except OSError:
        print(f"Warning: Could not open calibration file {filename}")
        print("Using default calibration (no error compensation)")
        # Create default table with zero errors
        for angle in range(0, 181, 10):
            calibration_table['shoulder'].append((angle, 0.0))
            calibration_table['elbow'].append((angle, 0.0))
    
    return calibration_table

def interpolate_error(angle, error_table):
    """
    Interpolate the error for a given angle using the error table.
    This uses linear interpolation between the two nearest calibration points.
    The function was created by ChatGPT but was unused since we we couldn't get
    passed inverse_kinematics issues.
    """
    # Handle edge cases
    if not error_table:
        return 0.0
    
    # If angle is below the first calibration pointSS
    if angle <= error_table[0][0]:
        return error_table[0][1]
    
    # If angle is above the last calibration point
    if angle >= error_table[-1][0]:
        return error_table[-1][1]
    
    # Find the two points to interpolate between
    for i in range(len(error_table) - 1):
        angle_low, error_low = error_table[i]
        angle_high, error_high = error_table[i + 1]
        
        if angle_low <= angle <= angle_high:
            # Linear interpolation
            if angle_high == angle_low:
                return error_low
            
            # Calculate interpolation factor
            t = (angle - angle_low) / (angle_high - angle_low)
            
            # Interpolate error
            interpolated_error = error_low + t * (error_high - error_low)
            return interpolated_error
    
    # Fallback (should not reach here)
    return 0.0

def send_angle(shoulder_angle, elbow_angle, calibration_table=None):
    """
    Send angles to servos, if provided a calibration table, compensate for errors
    """
    if calibration_table:
        # Get interpolated errors for each servo
        shoulder_error = interpolate_error(shoulder_angle, calibration_table['shoulder'])
        elbow_error = interpolate_error(elbow_angle, calibration_table['elbow'])
        
        # Compensate for the error by subtracting it from the desired angle
        compensated_shoulder = shoulder_angle - shoulder_error
        compensated_elbow = elbow_angle - elbow_error
        
        # Clamp to valid servo range
        compensated_shoulder = max(0, min(180, compensated_shoulder))
        compensated_elbow = max(0, min(180, compensated_elbow))
        
        # Send to servos
        set_servo_deg(pwm_shoulder, compensated_shoulder)
        set_servo_deg(pwm_elbow, compensated_elbow)
    else:
        # Clamp to valid servo range
        shoulder_angle = max(0, min(180, shoulder_angle))
        elbow_angle = max(0, min(180, elbow_angle))
        
        # Send to servos
        set_servo_deg(pwm_shoulder, shoulder_angle)
        set_servo_deg(pwm_elbow, elbow_angle)

def move_to(x, y, calibration_table=None):
    """
    Move the pen to position (x, y) in mm
    """
    servoA, servoB = inverse_kinematics(x, y) # Calculate servo angles
    
    send_angle(calibration_table, servoA, servoB) # Send angles to servos
    
    time.sleep_ms(50)

def draw_square(calibration_table=None):
    """
    Draw a square
    """    

    # Define corners
    corners = [
        (50, 50),  # bottom-left
        (150, 50),  # bottom-right
        (150, 150),  # top-right
        (50, 150),  # top-left
        (50, 50),  # back to start
    ]
    
    for x, y in corners:
        move_to(x, y, calibration_table)

def compare_calibration(jig_id):
    """
    Compare drawing results with and without calibration
    """
    # Load calibration data
    calibration_table = calibration_setup(jig_id)

    # Test 1: Square without calibration
    print("\n--- Drawing square WITHOUT calibration ---")
    draw_square(calibration_table=None)
    
    time.sleep(1)
    
    # Test 2: Square with calibration
    print("\n--- Drawing square WITH calibration ---")
    draw_square(calibration_table=calibration_table)

if __name__ == "__main__":    
    # Run comparison test for inputed jig_id
    compare_calibration(input("Enter your test jig ID: ").strip())