from machine import Pin, PWM
import math, time

# Constants
ServoFreq = 50  # Hz

# Robot arm dimensions (in mm)
La = 155  # shoulder to elbow
Lb = 155  # elbow to pen

# Shoulder base position (in mm)
ShoulderX = -50
ShoulderY = 139.5

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
    Calculate the servo angles needed to position the pen at point C(Cx, Cy)
    All the equations were retrieved from SED1115 Lab8
    """
    AC = math.sqrt((ShoulderX - Cx)**2 + (ShoulderY - Cy)**2)
    AbaseC = math.sqrt((ShoulderX - Cx)**2 + Cy**2)
    Angle_BAC = math.acos((La**2 + AC**2 - Lb**2) / (2 * La * AC))
    Angle_ACB = math.asin((La * math.sin(Angle_BAC)) / Lb)
    Angle_YAC = math.acos((ShoulderY**2 + AC**2 - AbaseC**2) / (2 * ShoulderY * AC))
    alpha = math.degrees(Angle_YAC + Angle_BAC)
    beta = math.degrees(Angle_ACB + Angle_BAC)
    return alpha, beta

def forward_kinematics(shoulder, elbow):
    _shoulder = 180 - shoulder
    # x1 = ShoulderX + La * math.cos(math.radians(_shoulder))
    # y1 = ShoulderY + La * math.sin(math.radians(_shoulder))
    x1 = La * math.cos(math.radians(_shoulder))
    y1 = La * math.sin(math.radians(_shoulder))
    x2 = x1 + Lb * math.cos(math.radians(_shoulder + elbow))
    y2 = y1 + Lb * math.sin(math.radians(_shoulder + elbow))
    return x1, y1, x2, y2

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
        
        # Ensure sort by desired angle
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
    Interpolate the error for a given angle using the error table
    """
    # Handle edge cases
    if not error_table:
        return 0.0
    
    # If angle is below the first calibration point
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

def send_compensated_angle(calibration_table, shoulder_angle, elbow_angle):
    """
    Send compensated angles to the servos using the calibration table
    """
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

def send_uncompensated_angle(shoulder_angle, elbow_angle):
    """
    Send angles to servos without any compensation (for comparison)
    
    Parameters:
        shoulder_angle: Desired shoulder angle in degrees
        elbow_angle: Desired elbow angle in degrees
    """
    # Clamp to valid servo range
    shoulder_angle = max(0, min(180, shoulder_angle))
    elbow_angle = max(0, min(180, elbow_angle))
    
    # Send to servos
    set_servo_deg(pwm_shoulder, shoulder_angle)
    set_servo_deg(pwm_elbow, elbow_angle)

def move_to(x, y, calibration_table=None):
    """
    Move the pen to position (x, y) in mm
    
    Parameters:
        x, y: Target coordinates in mm
    """
    servoA, servoB = inverse_kinematics(x, y)
    
    if calibration_table:
        send_compensated_angle(calibration_table, servoA, servoB)
    else:
        send_uncompensated_angle(servoA, servoB)
    
    time.sleep_ms(50)

def draw_circle(center_x, center_y, radius, num_points=36, calibration_table=None):
    """
    Draw a circle
    
    Parameters:
        center_x, center_y: Center of circle in mm
        radius: Radius of circle in mm
        num_points: Number of points to use for the circle
    """    
    for i in range(num_points + 1):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        move_to(x, y, calibration_table)

def draw_square(calibration_table=None):
    """
    Draw a square
    """ 
    
    # Define corners
    corners = [
        (20, 20),  # bottom-left
        (50, 20),  # bottom-right
        (50, 50),  # top-right
        (20, 50),  # top-left
        (20, 20),  # back to start
    ]
    
    for x, y in corners:
        move_to(x, y, calibration_table)
        time.sleep(1)

def compare_calibration(jig_id):
    """
    Compare drawing results with and without calibration
    
    Parameters:
        jig_id: The unique identifier for the test jig
    """
    # Load calibration data
    calibration_table = calibration_setup(jig_id)
    
    print(calibration_table)
    # Define test shape parameters
    # Position in the middle of the paper
    
    print("CALIBRATION COMPARISON TEST")
    
    # Test 1: Circle without calibration
    print("\n--- Drawing circle WITHOUT calibration ---")
    input("Type anything to start...")
    draw_circle(70, 210, 30, num_points=36, calibration_table=None)
        
    # Test 2: Circle with calibration
    print("\n--- Drawing circle WITH calibration ---")
    input("Type anything to start...")
    draw_circle(140, 210, 30, num_points=36, calibration_table=calibration_table)
        
    # Test 3: Square without calibration
    print("\n--- Drawing square WITHOUT calibration ---")
    input("Type anything to start...")
    draw_square()
    
    # Test 4: Square with calibration
    print("\n--- Drawing square WITH calibration ---")
    input("Type anything to start...")
    draw_square(calibration_table=calibration_table)
    
    print("Comparison test complete!")


if __name__ == "__main__":    
    # Run comparison test for inputed jig_id
    calibration_table = calibration_setup(1)

    angle_shoulder, angle_elbow = inverse_kinematics(148.6,81.5)
    print(angle_shoulder, angle_elbow)
    x, y, x2, y2 = forward_kinematics(angle_shoulder, angle_elbow)
    print(x,y,x2,y2)

    # move_to(1, 1, None)
    # time.sleep(2)
    # move_to(215, 1, None)
    # time.sleep(2)
    # move_to(215, 279, None)
    # time.sleep(2)
    # move_to(1, 279, None)
    #compare_calibration(input("Enter your test jig ID: ").strip())