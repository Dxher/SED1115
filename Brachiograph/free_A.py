import math
import time
from machine import Pin, PWM, ADC

# Constants for the brachiograph geometry
La = 155.0  # Length of first arm segment (shoulder to elbow) in mm
Lb = 155.0  # Length of second arm segment (elbow to pen) in mm
Ax = -50.0  # X coordinate of shoulder base in mm
Ay = 139.5  # Y coordinate of shoulder base in mm

# Workspace boundaries (paper area)
PAPER_X_MIN = 0
PAPER_X_MAX = 214
PAPER_Y_MIN = 0
PAPER_Y_MAX = 278

# Servo setup
ServoFreq = 50  # Hz

# Shoulder servo
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)

# Elbow servo
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)

# Pen servo
pwm_pen = PWM(Pin(2))
pwm_pen.freq(ServoFreq)

# Potentiometers for X and Y control
pot_x = ADC(Pin(26))  # X-axis potentiometer
pot_y = ADC(Pin(27))  # Y-axis potentiometer

# Button for pen toggle
pen_button = Pin(16, Pin.IN, Pin.PULL_UP)

# Pen state
pen_is_down = False
pen_moving = False
pen_move_start_time = 0

# Pen servo angles
PEN_UP_ANGLE = 90      # Angle when pen is lifted
PEN_DOWN_ANGLE = 45    # Angle when pen is on paper
PEN_MOVE_DURATION = 300  # ms to complete pen movement

# Debounce settings
last_button_press = 0
DEBOUNCE_MS = 200

# Calibration data
calibration_data = None

# Current position
current_x = PAPER_X_MAX / 2
current_y = PAPER_Y_MAX / 2

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
    Calculate the servo angles needed to position the pen at (Cx, Cy)
    
    Args:
        Cx: Target X coordinate in mm
        Cy: Target Y coordinate in mm
    
    Returns:
        tuple: (servoA_angle, servoB_angle) in degrees, or (None, None) if unreachable
    """
    # Calculate distance from shoulder to pen
    AC = math.sqrt((Ax - Cx)**2 + (Ay - Cy)**2)
    
    # Check if the target is reachable
    if AC > (La + Lb) or AC < abs(La - Lb):
        return None, None
    
    # Calculate intermediate distances and angles
    AbaseC = math.sqrt((Ax - Cx)**2 + Cy**2)
    
    # Law of cosines to find angle at shoulder
    cos_angle_bac = (La**2 + AC**2 - Lb**2) / (2 * La * AC)
    cos_angle_bac = max(-1, min(1, cos_angle_bac))  # Clamp for numerical stability
    Angle_BAC = math.acos(cos_angle_bac)
    
    # Law of sines to find angle at pen
    sin_angle_acb = (La * math.sin(Angle_BAC)) / Lb
    sin_angle_acb = max(-1, min(1, sin_angle_acb))  # Clamp for numerical stability
    Angle_ACB = math.asin(sin_angle_acb)
    
    # Angle from vertical to line AC
    cos_angle_yac = (Ay**2 + AC**2 - AbaseC**2) / (2 * Ay * AC)
    cos_angle_yac = max(-1, min(1, cos_angle_yac))  # Clamp for numerical stability
    Angle_YAC = math.acos(cos_angle_yac)
    
    # Calculate joint angles in radians
    alpha_rad = Angle_YAC + Angle_BAC
    beta_rad = Angle_ACB + Angle_BAC
    
    # Convert to degrees
    alpha = math.degrees(alpha_rad)
    beta = math.degrees(beta_rad)
    
    # Convert to servo angles based on mounting configuration
    servoA = alpha - 75
    servoB = 150 - beta
    
    # Ensure servo angles are within valid range [0, 180]
    servoA = max(0, min(180, servoA))
    servoB = max(0, min(180, servoB))
    
    return servoA, servoB

def calibration_setup(jig_id):
    """
    Read calibration file and return parsed error table
    """
    filename = f"calibration_data_{jig_id}.txt"
    
    calibration = {
        'shoulder': {'angles': [], 'errors': []},
        'elbow': {'angles': [], 'errors': []}
    }
    
    try:
        with open(filename, 'r') as f:
            current_servo = None
            
            for line in f:
                line = line.strip()
                
                if line.startswith('#') or not line:
                    continue
                
                if line == '[SHOULDER]':
                    current_servo = 'shoulder'
                    continue
                elif line == '[ELBOW]':
                    current_servo = 'elbow'
                    continue
                
                if current_servo and ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 3:
                        desired_angle = float(parts[0])
                        error = float(parts[2])
                        
                        calibration[current_servo]['angles'].append(desired_angle)
                        calibration[current_servo]['errors'].append(error)
        
        print(f"Calibration loaded from {filename}")
        return calibration
    
    except Exception as e:
        print(f"No calibration file found: {e}")
        return None

def interpolate_error(angle, angles_list, errors_list):
    """
    Interpolate error value for a given angle using linear interpolation
    """
    if not angles_list or not errors_list:
        return 0.0
    
    if angle <= angles_list[0]:
        return errors_list[0]
    if angle >= angles_list[-1]:
        return errors_list[-1]
    
    for i in range(len(angles_list) - 1):
        if angles_list[i] <= angle <= angles_list[i + 1]:
            x0, x1 = angles_list[i], angles_list[i + 1]
            y0, y1 = errors_list[i], errors_list[i + 1]
            
            interpolated_error = y0 + (angle - x0) * (y1 - y0) / (x1 - x0)
            return interpolated_error
    
    return 0.0

def send_compensated_angle(shoulder_angle, elbow_angle):
    """
    Send compensated angles to servos using calibration data
    """
    global calibration_data
    
    if calibration_data:
        shoulder_error = interpolate_error(
            shoulder_angle,
            calibration_data['shoulder']['angles'],
            calibration_data['shoulder']['errors']
        )
        
        elbow_error = interpolate_error(
            elbow_angle,
            calibration_data['elbow']['angles'],
            calibration_data['elbow']['errors']
        )
        
        compensated_shoulder = shoulder_angle - shoulder_error
        compensated_elbow = elbow_angle - elbow_error
        
        compensated_shoulder = max(0, min(180, compensated_shoulder))
        compensated_elbow = max(0, min(180, compensated_elbow))
        
    else:
        compensated_shoulder = shoulder_angle
        compensated_elbow = elbow_angle
    
    set_servo_deg(pwm_shoulder, compensated_shoulder)
    set_servo_deg(pwm_elbow, compensated_elbow)

def move_to(x, y):
    """
    Move the pen to position (x, y) using inverse kinematics
    
    Returns:
        bool: True if position is reachable, False otherwise
    """
    servo_a, servo_b = inverse_kinematics(x, y)
    
    if servo_a is not None and servo_b is not None:
        send_compensated_angle(servo_a, servo_b)
        return True
    return False

def read_potentiometers():
    """
    Read potentiometer values and map to X, Y coordinates
    
    Returns:
        tuple: (x, y) coordinates in mm
    """
    # Read raw ADC values (0-65535 for 16-bit ADC)
    raw_x = pot_x.read_u16()
    raw_y = pot_y.read_u16()
    
    # Map to paper coordinates
    # Apply some smoothing by averaging with current position
    x = (raw_x / 65535) * (PAPER_X_MAX - PAPER_X_MIN) + PAPER_X_MIN
    y = (raw_y / 65535) * (PAPER_Y_MAX - PAPER_Y_MIN) + PAPER_Y_MIN
    
    return x, y

def pen_up():
    """Lift the pen off the paper"""
    global pen_is_down, pen_moving, pen_move_start_time
    
    if pen_is_down or pen_moving:
        set_servo_deg(pwm_pen, PEN_UP_ANGLE)
        pen_is_down = False
        pen_moving = True
        pen_move_start_time = time.ticks_ms()
        print("Pen UP")

def pen_down():
    """Lower the pen onto the paper"""
    global pen_is_down, pen_moving, pen_move_start_time
    
    if not pen_is_down:
        set_servo_deg(pwm_pen, PEN_DOWN_ANGLE)
        pen_is_down = True
        pen_moving = True
        pen_move_start_time = time.ticks_ms()
        print("Pen DOWN")

def update_pen_state():
    """
    Update pen movement state - check if pen has finished moving
    This prevents continuous servo strain once pen reaches target
    """
    global pen_moving
    
    if pen_moving:
        elapsed = time.ticks_diff(time.ticks_ms(), pen_move_start_time)
        if elapsed >= PEN_MOVE_DURATION:
            pen_moving = False
            # Optionally reduce servo power after movement completes
            # This helps prevent servo strain when pen is against paper

def toggle_pen():
    """Toggle pen between up and down states"""
    if pen_is_down:
        pen_up()
    else:
        pen_down()

def check_button():
    """
    Check if the pen button has been pressed (with debouncing)
    
    Returns:
        bool: True if button was just pressed, False otherwise
    """
    global last_button_press
    
    current_time = time.ticks_ms()
    
    # Button is active LOW (pressed = 0)
    if pen_button.value() == 0:
        # Check debounce
        if time.ticks_diff(current_time, last_button_press) > DEBOUNCE_MS:
            last_button_press = current_time
            return True
    
    return False

def apply_smoothing(new_val, old_val, factor=0.3):
    """
    Apply exponential smoothing to reduce jitter
    
    Args:
        new_val: New reading
        old_val: Previous value
        factor: Smoothing factor (0-1, lower = more smoothing)
    
    Returns:
        Smoothed value
    """
    return old_val + factor * (new_val - old_val)

def apply_deadzone(new_val, old_val, deadzone=2.0):
    """
    Apply deadzone to prevent small movements
    
    Args:
        new_val: New position
        old_val: Old position
        deadzone: Minimum change required to move (mm)
    
    Returns:
        Position (old if within deadzone, new otherwise)
    """
    if abs(new_val - old_val) < deadzone:
        return old_val
    return new_val

def free_draw_mode(jig_id=None):
    """
    Main free drawing mode using potentiometers
    
    Args:
        jig_id: Test jig ID for loading calibration (optional)
    """
    global calibration_data, current_x, current_y
    
    print("\n" + "="*50)
    print("FREE DRAW MODE")
    print("="*50)
    print("\nControls:")
    print("  - X Potentiometer: Move left/right")
    print("  - Y Potentiometer: Move up/down")
    print("  - Button: Toggle pen up/down")
    print("  - Press Ctrl+C to exit")
    print("="*50 + "\n")
    
    # Load calibration if jig_id provided
    if jig_id:
        calibration_data = calibration_setup(jig_id)
    
    # Initialize pen to UP position
    pen_up()
    time.sleep(0.5)
    
    # Initialize to center of paper
    current_x = PAPER_X_MAX / 2
    current_y = PAPER_Y_MAX / 2
    
    # Move to initial position
    if move_to(current_x, current_y):
        print(f"Starting position: ({current_x:.1f}, {current_y:.1f})")
    else:
        print("Warning: Starting position unreachable!")
    
    # Smoothed position values
    smooth_x = current_x
    smooth_y = current_y
    
    # Position display counter (to avoid flooding serial)
    display_counter = 0
    
    try:
        while True:
            # Check for pen button press
            if check_button():
                toggle_pen()
            
            # Update pen movement state
            update_pen_state()
            
            # Read potentiometers
            raw_x, raw_y = read_potentiometers()
            
            # Apply smoothing to reduce jitter
            smooth_x = apply_smoothing(raw_x, smooth_x, factor=0.2)
            smooth_y = apply_smoothing(raw_y, smooth_y, factor=0.2)
            
            # Apply deadzone to prevent unnecessary small movements
            new_x = apply_deadzone(smooth_x, current_x, deadzone=1.5)
            new_y = apply_deadzone(smooth_y, current_y, deadzone=1.5)
            
            # Check if position has changed enough to update
            if new_x != current_x or new_y != current_y:
                # Clamp to paper boundaries
                new_x = max(PAPER_X_MIN, min(PAPER_X_MAX, new_x))
                new_y = max(PAPER_Y_MIN, min(PAPER_Y_MAX, new_y))
                
                # Try to move to new position
                if move_to(new_x, new_y):
                    current_x = new_x
                    current_y = new_y
                    
                    # Display position periodically
                    display_counter += 1
                    if display_counter >= 10:
                        pen_status = "DOWN" if pen_is_down else "UP"
                        print(f"Pos: ({current_x:.1f}, {current_y:.1f}) - Pen: {pen_status}")
                        display_counter = 0
                else:
                    # Position unreachable - provide feedback
                    if display_counter >= 20:
                        print(f"Position ({new_x:.1f}, {new_y:.1f}) unreachable")
                        display_counter = 0
            
            # Small delay to prevent overwhelming the system
            time.sleep_ms(20)
    
    except KeyboardInterrupt:
        print("\n\nExiting free draw mode...")
        pen_up()
        time.sleep(0.5)
        print("Pen raised. Goodbye!")

def test_pen_servo():
    """
    Test the pen servo to find optimal up/down angles
    """
    print("\nPen Servo Test")
    print("="*30)
    
    print("Testing PEN UP position...")
    set_servo_deg(pwm_pen, PEN_UP_ANGLE)
    time.sleep(2)
    
    print("Testing PEN DOWN position...")
    set_servo_deg(pwm_pen, PEN_DOWN_ANGLE)
    time.sleep(2)
    
    print("Returning to UP position...")
    set_servo_deg(pwm_pen, PEN_UP_ANGLE)
    time.sleep(1)
    
    print("Pen servo test complete!")

def test_potentiometers():
    """
    Test potentiometer readings
    """
    print("\nPotentiometer Test")
    print("Press Ctrl+C to stop")
    print("="*30)
    
    try:
        while True:
            x, y = read_potentiometers()
            print(f"X: {x:.1f} mm, Y: {y:.1f} mm")
            time.sleep_ms(200)
    except KeyboardInterrupt:
        print("\nTest stopped")

# Main execution
if __name__ == "__main__":
    print("\n" + "="*50)
    print("BRACHIOGRAPH FREE DRAW")
    print("="*50)
    
    print("\nOptions:")
    print("  1. Start free draw mode")
    print("  2. Test pen servo")
    print("  3. Test potentiometers")
    
    choice = input("\nEnter choice (1-3): ").strip()
    
    if choice == "1":
        jig_id = input("Enter jig ID for calibration (or press Enter to skip): ").strip()
        if not jig_id:
            jig_id = None
        free_draw_mode(jig_id)
    
    elif choice == "2":
        test_pen_servo()
    
    elif choice == "3":
        test_potentiometers()
    
    else:
        print("Invalid choice. Starting free draw mode...")
        free_draw_mode()