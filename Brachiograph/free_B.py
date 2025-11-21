import math
import time
from machine import Pin, PWM, I2C
from ads1x15 import ADS1015

# Constants for the brachiograph geometry
La = 155.0  # Length of first arm segment (shoulder to elbow) in mm
Lb = 155.0  # Length of second arm segment (elbow to pen) in mm
Ax = -50.0  # X coordinate of shoulder base in mm
Ay = 139.5  # Y coordinate of shoulder base in mm

# Drawing area constraints (paper boundaries)
PAPER_X_MIN = 0.0
PAPER_X_MAX = 214.0
PAPER_Y_MIN = 0.0
PAPER_Y_MAX = 278.0

# Pen servo positions
PEN_UP_ANGLE = 90    # Angle when pen is lifted off paper
PEN_DOWN_ANGLE = 120  # Angle when pen touches paper (adjust as needed)
PEN_SERVO_PIN = 2     # GPIO pin for pen servo

# Servo setup
ServoFreq = 50  # Hz
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)
pwm_pen = PWM(Pin(PEN_SERVO_PIN))
pwm_pen.freq(ServoFreq)

# ADC setup for potentiometers
i2c = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)
adc = ADS1015(i2c, address=0x48, gain=1)

# Button setup for pen control
button_pen = Pin(16, Pin.IN, Pin.PULL_UP)  # Button connected to GPIO 16

# Current position tracking
current_x = (PAPER_X_MIN + PAPER_X_MAX) / 2  # Start at center
current_y = (PAPER_Y_MIN + PAPER_Y_MAX) / 2
pen_is_down = False
last_button_state = 1  # Pull-up means 1 when not pressed
button_debounce_time = 0

# Pen servo feedback (if available on AIN3)
PEN_FEEDBACK_CHANNEL = 3  # Assuming pen servo feedback on AIN3
PEN_TOUCHDOWN_THRESHOLD = 2.5  # Voltage threshold indicating pen touched paper

# Calibration data (optional - set to None if not using)
calibration_data = None

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
        tuple: (servoA_angle, servoB_angle) in degrees
    """
    # Calculate distance from shoulder to pen
    AC = math.sqrt((Ax - Cx)**2 + (Ay - Cy)**2)
    
    # Check if the target is reachable
    if AC > (La + Lb) or AC < abs(La - Lb):
        return None, None
    
    # Calculate intermediate distances and angles
    AbaseC = math.sqrt((Ax - Cx)**2 + Cy**2)
    
    # Law of cosines to find angle at shoulder
    Angle_BAC = math.acos((La**2 + AC**2 - Lb**2) / (2 * La * AC))
    
    # Law of sines to find angle at pen
    Angle_ACB = math.asin((La * math.sin(Angle_BAC)) / Lb)
    
    # Angle from vertical to line AC
    Angle_YAC = math.acos((Ay**2 + AC**2 - AbaseC**2) / (2 * Ay * AC))
    
    # Calculate joint angles in radians
    alpha_rad = Angle_YAC + Angle_BAC
    beta_rad = Angle_ACB + Angle_BAC
    
    # Convert to degrees
    alpha = math.degrees(alpha_rad)
    beta = math.degrees(beta_rad)
    
    # Convert to servo angles based on mounting configuration
    servoA = alpha - 75
    servoB = 150 - beta
    
    # Ensure servo angles are within valid range
    servoA = max(0, min(180, servoA))
    servoB = max(0, min(180, servoB))
    
    return servoA, servoB

def read_potentiometers():
    """
    Read X and Y potentiometer values
    
    Returns:
        tuple: (x_value, y_value) normalized to 0.0-1.0 range
    """
    # Read from AIN2 (X pot) and AIN3 (Y pot)
    # Note: Adjust channels based on your wiring
    raw_x = adc.read(rate=4, channel1=2)  # X potentiometer on AIN2
    raw_y = adc.read(rate=4, channel1=3)  # Y potentiometer on AIN3
    
    # Convert to voltage
    v_x = adc.raw_to_v(raw_x)
    v_y = adc.raw_to_v(raw_y)
    
    # Normalize to 0.0-1.0 range (assuming 3.3V max)
    norm_x = min(1.0, max(0.0, v_x / 3.3))
    norm_y = min(1.0, max(0.0, v_y / 3.3))
    
    return norm_x, norm_y

def read_pen_feedback():
    """
    Read pen servo feedback to detect when pen touches paper
    
    Returns:
        float: Feedback voltage from pen servo
    """
    try:
        raw = adc.read(rate=4, channel1=PEN_FEEDBACK_CHANNEL)
        voltage = adc.raw_to_v(raw)
        return voltage
    except:
        # If no feedback available, return 0
        return 0.0

def move_pen_with_feedback(target_angle, timeout_ms=1000):
    """
    Move pen servo to target angle, stopping early if paper contact detected
    
    Args:
        target_angle: Target angle for pen servo
        timeout_ms: Maximum time to wait for movement
    """
    set_servo_deg(pwm_pen, target_angle)
    
    if target_angle == PEN_DOWN_ANGLE:
        # When lowering pen, monitor for paper contact
        start_time = time.ticks_ms()
        initial_voltage = read_pen_feedback()
        
        while time.ticks_diff(time.ticks_ms(), start_time) < timeout_ms:
            current_voltage = read_pen_feedback()
            
            # Check if voltage changed significantly (indicating resistance/contact)
            if abs(current_voltage - initial_voltage) > 0.5:
                # Pen likely touched paper, stop here
                print("Pen touchdown detected")
                break
            
            time.sleep_ms(10)

def set_pen_state(down):
    """
    Set pen up or down
    
    Args:
        down: True to lower pen, False to raise pen
    """
    global pen_is_down
    
    if down and not pen_is_down:
        print("Pen DOWN")
        move_pen_with_feedback(PEN_DOWN_ANGLE)
        pen_is_down = True
    elif not down and pen_is_down:
        print("Pen UP")
        set_servo_deg(pwm_pen, PEN_UP_ANGLE)
        pen_is_down = False
        time.sleep_ms(200)  # Give pen time to lift

def toggle_pen():
    """Toggle pen up/down state"""
    global pen_is_down
    set_pen_state(not pen_is_down)

def check_button():
    """
    Check button state with debouncing
    
    Returns:
        bool: True if button was just pressed (falling edge)
    """
    global last_button_state, button_debounce_time
    
    current_state = button_pen.value()
    current_time = time.ticks_ms()
    
    # Check if enough time has passed for debouncing (50ms)
    if time.ticks_diff(current_time, button_debounce_time) > 50:
        # Detect falling edge (button press with pull-up)
        if last_button_state == 1 and current_state == 0:
            last_button_state = current_state
            button_debounce_time = current_time
            return True
        last_button_state = current_state
    
    return False

def constrain_to_paper(x, y):
    """
    Constrain coordinates to paper boundaries
    
    Args:
        x, y: Desired coordinates
    
    Returns:
        tuple: (constrained_x, constrained_y)
    """
    x = max(PAPER_X_MIN, min(PAPER_X_MAX, x))
    y = max(PAPER_Y_MIN, min(PAPER_Y_MAX, y))
    return x, y

def map_potentiometer_to_position(pot_x, pot_y, current_x, current_y, sensitivity=5.0):
    """
    Map potentiometer values to position changes
    
    Args:
        pot_x, pot_y: Normalized potentiometer values (0.0-1.0)
        current_x, current_y: Current position
        sensitivity: Movement sensitivity in mm per update
    
    Returns:
        tuple: (new_x, new_y) position
    """
    # Center point for potentiometers (0.5 = no movement)
    center = 0.5
    deadzone = 0.05  # Small deadzone around center
    
    # Calculate movement deltas
    delta_x = 0
    delta_y = 0
    
    # X-axis movement
    if abs(pot_x - center) > deadzone:
        delta_x = (pot_x - center) * sensitivity * 2  # *2 because range is 0.5
    
    # Y-axis movement
    if abs(pot_y - center) > deadzone:
        delta_y = (pot_y - center) * sensitivity * 2
    
    # Apply deltas to current position
    new_x = current_x + delta_x
    new_y = current_y + delta_y
    
    # Constrain to paper boundaries
    new_x, new_y = constrain_to_paper(new_x, new_y)
    
    return new_x, new_y

def move_to_position(x, y):
    """
    Move arm to specified position
    
    Args:
        x, y: Target coordinates in mm
    
    Returns:
        bool: True if movement successful
    """
    servo_a, servo_b = inverse_kinematics(x, y)
    
    if servo_a is not None and servo_b is not None:
        set_servo_deg(pwm_shoulder, servo_a)
        set_servo_deg(pwm_elbow, servo_b)
        return True
    return False

def calibrate_potentiometers():
    """
    Simple calibration routine for potentiometers
    """
    print("\n=== POTENTIOMETER CALIBRATION ===")
    print("Move X pot to minimum (full left), then press button...")
    
    while not check_button():
        time.sleep_ms(10)
    
    x_min, _ = read_potentiometers()
    print(f"X min: {x_min:.3f}")
    
    print("Move X pot to maximum (full right), then press button...")
    while not check_button():
        time.sleep_ms(10)
    
    x_max, _ = read_potentiometers()
    print(f"X max: {x_max:.3f}")
    
    print("Move Y pot to minimum (full down), then press button...")
    while not check_button():
        time.sleep_ms(10)
    
    _, y_min = read_potentiometers()
    print(f"Y min: {y_min:.3f}")
    
    print("Move Y pot to maximum (full up), then press button...")
    while not check_button():
        time.sleep_ms(10)
    
    _, y_max = read_potentiometers()
    print(f"Y max: {y_max:.3f}")
    
    print("Calibration complete! Center both pots to begin drawing.")
    print("Press button to start...")
    
    while not check_button():
        time.sleep_ms(10)
    
    return (x_min, x_max), (y_min, y_max)

def draw_boundary():
    """Draw the paper boundary to show drawable area"""
    print("Drawing paper boundary...")
    
    # Lift pen
    set_pen_state(False)
    
    # Move to corner
    move_to_position(PAPER_X_MIN, PAPER_Y_MIN)
    time.sleep_ms(500)
    
    # Lower pen
    set_pen_state(True)
    
    # Draw rectangle
    corners = [
        (PAPER_X_MIN, PAPER_Y_MIN),
        (PAPER_X_MAX, PAPER_Y_MIN),
        (PAPER_X_MAX, PAPER_Y_MAX),
        (PAPER_X_MIN, PAPER_Y_MAX),
        (PAPER_X_MIN, PAPER_Y_MIN)
    ]
    
    for x, y in corners:
        move_to_position(x, y)
        time.sleep_ms(200)
    
    # Lift pen
    set_pen_state(False)
    print("Boundary complete!")

def free_draw_mode():
    """
    Main free drawing mode using potentiometers
    """
    global current_x, current_y
    
    print("\n" + "="*50)
    print("FREE DRAWING MODE")
    print("="*50)
    print("\nControls:")
    print("- X Potentiometer: Move left/right")
    print("- Y Potentiometer: Move up/down")
    print("- Button: Toggle pen up/down")
    print("- Press CTRL+C to exit")
    print("\nInitializing...")
    
    # Initial pen position (up)
    set_pen_state(False)
    
    # Optional: Calibrate potentiometers
    print("\nPress button to skip calibration, or wait 3 seconds to calibrate...")
    start_time = time.ticks_ms()
    skip_calibration = False
    
    while time.ticks_diff(time.ticks_ms(), start_time) < 3000:
        if check_button():
            skip_calibration = True
            break
        time.sleep_ms(10)
    
    pot_ranges = None
    if not skip_calibration:
        pot_ranges = calibrate_potentiometers()
    
    # Move to center position
    print(f"\nMoving to start position ({current_x:.1f}, {current_y:.1f})...")
    move_to_position(current_x, current_y)
    time.sleep(1)
    
    # Optional: Draw boundary
    print("\nPress button within 2 seconds to draw paper boundary...")
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < 2000:
        if check_button():
            draw_boundary()
            move_to_position(current_x, current_y)
            break
        time.sleep_ms(10)
    
    print("\n--- READY TO DRAW ---")
    print(f"Starting position: ({current_x:.1f}, {current_y:.1f})")
    print("Use potentiometers to move, button to toggle pen\n")
    
    # Main drawing loop
    update_counter = 0
    last_position_print = time.ticks_ms()
    
    try:
        while True:
            # Read potentiometers
            pot_x, pot_y = read_potentiometers()
            
            # Check button for pen toggle
            if check_button():
                toggle_pen()
            
            # Calculate new position based on potentiometer input
            new_x, new_y = map_potentiometer_to_position(
                pot_x, pot_y, current_x, current_y, 
                sensitivity=3.0  # Adjust sensitivity as needed
            )
            
            # Only move if position changed significantly
            if abs(new_x - current_x) > 0.5 or abs(new_y - current_y) > 0.5:
                if move_to_position(new_x, new_y):
                    current_x = new_x
                    current_y = new_y
                    
                    # Print position occasionally
                    if time.ticks_diff(time.ticks_ms(), last_position_print) > 1000:
                        status = "Drawing" if pen_is_down else "Moving"
                        print(f"{status} at ({current_x:.1f}, {current_y:.1f})")
                        last_position_print = time.ticks_ms()
            
            # Small delay for smooth operation
            time.sleep_ms(20)
            
            update_counter += 1
            
            # Periodic status update
            if update_counter % 100 == 0:
                # Check if pen servo is struggling (optional)
                if pen_is_down:
                    pen_feedback = read_pen_feedback()
                    if pen_feedback > PEN_TOUCHDOWN_THRESHOLD + 0.5:
                        print("Warning: Pen servo may be struggling")
    
    except KeyboardInterrupt:
        print("\n\nExiting free draw mode...")
        
        # Clean up - lift pen and move to safe position
        set_pen_state(False)
        print("Moving to home position...")
        move_to_position((PAPER_X_MIN + PAPER_X_MAX) / 2, PAPER_Y_MAX / 2)
        
        print("Free draw mode ended.")

def test_pen_servo():
    """Test pen servo operation"""
    print("\n=== PEN SERVO TEST ===")
    print("Press button to cycle pen up/down...")
    print("Press CTRL+C to exit test\n")
    
    try:
        while True:
            if check_button():
                toggle_pen()
                feedback = read_pen_feedback()
                print(f"Pen feedback voltage: {feedback:.3f}V")
            time.sleep_ms(10)
    except KeyboardInterrupt:
        print("\nPen test complete")
        set_pen_state(False)

# Main execution
if __name__ == "__main__":
    print("\n" + "="*50)
    print("BRACHIOGRAPH FREE DRAWING")
    print("="*50)
    
    print("\nSelect mode:")
    print("1. Free Drawing Mode")
    print("2. Test Pen Servo")
    print("3. Draw Test Pattern")
    
    # Simple menu (you might need to implement a different input method for MicroPython)
    print("\nStarting Free Drawing Mode in 3 seconds...")
    print("(Press button now for Pen Test mode)")
    
    start_time = time.ticks_ms()
    mode_selected = 1
    
    while time.ticks_diff(time.ticks_ms(), start_time) < 3000:
        if check_button():
            mode_selected = 2
            break
        time.sleep_ms(10)
    
    if mode_selected == 1:
        free_draw_mode()
    elif mode_selected == 2:
        test_pen_servo()