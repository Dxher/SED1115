from machine import Pin, PWM, ADC
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
pwm_pen = PWM(Pin(2))
pwm_pen.freq(ServoFreq)

# Potentiometers for X and Y control
pot_x = ADC(Pin(26))  # X-axis potentiometer
pot_y = ADC(Pin(27))  # Y-axis potentiometer

# Button for pen toggle
pen_button = Pin(16, Pin.IN, Pin.PULL_DOWN)

# Pen state
pen_is_down = False
pen_moving = False
pen_move_start_time = 0
PEN_UP_ANGLE = 90      # Angle when pen is lifted
PEN_DOWN_ANGLE = 45    # Angle when pen is on paper
PEN_MOVE_DURATION = 300  # ms to complete pen movement

prev_state = 1

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

    # Clamp to paper max degrees
    servoA = max(13, min(162, shoulder_servo))
    servoB = max(1, min(140, elbow_servo))

    return servoA, servoB

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
    x = (raw_x / 65535) * 215
    y = (raw_y / 65535) * 279
    
    return x, y

def toggle_pen():
    global pen_is_down, pen_moving, pen_move_start_time
    """Toggle pen between up and down states"""
    if pen_is_down or pen_moving:
        set_servo_deg(pwm_pen, PEN_UP_ANGLE)
        pen_is_down = False
        pen_moving = True
        pen_move_start_time = time.ticks_ms()
        print("Pen UP")
    else:
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

def send_angle(shoulder_angle, elbow_angle):    
    # Send to servos
    set_servo_deg(pwm_shoulder, shoulder_angle)
    set_servo_deg(pwm_elbow, elbow_angle)

def move_to(x, y):
    """
    Move the pen to position (x, y) in mm
    """
    servoA, servoB = inverse_kinematics(x, y) # Calculate servo angles
    
    send_angle(servoA, servoB) # Send angles to servos
    
    time.sleep_ms(50)


def main():
    """
    Compare drawing results with and without calibration
    """
    while True:

        # current_state = pen_button.value()
    
        # if prev_state == 0 and current_state == 1:
        #     update_pen_state()
        #     toggle_pen()
        #     time.sleep_ms(200)
        
        # prev_state = current_state

        x,y = read_potentiometers()

        move_to(x, y)

        time.sleep_us(50)

if __name__ == "__main__":    
    # Run comparison test for inputed jig_id
    move_to(90,90)
    main()