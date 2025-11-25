from machine import Pin, PWM
import time

# Constants
ServoFreq = 50  # Hz

# Shoulder base position (in mm)
ShoulderX = -50
ShoulderY = 139.5

# Servos
pwm_shoulder = PWM(Pin(0))
pwm_shoulder.freq(ServoFreq)
pwm_elbow = PWM(Pin(1))
pwm_elbow.freq(ServoFreq)
pwm_pen = PWM(Pin(2))
pwm_pen.freq(ServoFreq)

# Button for pen toggle
pen_button = Pin(16, Pin.IN, Pin.PULL_DOWN)

# Pen state
pen_is_down = False
pen_moving = False
pen_move_start_time = 0
pen_up_angle = 0      # Angle when pen is lifted
pen_down_angle = 30    # Angle when pen is on paper

def pen_up():
    """
    Lifts the pen up
    """
    global pen_is_down, pen_moving, pen_move_start_time

    if pen_is_down or not pen_moving:
        set_servo_deg(pwm_pen, pen_up_angle)
        pen_is_down = False
        pen_moving = True
        pen_move_start_time = time.ticks_ms()
        print("Pen UP")
    return

def pen_down():
    """
    Lowers the pen down
    """
    global pen_is_down, pen_moving, pen_move_start_time

    if not pen_is_down or not pen_moving:
            set_servo_deg(pwm_pen, pen_down_angle)
            pen_is_down = True
            pen_moving = True
            pen_move_start_time = time.ticks_ms()
            print("Pen DOWN")
    return

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
    """
    Give the servo (pwm) an angle (angle_deg) to go to
    """
    pwm.duty_u16(translate(angle_deg))
    return

def send_angle(shoulder_angle, elbow_angle):    
    """
    Sends angles to shoulder and elbow servos
    """
    set_servo_deg(pwm_shoulder, shoulder_angle)
    set_servo_deg(pwm_elbow, elbow_angle)

def run_gcode(commands):
    """
    Runs a list of G-code commands
    """
    for cmd in commands:
        c = cmd["cmd"]
        
        # Pen up
        if c == "M5":
            pen_up()
            time.sleep(0.5)

        # Pen down
        elif c == "M3":
            pen_down()
            time.sleep(0.5)

        # Move command
        elif c == "G1":
            S = cmd["params"].get("S") # Shoulder angle
            E = cmd["params"].get("E") # Elbow angle
            send_angle(S, E) # Send angles to servos
            time.sleep(0.5)

        # End of program
        elif c == "M18":
            print("End of program (M18).")
            break

def read_gcode(filename):
    """
    Reads a G-code file and returns a list of commands
    """
    commands = []
    try:
        with open(filename, "r") as file:
            for line in file:
                line = line.strip() # Remove leading/trailing whitespace
                if not line: # Skip empty lines
                    continue

                parts = line.split() # Split line into parts
                cmd = parts[0] # First part is the command
                params = {} 

                for token in parts[1:]:# Process parameters
                    key = token[0] # Parameter key (first character)
                    value = float(token[1:]) # Make value a float
                    params[key] = value # Store parameter

                commands.append({"cmd": cmd, "params": params}) # Add command to list

        return commands
    except OSError:
        print("Error: Could not open file", filename)
        return []

def take_input(input):
    """
    Takes user input to select and run a G-code file
    """
    # Execute line.gcode
    if input =="1":
        print("Executing the line.gcode file.")
        gcode = read_gcode("line.gcode")
        run_gcode(gcode)

    # Execute square.gcode
    elif input =="2":
        print("Executing the square.gcode file.")
        gcode = read_gcode("square.gcode")
        run_gcode(gcode)
    
    # Execute circle.gcode
    elif input =="3":
        print("Executing the circle.gcode file.")
        gcode = read_gcode("circle.gcode")
        run_gcode(gcode)

    # Execute circle_and_square.gcode
    elif input =="4":
        print("Executing the circle_and_square.gcode file.")
        gcode = read_gcode("circle_and_square.gcode")
        run_gcode(gcode)
    
    # Invalid input
    else:
        print("Invalid input. Please enter '1' to start or '4' to exit.")

def main():
    # Initialize pen to up position
    set_servo_deg(pwm_pen, pen_up_angle)

    while True:
        start = input("Enter '1' to start the line drawing gcode, '2' for square, '3' for \n"
        "circle, 4 for circle_and_square, or '5' to exit:")

        if start == "5":
            print("Exiting program.")
            break
        else:
            take_input(start)


if __name__ == "__main__":    
    try:
        main()
    except KeyboardInterrupt:
        print("Program interrupted by user.")