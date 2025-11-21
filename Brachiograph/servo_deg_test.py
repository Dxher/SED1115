import math

# --- Geometry constants ---
ShoulderX, ShoulderY = -50.0, 139.5       # Shoulder position (mm)
Lab = 155.0                 # Shoulder-to-elbow (AB)
Lbc = 155.0                 # Elbow-to-tip (BC)

# --- Calibration constants (from the 4 premeasured angles) ---\

# Constants that calbrate geometric angles to servo angles
SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11


def inverse_kinematics_servos(Cx, Cy):
    """
    Given a target point C (Cx, Cy) in mm, compute:

        - shoulder_servo_deg
        - elbow_servo_deg
        - AC_length (distance from A to C)

    Uses 2-link IK with link lengths Lab = Lbc = 155, 
    shoulder at A = (-50, 139.5), then maps to servo
    angles using your measured calibration.
    """

    # --- 1) Distance AC and reachability check ---
    dx = Cx - ShoulderX
    dy = Cy - ShoulderY
    Lac = math.sqrt(dx*dx + dy*dy)

    max_reach = Lab + Lbc
    if Lac == 0 or Lac > max_reach:
        raise ValueError("Point C is unreachable for this arm")

    # --- 2) Geometric elbow angle (theta2), elbow-up solution ---
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
    shoulder_servo = max(0, min(180, shoulder_servo))
    elbow_servo    = max(0, min(180, elbow_servo))

    return shoulder_servo, elbow_servo

print(inverse_kinematics_servos(0, 0), "Expected 14, 36")

print(inverse_kinematics_servos(215,0), "Expected 105, 138")

print(inverse_kinematics_servos(215, 279), "Expected 160, 138")

print(inverse_kinematics_servos(0, 279), "Expected 150, 36")