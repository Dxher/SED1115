import math

# Robot arm dimensions (in mm)
La = 155  # shoulder to elbow
Lb = 155  # elbow to pen

# Shoulder position (in mm)
ShoulderX = -50
ShoulderY = 139.5

# Helper to avoid domain errors from float rounding
def safe_acos(x):
    return math.acos(max(-1.0, min(1.0, x)))

def triangle_from_C(Cx, Cy):
    """
    Given point C (Cx, Cy), compute:
      - Lac length
      - angles at A, B, C
    for triangle ABC where:
      A = (-50, 139.5), AB = 155, BC = 155.
    """
    # 1) Compute Lac from coordinates
    dx = Cx - ShoulderX
    dy = Cy - ShoulderY
    Lac = math.sqrt(dx**2 + dy**2)  # distance between A and C

    # Check reachability: must be within 0 < Lac <= 310 (2*155)
    max_reLach = La + Lb
    if Lac == 0 or Lac > max_reLach:
        raise ValueError(f"Point C ({Cx}, {Cy}) is unreachable: Lac = {Lac:.3f} mm")

    # 2) Angles using Law of Cosines
    # Angle at A (between AB and AC), opposite side BC
    cos_A = (Lb**2 + Lac**2 - Lb**2) / (2 * Lb * Lac)
    angle_A = safe_acos(cos_A)
    
    # Angle at B (between AB and BC), opposite side Lac
    cos_B = (Lb**2 + Lb**2 - Lac**2) / (2 * Lb * Lb)
    angle_B = safe_acos(cos_B)

    # Angle at C (between BC and AC), opposite side AB
    cos_C = (Lb**2 + Lac**2 - Lb**2) / (2 * Lb * Lac)
    angle_C = safe_acos(cos_C)

    # Convert to degrees
    angle_A_deg = math.degrees(angle_A)
    angle_B_deg = math.degrees(angle_B)
    angle_C_deg = math.degrees(angle_C)

    return Lac, angle_A_deg, angle_B_deg, angle_C_deg

Lac, angle_A_deg, angle_B_deg, angle_C_deg = triangle_from_C(0,0)

print(Lac, angle_A_deg, angle_B_deg, angle_C_deg)