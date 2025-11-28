import math
import numpy as np
# Constants
Ax, Ay = -50.0, 139.5     # Shoulder position
L1 = 155.0                # AB
L2 = 155.0                # BC

# Constants that calbrate geometric angles to servo angles
SHOULDER_A = 1
SHOULDER_B = 146
ELBOW_A = -1.1
ELBOW_B = 171.11

def og_inverse_kinematics(Cx,Cy):
    
    # AC = sqrt((Ax - Cx)^2 + (Ay - Cy)^2)
    AC = math.sqrt((Ax - Cx)**2 + (Ay - Cy)**2)

    # AbaseC = sqrt((Ax - Cx)^2 + Cy^2)
    AbaseC = math.sqrt((Ax - Cx)**2 + Cy**2)

    # ∠BAC = cos^-1( (La^2 + AC^2 - Lb^2) / (2 * La * AC) )
    cos_BAC = (L1**2 + AC**2 - L2**2) / (2.0 * L1 * AC)
    angle_BAC = math.degrees(math.acos(cos_BAC))

    # ∠ACB = sin^-1( La * sin(∠BAC) / Lb )
    sin_ACB = L1 * math.sin(math.radians(angle_BAC)) / L2
    angle_ACB = math.degrees(math.asin(sin_ACB))

    # ∠YAC = cos^-1( (Ay^2 + AC^2 - AbaseC^2) / (2 * Ay * AC) )
    cos_YAC = (Ay**2 + AC**2 - AbaseC**2) / (2.0 * Ay * AC)
    angle_YAC = math.degrees(math.acos(cos_YAC))

    # α = ∠BAC + ∠YAC
    alpha = angle_BAC + angle_YAC

    # β = ∠BAC + ∠ACB
    beta = angle_BAC + angle_ACB

    return alpha, beta

def inverse_kinematics(Cx, Cy):
    """
    Given a target point C (Cx, Cy) in mm, compute 
    shoulder_servo_deg and elbow_servo_deg
    """

    # 1) AC length and reachability check
    dx = Cx - Ax
    dy = Cy - Ay
    Lac = math.sqrt(dx*dx + dy*dy)

    max_reach = L1 + L1
    if Lac == 0 or Lac > max_reach:
        raise ValueError("Point C is unreachable for this arm")

    # 2) Geometric elbow angle (theta2), elbow-up solution ---
    # Law of cosines for elbow angle
    cos_t2 = (dx*dx + dy*dy - L1*L1 - L2*L2) / (2 * L1 * L2)
    # Clamp for numerical safety
    cos_t2 = min(1, max(-1,cos_t2))

    # Elbow-up → positive sin
    sin_t2 = math.sqrt(max(0.0, 1.0 - cos_t2*cos_t2))
    theta2 = math.atan2(sin_t2, cos_t2)  # radians

    # --- 3) Geometric shoulder angle (theta1) ---
    k1 = L1 + L2 * cos_t2
    k2 = L2 * sin_t2
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


def gpt_inverse_kinematics(Cx, Cy):
    # Vector A -> C
    dx = Cx - Ax
    dy = Cy - Ay
    r = np.hypot(dx, dy)   # distance AC

    # Reachability check
    if r > (L1 + L2) or r < abs(L1 - L2):
        raise ValueError("Target point is unreachable with the given arm lengths.")

    # --- 1) Elbow angle (interior angle at B) ---
    # cos(∠ABC) = (L1² + L2² - r²) / (2 L1 L2)
    cos_gamma = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    cos_gamma = max(-1.0, min(1.0, cos_gamma))  # clamp for safety
    gamma = math.acos(cos_gamma)                # interior elbow angle (radians)

    # Servo beta is the external angle as in your drawing:
    beta_deg = 180.0 - math.degrees(gamma)

    # --- 2) Shoulder angle ---
    # Angle between AC and AB inside the triangle (at A)
    # cos(∠CAB) = (L1² + r² - L2²) / (2 L1 r)
    cos_delta = (L1**2 + r**2 - L2**2) / (2 * L1 * r)
    cos_delta = max(-1.0, min(1.0, cos_delta))
    delta = math.acos(cos_delta)                # radians

    # Angle of AC from +x axis
    theta_AC = math.atan2(dy, dx)

    # Shoulder angle from +x axis (choose the "elbow-down" configuration)
    theta_shoulder = theta_AC - delta

    # Convert to servo alpha: from vertical DOWN (−90°) CCW
    alpha_deg = math.degrees(theta_shoulder + math.pi / 2)

    return alpha_deg, beta_deg

def run_each_ik(Cx,Cy):
    alpha,beta = og_inverse_kinematics(Cx,Cy)
    servoA, servoB = inverse_kinematics(Cx,Cy)
    gpt_alpha, gpt_beta = gpt_inverse_kinematics(Cx,Cy)
    print("OG_inverse: alpha={:.2f}, beta={:.2f}".format(alpha,beta))
    print("IK_solver: servoA={:.2f}, servoB={:.2f}".format(servoA,servoB))
    print("GPT_inverse: gpt_alpha={:.2f}, GPTbeta={:.2f}".format(gpt_alpha,gpt_beta))

if __name__ == "__main__":
    while True:
        try:
            Cx = float(input("Enter target Cx (mm): "))
            Cy = float(input("Enter target Cy (mm): "))
            run_each_ik(Cx,Cy)
            con = input("Continue? (y/n): ")
            if con.lower() != 'y':
                break
        except Exception as e:
            print("Error:", e)