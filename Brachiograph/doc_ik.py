import math

ShoulderX = -50
ShoulderY = 139.5
La = 155
Lb = 155


def inverse_kinematics(Cx, Cy):
    dx = ShoulderX - Cx
    dy = ShoulderY - Cy
    AC = math.sqrt(dx**2 + dy**2)
    #print("AC length:", AC)
    delta = math.asin(abs(dy)/AC)
    delta = math.degrees(delta)
    angle_BAC = math.degrees(math.acos((Lb**2 + AC**2 - La**2) / (2 * Lb * AC)))
    angle_ABC = math.degrees(math.acos((La**2 + Lb**2 - AC**2) / (2 * La * Lb)))
    if Cy > ShoulderY:
        delta = -delta
    #print("Angle BAC:", angle_BAC)
    #print("Angle ABC:", angle_ABC)
    #print("Delta:", delta)
    beta = 180 - angle_ABC
    alpha = 90 - angle_BAC - delta
    return alpha, beta

#alpha,beta = inverse_kinematics(148.6,81.5)
#print("148.6, 81.5", alpha,beta)
alpha,beta = inverse_kinematics(0,0)
print("Bottom left", alpha,beta)
alpha,beta = inverse_kinematics(215, 0)
print("Bottom right", alpha,beta)
alpha,beta = inverse_kinematics(215,279)
print("Top right", alpha,beta)
alpha,beta = inverse_kinematics(0, 279)
print("Top left", alpha,beta)