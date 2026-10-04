import numpy as np
from numpy.linalg import norm


#----- Quaternion and Axis rotation -----#
def quat_from_axis_rot(angle, axis):
    degrees = angle
    axis_norm = axis / norm(axis)

    w = np.cos(degrees/2)
    x,y,z = [np.sin(degrees / 2)*i for i in axis_norm]

    return np.array([w,x,y,z])

def axis_rot_from_quat(quat):
    w,x,y,z = quat
    
    angle = 2*np.arccos(w)

    if angle == 0:
        return np.zeros(4)
    
    i = x/np.sin(angle/2)
    j = y/np.sin(angle/2)
    k = z/np.sin(angle/2)


    return angle, np.array([i, j, k])

#----- Quaternion and Rotation Matrix -----#
def quat_to_R(q):
    if len(q) != 4:
        print(f"Input quaternion should have 4 elements. Input was {q}")
        return np.identity(3)
    
    # 0 Quaternion   
    if norm(q) == 0:

        return np.zeros((3,3))
    
    q_norm = unit(q)

    # if abs(q_norm[0] - 1) < 1e-10:
    #     print(f"Identity quaternion with Q={q_norm}")

    w, i, j, k = q_norm

    R00 = 1 - 2*(j*j + k*k)
    R01 = 2 * (i*j - k*w)
    R02 = 2 * (i*k + j*w)

    R10 = 2 * (i*j + k*w)
    R11 = 1 - 2 * (i*i + k*k)
    R12 = 2 * (j*k - i*w)

    R20 = 2 * (i*k - j*w)
    R21 = 2 * (j*k + i*w)
    R22 = 1 - 2*(i*i + j*j)

    R = np.array([[R00, R01, R02],
                  [R10, R11, R12],
                  [R20, R21, R22]])
    return R

def R_to_quat(R):

    # Makes sure the transpose can be taken
    if type(R) != np.ndarray:
        R = np.array(R)

    # Insomniac games formula, but taking transpose 
    # because they use scaler last convection
    row0, row1, row2 = R.T 
    m00, m01, m02 = row0
    m10, m11, m12 = row1
    m20, m21, m22 = row2

    if m22 < 0:
        if m00 > m11:
            t = 1 + m00 - m11 - m22
            q = np.array([t, m01+m10, m20+m02, m12-m21])    
        else:
            t = 1 - m00 + m11 - m22
            q = np.array([m01+m10, t, m12+m21, m20-m02])
    else:
        if m00 < -m11:
            t = 1 - m00 - m11 + m22
            q = np.array([m20+m02, m12+m21, t, m01-m10])
        else:
            t = 1 + m00 + m11 + m22
            q = np.array([m12-m21, m20-m02, m01-m10, t])

    q *= 1/2/np.sqrt(t)

    return q


#----- Quaternion math! -----#
def quat_mult(q1, q2):

    if len(q1) == 3:
        quat1 = np.array([0, *q1])
        # print("q1 is a vector")
    else:
        quat1 = q1

    if len(q2) == 3:
        quat2 = np.array([0, *q2])
        # print("q2 is a vector")
    else:
        quat2 = q2
    # print(quat1)
    # print(quat2)
    
    w1, x1, y1, z1 = quat1
    w2, x2, y2, z2 = quat2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

#----- Quaternion Operations -----#
def quat_inv(q):
    q = -q
    q[0] *= -1
    return q

def unit(q):
    if abs(norm(q)) < 0.000001:
        return np.zeros(len(q))
    
    return q / norm(q)

def slerp(q1, q2, t):
    """
    Spherical linear interpolation between two quaternions.
    
    Args:
        q1: First quaternion [w, x, y, z]
        q2: Second quaternion [w, x, y, z]
        t: Interpolation weight (0.0 = q1, 1.0 = q2)
    
    Returns:
        Interpolated quaternion [w, x, y, z]
    """
    # Normalize input quaternions
    q1 = unit(np.array(q1))
    q2 = unit(np.array(q2))
    
    # Compute dot product
    dot = np.dot(q1, q2)
    
    # If dot product is negative, negate q2 to take shorter path
    if dot < 0.0:
        q2 = -q2
        dot = -dot
    
    # Clamp dot product to avoid numerical issues with arccos
    dot = np.clip(dot, -1.0, 1.0)
    
    # If quaternions are very close, use linear interpolation
    if dot > 0.9995:
        result = q1 + t * (q2 - q1)
        return unit(result)
    
    # Calculate angle between quaternions
    theta = np.arccos(dot)
    sin_theta = np.sin(theta)
    
    # Compute interpolation weights
    w1 = np.sin((1.0 - t) * theta) / sin_theta
    w2 = np.sin(t * theta) / sin_theta
    
    # Return interpolated quaternion
    return w1 * q1 + w2 * q2

#----- Euler Angles to Quaternion Conversions -----#
def euler_ZYX_to_quat(roll, pitch, yaw):
    """
    Convert ZYX Euler angles to quaternion.
    Rotation order: Yaw (Z) -> Pitch (Y) -> Roll (X)
    
    Args:
        roll: Rotation about x-axis (radians)
        pitch: Rotation about y-axis (radians)
        yaw: Rotation about z-axis (radians)
    
    Returns:
        Quaternion [w, x, y, z]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return np.array([w, x, y, z])

def euler_ZXY_to_quat(roll, pitch, yaw):
    """
    Convert ZXY Euler angles to quaternion.
    Rotation order: Yaw (Z) -> Roll (X) -> Pitch (Y)
    
    Args:
        roll: Rotation about x-axis (radians)
        pitch: Rotation about y-axis (radians)
        yaw: Rotation about z-axis (radians)
    
    Returns:
        Quaternion [w, x, y, z]
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy - sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy + sr * sp * cy
    
    return np.array([w, x, y, z])

#----- Quaternion to Euler Angles Conversions -----#
def quat_to_euler_ZYX(q):
    """
    Convert quaternion to ZYX Euler angles.
    Rotation order: Yaw (Z) -> Pitch (Y) -> Roll (X)
    
    Args:
        q: Quaternion [w, x, y, z]
    
    Returns:
        (roll, pitch, yaw) in radians
    """
    q = unit(q)
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

def quat_to_euler_ZXY(q):
    """
    Convert quaternion to ZXY Euler angles.
    Rotation order: Yaw (Z) -> Roll (X) -> Pitch (Y)
    
    Args:
        q: Quaternion [w, x, y, z]
    
    Returns:
        (roll, pitch, yaw) in radians
    """
    q = unit(q)
    w, x, y, z = q
    
    # Roll (x-axis rotation)
    sinr = 2 * (w * x + y * z)
    if abs(sinr) >= 1:
        roll = np.copysign(np.pi / 2, sinr)  # Use 90 degrees if out of range
    else:
        roll = np.arcsin(sinr)
    
    # Pitch (y-axis rotation)
    siny_cosr = 2 * (w * y - z * x)
    cosy_cosr = 1 - 2 * (x * x + y * y)
    pitch = np.arctan2(siny_cosr, cosy_cosr)
    
    # Yaw (z-axis rotation)
    sinz_cosr = 2 * (w * z - x * y)
    cosz_cosr = 1 - 2 * (x * x + z * z)
    yaw = np.arctan2(sinz_cosr, cosz_cosr)
    
    return roll, pitch, yaw

#----- Applies Quaternion to Vector -----#
def quat_apply(quat, vector):
    quat = np.array(quat)
    temp = quat_mult(quat, vector)
    rslt = quat_mult(temp, quat_inv(quat))

    if abs(rslt[0]) > 0.0001:
        print(f"Quanternion is not normalized. Result vector of {rslt}") 
    
    # Discards 
    return rslt[1:4]
