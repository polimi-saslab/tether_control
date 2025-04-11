import numpy as np
from scipy.spatial.transform import Rotation as R

def quaternion_multiply(q1, q2):
    """Hamilton product of two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return np.array([w, x, y, z])

def quaternion_rotate_vector_inverse(q, v):
    """
    Rotate vector v using the inverse of quaternion q.
    This is equivalent to applying q⁻¹ * v * q.
    """
    q = q / np.linalg.norm(q)
    q_inv = quaternion_conjugate(q)
    v_quat = np.array([0.0] + list(v))

    rotated = quaternion_multiply(quaternion_multiply(q_inv, v_quat), q)
    return rotated[1:]

def quaternion_conjugate(q):
    """Conjugate of a quaternion."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z])

def quaternion_rotate_vector(q, v):
    """Rotate a vector v using quaternion q."""
    q = q / np.linalg.norm(q)  # Normalize quaternion
    v_quat = np.array([0.0] + list(v))  # Make vector a pure quaternion
    q_conj = quaternion_conjugate(q)

    rotated = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)
    return rotated[1:]  # Return vector part only

def quaternion_to_rpy(q):
    """
    Convert quaternion [w, x, y, z] to RPY angles in degrees.
    Returns roll, pitch, yaw
    """
    # Convert to [x, y, z, w] for scipy
    r = R.from_quat([q[1], q[2], q[3], q[0]])
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    return roll, pitch, yaw


if __name__ == "__main__":

    q_ned = np.array([0.576528, -0.025392, -0.160942, 0.800668])
    q_enu = np.array([-0.096795, -0.159536, -0.973489, -0.132281])
    force_before = np.array([-3.993007, -3.829808, -5.758568])
    #                       [-2.39042896 -7.60148984 -3.32355733]
    rpy = quaternion_to_rpy(q_ned)
    print("BEFORE NED (degrees): Roll={:.2f}, Pitch={:.2f}, Yaw={:.2f}".format(*rpy))

    rpy_after = quaternion_to_rpy(q_enu)
    print("AFTER ENU (degrees): Roll={:.2f}, Pitch={:.2f}, Yaw={:.2f}".format(*rpy_after))

    print("Force before:", force_before)
    force_sent = quaternion_rotate_vector(q_enu, force_before)
    force_sent_inv = quaternion_rotate_vector_inverse(q_enu, force_before)
    print("Rotated Vector:", force_sent)
    print("Rotated Vector Inv:", force_sent_inv)
