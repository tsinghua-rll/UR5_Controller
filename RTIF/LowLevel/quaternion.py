from __future__ import division
import numpy as np
import math


# all quaternion should be in form of (w, i, j, k)
def qmul(q0, q1):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([-x0 * x1 - y0 * y1 - z0 * z1 + w0 * w1,
                     x0 * w1 + y0 * z1 - z0 * y1 + w0 * x1,
                     -x0 * z1 + y0 * w1 + z0 * x1 + w0 * y1,
                     x0 * y1 - y0 * x1 + z0 * w1 + w0 * z1], dtype=np.float32)


def qconj(q0):
    w0, x0, y0, z0 = q0
    return np.array([w0, -x0, -y0, -z0], dtype=np.float32)


def qrotote_v(q0, v1, trans=(0, 0, 0)):
    """
    rotate the vector, if want rotate the coordinate, use qrotate_v(qconj(q0), v1)
    """
    q1 = np.zeros_like(q0)
    q1[1:] = v1
    return qmul(qmul(q0, q1), qconj(q0))[1:] + np.asarray(trans, dtype=np.float32)


def to_angle_axis(q0):
    rad = math.atan2(np.linalg.norm(q0[1:]), q0[0])
    sin_x = math.sin(rad)
    if abs(sin_x) < 1e-5:
        return np.array([sin_x * 2.0, q0[1], q0[2], q0[3]], dtype=np.float32)
    return np.array([rad * 2.0, q0[1] / sin_x, q0[2] / sin_x, q0[3] / sin_x], dtype=np.float32)


def from_matrix_to_q(mat):
    qw = math.sqrt(max(0, 1 + mat[0][0] + mat[1][1] + mat[2][2])) / 2.0
    qi = math.copysign(math.sqrt(max(0, 1 + mat[0][0] - mat[1][1] - mat[2][2])) / 2.0, mat[2][1] - mat[1][2])
    qj = math.copysign(math.sqrt(max(0, 1 - mat[0][0] + mat[1][1] - mat[2][2])) / 2.0, mat[0][2] - mat[2][0])
    qk = math.copysign(math.sqrt(max(0, 1 - mat[0][0] - mat[1][1] + mat[2][2])) / 2.0, mat[1][0] - mat[0][1])
    return np.asarray((qw, qi, qj, qk), dtype=np.float32)


def from_angle_axis(a):
    return np.asarray((math.cos(0.5 * a[0]),
                       math.sin(0.5 * a[0]) * a[1],
                       math.sin(0.5 * a[0]) * a[2],
                       math.sin(0.5 * a[0]) * a[3]), dtype=np.float32)


def from_euler(rpy, order="321"):
    qx = (math.cos(rpy[0] / 2.), math.sin(rpy[0] / 2.), 0., 0.)
    qy = (math.cos(rpy[1] / 2.), 0., math.sin(rpy[1] / 2.), 0.)
    qz = (math.cos(rpy[2] / 2.), 0., 0., math.sin(rpy[2] / 2.))
    l = (qx, qy, qz)
    order = (ord(order[0]) - ord('1'), ord(order[1]) - ord('1'), ord(order[2]) - ord('1'))
    return normalize(qmul(qmul(l[order[0]], l[order[1]]), l[order[2]]))


def to_euler(q):
    # rpy
    sinr = 2.0 * (q[0] * q[1] + q[2] * q[3])
    cosr = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q[0] * q[2] - q[3] * q[1])
    if math.fabs(sinp) >= 1.:
        pitch = math.copysign(np.pi / 2., sinp)
    else:
        pitch = math.asin(sinp)

    siny = 2.0 * (q[0] * q[3] + q[1] * q[2])
    cosy = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    yaw = math.atan2(siny, cosy)

    return np.asarray((roll, pitch, yaw), np.float32)


def from_vector_to_q(v1_c1, v1_c2, v2_c1, v2_c2, v3_c1=(0, 0, 0), v3_c2=(0, 0, 0)):
    """
    Notice: v1 and v2 should not be in the same direction !!!!
    :param v1_c1, v1_c2,: vector 1 in coordinate 1, 2
    :param v2_c1, v2_c2: vector 2 in coordinate 1, 2
    :param v3_c1, v3_c2: optional origin point in coordinate 1, 2
    :return: coordinate rotate quaternion from c1 to c2, translation from v1 to v2
            (qw, qi, qj, qk), (x, y, z)
    """
    v1_c1 = np.asarray(v1_c1, dtype=np.float32)
    v2_c1 = np.asarray(v2_c1, dtype=np.float32)
    v3_c1 = np.asarray(v3_c1, dtype=np.float32)
    v1_c2 = np.asarray(v1_c2, dtype=np.float32)
    v2_c2 = np.asarray(v2_c2, dtype=np.float32)
    v3_c2 = np.asarray(v3_c2, dtype=np.float32)
    c1 = np.asarray((v1_c1 - v3_c1, v2_c1 - v3_c1, np.cross(v1_c1 - v3_c1, v2_c1 - v3_c1)), dtype=np.float32).T
    c2 = np.asarray((v1_c2 - v3_c2, v2_c2 - v3_c2, np.cross(v1_c2 - v3_c2, v2_c2 - v3_c2)), dtype=np.float32).T
    mat = c2.dot(c1.T).dot(np.linalg.pinv(c1.dot(c1.T)))
    return from_matrix_to_q(mat), (v1_c2 - mat.dot(v1_c1) + v2_c2 - mat.dot(v2_c1) + v3_c2 - mat.dot(v3_c1)) / 3.0


def from_vector_array_to_q(v_c1, v_c2):
    """
    Calculate transform quaternion and translation vector from vector pairs in two coordinate
    :param v_c1: list or tuple of vector in source coordinate
    :param v_c2: list or tuple of vector in target coordinate
    :return: coordinate rotate quaternion from c1 to c2, translation from v1 to v2
            (qw, qi, qj, qk), (x, y, z)
    """
    if len(v_c1) != len(v_c2) or len(v_c1) <= 3:
        print ("Error! on enough vector pair or length of two array is different")
        return (1, 0, 0, 0), (0, 0, 0)

    v_c1 = np.asarray(v_c1, dtype=np.float32).T
    v_c2 = np.asarray(v_c2, dtype=np.float32).T
    mean_c1 = np.mean(v_c1, axis=1)
    mean_c2 = np.mean(v_c1, axis=1)
    v_c1 -= mean_c1
    v_c2 -= mean_c2

    mat = v_c2.dot(v_c1.T).dot(np.linalg.pinv(v_c1.dot(v_c1.T)))
    return from_matrix_to_q(mat), np.mean(v_c2 - mat.dot(v_c1), 1)


def normalize(q):
    q = np.asarray(q, dtype=np.float32) / np.linalg.norm(q)
    return q


if __name__ == "__main__":
    v1 = np.asarray((0, 0, 1, 1), dtype=np.float32).T
    v2 = np.asarray((0, 1, 0, 1), dtype=np.float32).T
    v3 = np.asarray((1, 0, 0, 1), dtype=np.float32).T
    T = np.asarray(((-1, 0, 0, 2), (0, 0, 1, 3), (0, 1, 0, 4)), dtype=np.float32)

    v1_n = T.dot(v1)
    v2_n = T.dot(v2)
    v3_n = T.dot(v3)

    q, t = from_vector_to_q(v1[:3], v1_n[:3], v2[:3], v2_n[:3], v3[:3], v3_n[:3])

    print (v1_n, qrotote_v(q, v1[:3], t), v2_n, qrotote_v(q, v2[:3], t), v3_n, qrotote_v(q, v3[:3], t))
