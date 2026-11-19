"""3-vectors and quaternions as plain lists (fast enough, no numpy needed).

Quaternions are [x, y, z, w] (ROS order). q_rot(q, v) maps a body-frame
vector into the parent frame; q_rot_inv does the opposite.
"""
import math


# ------------------------------------------------------------------ vectors
def v_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def v_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def v_mul(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def v_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def v_cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def v_norm(a):
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def v_unit(a):
    """Unit vector, or zero vector if a is (almost) zero."""
    n = v_norm(a)
    return [0.0, 0.0, 0.0] if n <= 1e-9 else [a[0] / n, a[1] / n, a[2] / n]


def v_clamp(a, lim):
    n = v_norm(a)
    return v_mul(a, lim / n) if n > lim else list(a)


def dist(a, b):
    return v_norm(v_sub(a, b))


def m_vec(M, v):
    """3x3 matrix (row tuples) times vector."""
    return [M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
            M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
            M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]]


# -------------------------------------------------------------- quaternions
def q_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return [w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2]


def q_conj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def q_normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return [c / n for c in q] if n > 1e-9 else [0.0, 0.0, 0.0, 1.0]


def q_rot(q, v):
    """Rotate body vector v into the parent frame."""
    u = q[:3]
    t = v_mul(v_cross(u, v), 2.0)
    return v_add(v_add(v, v_mul(t, q[3])), v_cross(u, t))


def q_rot_inv(q, v):
    """Rotate parent-frame vector v into the body frame."""
    return q_rot(q_conj(q), v)


def q_axis_angle(axis, ang):
    s = math.sin(0.5 * ang)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(0.5 * ang)]


def q_angle(q1, q2):
    """Smallest rotation angle [rad] between two attitudes."""
    dot = abs(sum(q1[i] * q2[i] for i in range(4)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def q_look_at(pos, target):
    """Attitude whose body +X points from pos to target (camera on +X)."""
    x = v_unit(v_sub(target, pos))
    up = [0.0, 0.0, 1.0] if abs(x[2]) < 0.95 else [0.0, 1.0, 0.0]
    z = v_unit(v_cross(x, up))
    y = v_cross(z, x)
    m00, m11, m22 = x[0], y[1], z[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        q = [(y[2] - z[1]) / s, (z[0] - x[2]) / s, (x[1] - y[0]) / s, 0.25 * s]
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        q = [0.25 * s, (y[0] + x[1]) / s, (z[0] + x[2]) / s, (y[2] - z[1]) / s]
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        q = [(y[0] + x[1]) / s, 0.25 * s, (z[1] + y[2]) / s, (z[0] - x[2]) / s]
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2
        q = [(z[0] + x[2]) / s, (z[1] + y[2]) / s, 0.25 * s, (x[1] - y[0]) / s]
    return q_normalize(q)
