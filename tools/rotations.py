"""Rotations the converters share: matrices as tuples of rows, quaternions Hamilton and
scalar first, Euler angles ZYX. Standard library only, like the converters.
"""

import math


def rotation(roll, pitch, yaw):
    """Body (forward, right, down) to north, east, down, for ZYX Euler angles in radians."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def euler(r):
    """ZYX roll, pitch and yaw in radians of a body-to-navigation rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    return math.atan2(r[2][1], r[2][2]), pitch, math.atan2(r[1][0], r[0][0])


def rotate(r, v):
    return tuple(sum(r[i][j] * v[j] for j in range(3)) for i in range(3))


def matmul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quaternion_matrix(w, x, y, z):
    """Hamilton, scalar first, normalized: the rotation it applies to a vector."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def rotation_vector_matrix(v):
    """exp([v]x), Rodrigues."""
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-12:
        return ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    k = tuple(c / angle for c in v)
    s, c = math.sin(angle), 1 - math.cos(angle)
    kx = ((0, -k[2], k[1]), (k[2], 0, -k[0]), (-k[1], k[0], 0))
    kx2 = matmul(kx, kx)
    return tuple(tuple((i == j) + s * kx[i][j] + c * kx2[i][j] for j in range(3))
                 for i in range(3))


def angle_between(a, b):
    """The angle of the rotation taking `a` onto `b`, in radians."""
    r = matmul(transpose(a), b)
    return math.acos(max(-1.0, min(1.0, (r[0][0] + r[1][1] + r[2][2] - 1) / 2)))
