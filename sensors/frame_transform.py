import math


def rotation_matrix(angles):
    """Rotation capteur -> robot : Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = angles  # radians

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return (
        (
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
        ),
        (
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
        ),
        (
            -sp,
            cp * sr,
            cp * cr,
        ),
    )


def rotate_vector(vector, matrix):
    """Produit matrice 3×3 par vecteur 3×1."""
    return tuple(
        sum(matrix[i][j] * vector[j] for j in range(3))
        for i in range(3)
    )
