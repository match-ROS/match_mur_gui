"""Plane constraints and nearest orientations for TCP alignment."""

import math
from dataclasses import dataclass


AXIS_VECTORS = {
    "+X": (1.0, 0.0, 0.0),
    "-X": (-1.0, 0.0, 0.0),
    "+Y": (0.0, 1.0, 0.0),
    "-Y": (0.0, -1.0, 0.0),
    "+Z": (0.0, 0.0, 1.0),
    "-Z": (0.0, 0.0, -1.0),
}


@dataclass(frozen=True)
class PlaneAlignment:
    key: str
    plane: str
    side_axis: str
    tool_z_axis: str
    color: str

    @property
    def side_label(self):
        return f"{self.side_axis} side"

    @property
    def direction_label(self):
        return f"TCP-Z -> {self.tool_z_axis}"


PLANE_ALIGNMENTS = (
    PlaneAlignment("xy_positive", "XY", "+Z", "-Z", "#2f855a"),
    PlaneAlignment("xy_negative", "XY", "-Z", "+Z", "#2f855a"),
    PlaneAlignment("xz_positive", "XZ", "+Y", "-Y", "#2b6cb0"),
    PlaneAlignment("xz_negative", "XZ", "-Y", "+Y", "#2b6cb0"),
    PlaneAlignment("yz_positive", "YZ", "+X", "-X", "#b7791f"),
    PlaneAlignment("yz_negative", "YZ", "-X", "+X", "#b7791f"),
)
PLANE_ALIGNMENT_BY_KEY = {alignment.key: alignment for alignment in PLANE_ALIGNMENTS}


def plane_alignments(plane):
    plane = str(plane).strip().upper()
    return tuple(alignment for alignment in PLANE_ALIGNMENTS if alignment.plane == plane)


def plane_alignment(key):
    try:
        return PLANE_ALIGNMENT_BY_KEY[str(key).strip().lower()]
    except KeyError as exc:
        choices = ", ".join(sorted(PLANE_ALIGNMENT_BY_KEY))
        raise ValueError(f"Unknown plane alignment '{key}'; choose one of: {choices}") from exc


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return sum(first * second for first, second in zip(a, b))


def normalize(vector):
    norm = math.sqrt(dot(vector, vector))
    if norm <= 1.0e-12:
        raise ValueError("Cannot normalize a zero-length vector")
    return tuple(value / norm for value in vector)


def quaternion_to_matrix(quaternion):
    x_value, y_value, z_value, w_value = normalize(quaternion)
    return (
        (
            1.0 - 2.0 * (y_value * y_value + z_value * z_value),
            2.0 * (x_value * y_value - z_value * w_value),
            2.0 * (x_value * z_value + y_value * w_value),
        ),
        (
            2.0 * (x_value * y_value + z_value * w_value),
            1.0 - 2.0 * (x_value * x_value + z_value * z_value),
            2.0 * (y_value * z_value - x_value * w_value),
        ),
        (
            2.0 * (x_value * z_value - y_value * w_value),
            2.0 * (y_value * z_value + x_value * w_value),
            1.0 - 2.0 * (x_value * x_value + y_value * y_value),
        ),
    )


def matrix_multiply(first, second):
    return tuple(
        tuple(
            sum(first[row][index] * second[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def axis_angle_rotation(axis, angle):
    x_value, y_value, z_value = normalize(axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus_cosine = 1.0 - cosine
    return (
        (
            cosine + x_value * x_value * one_minus_cosine,
            x_value * y_value * one_minus_cosine - z_value * sine,
            x_value * z_value * one_minus_cosine + y_value * sine,
        ),
        (
            y_value * x_value * one_minus_cosine + z_value * sine,
            cosine + y_value * y_value * one_minus_cosine,
            y_value * z_value * one_minus_cosine - x_value * sine,
        ),
        (
            z_value * x_value * one_minus_cosine - y_value * sine,
            z_value * y_value * one_minus_cosine + x_value * sine,
            cosine + z_value * z_value * one_minus_cosine,
        ),
    )


def matrix_to_quaternion(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1.0e-12:
        raise ValueError("Cannot create a quaternion from a degenerate rotation matrix")
    return tuple(value / norm for value in quaternion)


def nearest_alignment_rotation(current_quaternion, alignment):
    """Apply only the shortest rotation needed to align the current tool Z axis."""
    if not isinstance(alignment, PlaneAlignment):
        alignment = plane_alignment(alignment)
    current_rotation = quaternion_to_matrix(current_quaternion)
    current_tool_z = tuple(row[2] for row in current_rotation)
    target_tool_z = AXIS_VECTORS[alignment.tool_z_axis]
    cosine = max(-1.0, min(1.0, dot(current_tool_z, target_tool_z)))
    rotation_axis = cross(current_tool_z, target_tool_z)
    sine = math.sqrt(dot(rotation_axis, rotation_axis))

    if sine <= 1.0e-12:
        if cosine > 0.0:
            return current_rotation
        # Every perpendicular axis gives a 180-degree solution. Keeping the
        # current tool X axis avoids adding an arbitrary spin around tool Z.
        rotation_axis = tuple(row[0] for row in current_rotation)
        angle = math.pi
    else:
        rotation_axis = tuple(value / sine for value in rotation_axis)
        angle = math.atan2(sine, cosine)

    correction = axis_angle_rotation(rotation_axis, angle)
    return matrix_multiply(correction, current_rotation)


def nearest_alignment_quaternion(current_quaternion, alignment):
    quaternion = matrix_to_quaternion(
        nearest_alignment_rotation(current_quaternion, alignment)
    )
    if dot(quaternion, current_quaternion) < 0.0:
        quaternion = tuple(-value for value in quaternion)
    return quaternion
