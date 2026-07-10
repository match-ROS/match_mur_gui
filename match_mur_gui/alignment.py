"""Deterministic tool orientations for aligning a TCP with base-frame planes."""

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
    tool_x_axis: str
    tool_z_axis: str
    color: str

    @property
    def side_label(self):
        return f"{self.side_axis} side"

    @property
    def direction_label(self):
        return f"TCP-Z -> {self.tool_z_axis}"


PLANE_ALIGNMENTS = (
    PlaneAlignment("xy_positive", "XY", "+Z", "+X", "-Z", "#2f855a"),
    PlaneAlignment("xy_negative", "XY", "-Z", "+X", "+Z", "#2f855a"),
    PlaneAlignment("xz_positive", "XZ", "+Y", "+X", "-Y", "#2b6cb0"),
    PlaneAlignment("xz_negative", "XZ", "-Y", "+X", "+Y", "#2b6cb0"),
    PlaneAlignment("yz_positive", "YZ", "+X", "+Y", "-X", "#b7791f"),
    PlaneAlignment("yz_negative", "YZ", "-X", "+Y", "+X", "#b7791f"),
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


def alignment_rotation(alignment):
    if not isinstance(alignment, PlaneAlignment):
        alignment = plane_alignment(alignment)
    tool_x = AXIS_VECTORS[alignment.tool_x_axis]
    tool_z = AXIS_VECTORS[alignment.tool_z_axis]
    tool_y = cross(tool_z, tool_x)
    return (
        (tool_x[0], tool_y[0], tool_z[0]),
        (tool_x[1], tool_y[1], tool_z[1]),
        (tool_x[2], tool_y[2], tool_z[2]),
    )


def alignment_quaternion(alignment):
    return matrix_to_quaternion(alignment_rotation(alignment))
