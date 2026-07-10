import math

from match_mur_gui.alignment import (
    AXIS_VECTORS,
    PLANE_ALIGNMENTS,
    alignment_quaternion,
    alignment_rotation,
    plane_alignments,
)


def column(matrix, index):
    return tuple(row[index] for row in matrix)


def determinant(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def quaternion_rotation(quaternion):
    x_value, y_value, z_value, w_value = quaternion
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


def test_all_six_alignment_variants_are_available():
    assert len(PLANE_ALIGNMENTS) == 6
    assert len(plane_alignments("XY")) == 2
    assert len(plane_alignments("XZ")) == 2
    assert len(plane_alignments("YZ")) == 2


def test_tool_axes_match_each_alignment_definition():
    for alignment in PLANE_ALIGNMENTS:
        rotation = alignment_rotation(alignment)
        assert column(rotation, 0) == AXIS_VECTORS[alignment.tool_x_axis]
        assert column(rotation, 2) == AXIS_VECTORS[alignment.tool_z_axis]
        assert math.isclose(determinant(rotation), 1.0, abs_tol=1.0e-12)


def test_alignment_quaternions_are_normalized():
    for alignment in PLANE_ALIGNMENTS:
        quaternion = alignment_quaternion(alignment)
        assert math.isclose(
            sum(value * value for value in quaternion),
            1.0,
            abs_tol=1.0e-12,
        )
        expected = alignment_rotation(alignment)
        actual = quaternion_rotation(quaternion)
        for row in range(3):
            for column_index in range(3):
                assert math.isclose(
                    actual[row][column_index],
                    expected[row][column_index],
                    abs_tol=1.0e-12,
                )


def test_positive_side_always_points_toward_plane_origin():
    for alignment in PLANE_ALIGNMENTS:
        if "positive" not in alignment.key:
            continue
        assert alignment.side_axis[1] == alignment.tool_z_axis[1]
        assert alignment.side_axis[0] != alignment.tool_z_axis[0]
