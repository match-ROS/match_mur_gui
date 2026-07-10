import math

from match_mur_gui.alignment import (
    AXIS_VECTORS,
    PLANE_ALIGNMENTS,
    matrix_multiply,
    matrix_to_quaternion,
    nearest_alignment_quaternion,
    nearest_alignment_rotation,
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


def transpose(matrix):
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def rotation_x(angle):
    return (
        (1.0, 0.0, 0.0),
        (0.0, math.cos(angle), -math.sin(angle)),
        (0.0, math.sin(angle), math.cos(angle)),
    )


def rotation_y(angle):
    return (
        (math.cos(angle), 0.0, math.sin(angle)),
        (0.0, 1.0, 0.0),
        (-math.sin(angle), 0.0, math.cos(angle)),
    )


def rotation_z(angle):
    return (
        (math.cos(angle), -math.sin(angle), 0.0),
        (math.sin(angle), math.cos(angle), 0.0),
        (0.0, 0.0, 1.0),
    )


def test_all_six_alignment_variants_are_available():
    assert len(PLANE_ALIGNMENTS) == 6
    assert len(plane_alignments("XY")) == 2
    assert len(plane_alignments("XZ")) == 2
    assert len(plane_alignments("YZ")) == 2


def test_nearest_alignment_quaternions_match_the_target_rotation():
    current_rotation = matrix_multiply(
        rotation_z(-0.63),
        matrix_multiply(rotation_y(0.37), rotation_x(0.24)),
    )
    current_quaternion = matrix_to_quaternion(current_rotation)
    for alignment in PLANE_ALIGNMENTS:
        quaternion = nearest_alignment_quaternion(current_quaternion, alignment)
        assert math.isclose(
            sum(value * value for value in quaternion),
            1.0,
            abs_tol=1.0e-12,
        )
        expected = nearest_alignment_rotation(current_quaternion, alignment)
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


def test_nearest_alignment_only_changes_tool_z_as_much_as_required():
    current_rotation = matrix_multiply(
        rotation_z(0.73),
        matrix_multiply(rotation_y(0.48), rotation_x(-0.31)),
    )
    current_quaternion = matrix_to_quaternion(current_rotation)
    for alignment in PLANE_ALIGNMENTS:
        target_rotation = nearest_alignment_rotation(current_quaternion, alignment)
        target_tool_z = column(target_rotation, 2)
        expected_tool_z = AXIS_VECTORS[alignment.tool_z_axis]
        for actual, expected in zip(target_tool_z, expected_tool_z):
            assert math.isclose(actual, expected, abs_tol=1.0e-12)
        assert math.isclose(determinant(target_rotation), 1.0, abs_tol=1.0e-12)


def test_xy_alignment_adds_no_rotation_about_base_z():
    current_rotation = matrix_multiply(
        rotation_z(0.91),
        matrix_multiply(rotation_y(0.42), rotation_x(-0.27)),
    )
    current_quaternion = matrix_to_quaternion(current_rotation)
    target_rotation = nearest_alignment_rotation(current_quaternion, "xy_negative")
    correction = matrix_multiply(target_rotation, transpose(current_rotation))
    correction_axis_z = correction[1][0] - correction[0][1]
    assert math.isclose(correction_axis_z, 0.0, abs_tol=1.0e-12)


def test_alignment_keeps_an_already_valid_orientation_unchanged():
    current_rotation = matrix_multiply(rotation_z(1.17), rotation_x(math.pi))
    current_quaternion = matrix_to_quaternion(current_rotation)
    target_rotation = nearest_alignment_rotation(current_quaternion, "xy_positive")
    for row in range(3):
        for column_index in range(3):
            assert math.isclose(
                target_rotation[row][column_index],
                current_rotation[row][column_index],
                abs_tol=1.0e-12,
            )
