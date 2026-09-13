from niihan_description.differential_drive import twist_to_wheel_velocities


def test_forward_and_reverse_are_equal_on_both_front_wheels():
    forward = twist_to_wheel_velocities(1.0, 0.0, 0.14, 0.43, 10.0)
    reverse = twist_to_wheel_velocities(-1.0, 0.0, 0.14, 0.43, 10.0)
    assert forward[0] == forward[1]
    assert reverse[0] == reverse[1]
    assert forward[0] > 0.0
    assert reverse[0] < 0.0


def test_turning_changes_only_the_front_wheel_speed_difference():
    left, right = twist_to_wheel_velocities(0.56, 0.5, 0.14, 0.43, 10.0)
    assert left < right


def test_in_place_rotation_has_opposite_signed_wheel_speeds():
    left, right = twist_to_wheel_velocities(0.0, 1.0, 0.14, 0.43, 10.0)
    assert left < 0.0 < right


def test_each_wheel_is_limited_independently():
    assert twist_to_wheel_velocities(5.0, 0.0, 0.14, 0.43, 10.0) == (10.0, 10.0)