"""Pure differential-drive wheel-speed conversion helpers."""


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def twist_to_wheel_velocities(
    linear_velocity: float,
    angular_velocity: float,
    wheel_radius: float,
    track_width: float,
    max_wheel_velocity: float,
) -> tuple[float, float]:
    """Return signed left and right wheel speeds in rad/s.

    Positive angular velocity is a left turn. Saturation is applied to each
    wheel independently, preserving the requested turning direction.
    """
    if wheel_radius <= 0.0 or track_width <= 0.0 or max_wheel_velocity <= 0.0:
        raise ValueError('wheel dimensions and velocity limit must be positive')

    turn_velocity = angular_velocity * track_width / 2.0
    left = (linear_velocity - turn_velocity) / wheel_radius
    right = (linear_velocity + turn_velocity) / wheel_radius
    return (
        _clamp(left, max_wheel_velocity),
        _clamp(right, max_wheel_velocity),
    )