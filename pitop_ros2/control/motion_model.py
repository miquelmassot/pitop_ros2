import numpy as np


class RobotState:
    def __init__(self):
        self.N = 0
        self.E = 0
        self.N_dot = 0
        self.E_dot = 0
        self.x_dot = 0
        self.bearing = 0
        self.bearing_dot = 0


def motion_model(N, E, left_rpm, right_rpm, bearing, dt):
    # Constants
    wheel_diameter = 0.065
    wheel_base = 0.15

    # Calculate wheel velocities
    left_wheel_velocity = left_rpm * (wheel_diameter * np.pi) / 60
    right_wheel_velocity = right_rpm * (wheel_diameter * np.pi) / 60

    # Calculate robot velocities
    x_dot = (left_wheel_velocity + right_wheel_velocity) / 2
    bearing_dot = (right_wheel_velocity - left_wheel_velocity) / wheel_base

    # Calculate robot position
    N_dot = x_dot * np.cos(bearing)
    E_dot = x_dot * np.sin(bearing)

    # Calculate new robot position
    N = N + N_dot * dt
    E = E + E_dot * dt
    bearing = bearing + bearing_dot * dt

    return N, E, N_dot, E_dot, bearing, bearing_dot
