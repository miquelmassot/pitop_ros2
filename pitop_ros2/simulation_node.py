import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import numpy as np

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class PiTopSimulationNode:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        self.target_twist = Twist()
        self.node = rclpy.create_node("pitop_simulation_node")
        self.vel_sub = self.node.create_subscription(
            Twist, "/pitop/cmd_vel", self.cmd_vel_callback, 1
        )

        self.robot = webots_node.robot
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.lidar = self.robot.getDevice("lidar")
        sampling_period_in_ms = 100
        self.lidar.enable(sampling_period_in_ms)
        self.lidar.enablePointCloud()

        self.left_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)

        self.right_motor.setPosition(float("inf"))
        self.right_motor.setVelocity(0)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        forward_speed = self.target_twist.linear.x
        angular_speed = self.target_twist.angular.z

        command_motor_left = (
            forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS
        command_motor_right = (
            forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS
        ) / WHEEL_RADIUS

        self.left_motor.setVelocity(command_motor_left)
        self.right_motor.setVelocity(command_motor_right)


def main():
    pass


if __name__ == "__main__":
    main()
