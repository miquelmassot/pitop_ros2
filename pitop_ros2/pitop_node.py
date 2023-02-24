import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from pitop_ros2.get_quaternion_from_euler import get_quaternion_from_euler
from pitop_ros2.pitop_driver import Pitop


class PitopNode(Node):
    def __init__(self):
        super().__init__("robot_node")

        # chassis setup
        self.wheel_separation = 0.163
        self.wheel_diameter = 0.065
        self.timer_period = 0.01  # seconds

        self.declare_parameter("wheel_separation", self.wheel_separation)
        self.declare_parameter("wheel_diameter", self.wheel_diameter)
        self.declare_parameter("timer_period", self.timer_period)

        self.wheel_separation = self.get_parameter("wheel_separation").value
        self.wheel_diameter = self.get_parameter("wheel_diameter").value
        self.timer_period = self.get_parameter("timer_period").value

        # Create twist subscriber
        self.twist_sub = self.create_subscription(
            Twist, "cmd_vel", self.twist_callback, 10
        )
        self.left_wheel_rpm_pub = self.create_publisher(Float64, "left_wheel_rpm", 10)
        self.right_wheel_rpm_pub = self.create_publisher(Float64, "right_wheel_rpm", 10)
        self.imu_pub = self.create_publisher(Imu, "imu", 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Gyroscope resolution from datasheet is 1/131 = 0.0076 degrees/second +/- 1.5%
        # Measure variance from stationary IMU is sigma**2 = 0.0018 over approx 200 samples
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu"
        self.imu_msg.orientation_covariance = [0] * 9
        self.imu_msg.angular_velocity_covariance = [0] * 9
        self.imu_msg.linear_acceleration_covariance = [0] * 9

        self.imu_msg.orientation_covariance[0] = 0.5**2
        self.imu_msg.orientation_covariance[4] = 0.5**2
        self.imu_msg.orientation_covariance[8] = 0.5**2

        self.imu_msg.angular_velocity_covariance[0] = np.radians(0.0076) ** 2
        self.imu_msg.angular_velocity_covariance[4] = np.radians(0.0076) ** 2
        self.imu_msg.angular_velocity_covariance[8] = np.radians(0.0076) ** 2

        self.imu_msg.linear_acceleration_covariance[0] = 0.0018**2
        self.imu_msg.linear_acceleration_covariance[4] = 0.0018**2
        self.imu_msg.linear_acceleration_covariance[8] = 0.0018**2

        # Instance the PiTop controller
        self.controller = Pitop(
            wheel_diameter=self.wheel_diameter, wheel_separation=self.wheel_separation
        )

        if self.controller.imu is None:
            self.get_logger().warn("IMU not found or not available on this device")
            self.get_logger().warn("The IMU will not be published")

    def twist_callback(self, msg):
        self.get_logger().info(
            'I heard: "%s"' % msg.linear.x
            + ' "%s"' % msg.linear.y
            + ' "%s"' % msg.linear.z
            + ' "%s"' % msg.angular.x
            + ' "%s"' % msg.angular.y
            + ' "%s"' % msg.angular.z
        )
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        self.controller.robot_move(linear_speed, angular_speed)

    def timer_callback(self):
        # Publish wheel RPM
        left_wheel_rpm , right_wheel_rpm = self.controller.current_rpm()
        self.left_wheel_rpm_pub.publish(Float64(data=left_wheel_rpm))
        self.right_wheel_rpm_pub.publish(Float64(data=right_wheel_rpm))

        # Publish IMU if available
        if self.controller.imu is not None:
            acc = self.controller.imu.accelerometer
            gyro = self.controller.imu.gyroscope
            ori = self.controller.imu.orientation_radians
            q = get_quaternion_from_euler(ori.roll, ori.pitch, ori.yaw)

            self.imu_msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_msg.orientation.x = q.x
            self.imu_msg.orientation.y = q.y
            self.imu_msg.orientation.z = q.z
            self.imu_msg.orientation.w = q.w
            self.imu_msg.angular_velocity.x = np.radians(gyro.x)
            self.imu_msg.angular_velocity.y = np.radians(gyro.y)
            self.imu_msg.angular_velocity.z = np.radians(gyro.z)
            self.imu_msg.linear_acceleration.x = acc.x * 9.81
            self.imu_msg.linear_acceleration.y = acc.y * 9.81
            self.imu_msg.linear_acceleration.z = acc.z * 9.81
            self.imu_pub.publish(self.imu_msg)


def main(args=None):
    rclpy.init(args=args)
    robot_node = PitopNode()
    rclpy.spin(robot_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
