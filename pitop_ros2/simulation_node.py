import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64


from .tools.get_quaternion_from_euler import get_quaternion_from_euler

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
        self.ground_truth_odom_pub = self.node.create_publisher(
            Odometry, "/pitop/ground_truth_odometry", 1
        )
        self.imu_pub = self.node.create_publisher(Imu, "/pitop/imu", 1)
        self.left_wheel_pub = self.node.create_publisher(
            Float64, "/pitop/wheels/left_rpm", 1
        )
        self.right_wheel_pub = self.node.create_publisher(
            Float64, "/pitop/wheels/right_rpm", 1
        )
        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self.node)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)

        self.wheel_separation = 0.045 * 2
        self.wheel_diameter = 0.025 * 2

        # Get all sensors and devices
        self.robot = webots_node.robot
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.lidar = self.robot.getDevice("lidar")
        self.gps = self.robot.getDevice("gps")
        self.compass = self.robot.getDevice("compass")
        self.gyro = self.robot.getDevice("gyro")
        self.accelerometer = self.robot.getDevice("accelerometer")

        # Enable sensors
        sampling_period_in_ms = 100
        self.gps.enable(sampling_period_in_ms)
        self.compass.enable(sampling_period_in_ms)
        self.gyro.enable(sampling_period_in_ms)
        self.accelerometer.enable(sampling_period_in_ms)
        self.lidar.enable(sampling_period_in_ms)
        self.lidar.enablePointCloud()

        self.left_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0)

        self.right_motor.setPosition(float("inf"))
        self.right_motor.setVelocity(0)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def get_imu(self):
        imu = Imu()
        imu.header.stamp = self.node.get_clock().now().to_msg()
        imu.header.frame_id = "pitop"

        qx, qy, qz, qw = get_quaternion_from_euler(0, 0, self.get_bearing())
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.orientation.w = qw

        imu.angular_velocity.x = self.gyro.getValues()[0]
        imu.angular_velocity.y = self.gyro.getValues()[1]
        imu.angular_velocity.z = self.gyro.getValues()[2]

        imu.linear_acceleration.x = self.accelerometer.getValues()[0]
        imu.linear_acceleration.y = self.accelerometer.getValues()[1]
        imu.linear_acceleration.z = self.accelerometer.getValues()[2]

        return imu

    def get_odometry(self):
        odometry = Odometry()
        odometry.header.stamp = self.node.get_clock().now().to_msg()
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "pitop"
        print(self.gps.getValues())
        odometry.pose.pose.position.x = self.gps.getValues()[0]
        odometry.pose.pose.position.y = self.gps.getValues()[1]
        odometry.pose.pose.position.z = self.gps.getValues()[2]

        qx, qy, qz, qw = get_quaternion_from_euler(0, 0, self.get_bearing())

        odometry.pose.pose.orientation.x = qx
        odometry.pose.pose.orientation.y = qy
        odometry.pose.pose.orientation.z = qz
        odometry.pose.pose.orientation.w = qw
        odometry.twist.twist.linear.x = self.gps.getSpeedVector()[0]
        odometry.twist.twist.linear.y = self.gps.getSpeedVector()[1]
        odometry.twist.twist.linear.z = self.gps.getSpeedVector()[2]
        odometry.twist.twist.angular.x = self.gyro.getValues()[0]
        odometry.twist.twist.angular.y = self.gyro.getValues()[1]
        odometry.twist.twist.angular.z = self.gyro.getValues()[2]
        return odometry

    def get_bearing(self):
        compass_values = self.compass.getValues()
        bearing = np.arctan2(compass_values[0], compass_values[1])
        return bearing

    def broadcast_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "pitop"
        transform.transform.translation.x = self.gps.getValues()[0]
        transform.transform.translation.y = self.gps.getValues()[1]
        transform.transform.translation.z = self.gps.getValues()[2]
        qx, qy, qz, qw = get_quaternion_from_euler(0, 0, self.get_bearing())
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        transform2 = TransformStamped()
        transform2.header.stamp = transform.header.stamp
        transform2.header.frame_id = "pitop"
        transform2.child_frame_id = "gps"
        transform2.transform.translation.z = 0.1
        self.tf_static_broadcaster.sendTransform(transform2)

        transform3 = TransformStamped()
        transform3.header.stamp = transform.header.stamp
        transform3.header.frame_id = "pitop"
        transform3.child_frame_id = "lidar"
        transform3.transform.translation.x = 0.1
        transform3.transform.translation.z = 0.1
        self.tf_static_broadcaster.sendTransform(transform3)

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        left_rpm = Float64()
        left_rpm.data = (
            self.left_motor.getVelocity() * 60 / (self.wheel_diameter * np.pi)
        )
        right_rpm = Float64()
        right_rpm.data = (
            self.right_motor.getVelocity() * 60 / (self.wheel_diameter * np.pi)
        )

        self.left_wheel_pub.publish(left_rpm)
        self.right_wheel_pub.publish(right_rpm)

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

        self.broadcast_transform()
        self.imu_pub.publish(self.get_imu())
        self.ground_truth_odom_pub.publish(self.get_odometry())


def main():
    pass


if __name__ == "__main__":
    main()
