import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import math
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster

class RobotSimNode(Node):

    def __init__(self):
        super().__init__('robot_sim_node')

        # Simulation parameters
        self.dt = self.declare_parameter('dt', 0.02).value
        self.wheel_radius = self.declare_parameter('wheel_radius', 0.05).value
        self.wheel_base = self.declare_parameter('wheel_base', 0.30).value

        # Command profile in body frame
        self.v_cmd = self.declare_parameter('v_cmd', 0.50).value
        self.w_cmd = self.declare_parameter('w_cmd', 0.20).value

        # Ground truth state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Wheel odometry estimate state
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.yaw_odom = 0.0

        # Noise parameters
        self.left_wheel_noise_std = self.declare_parameter('left_wheel_noise_std', 0.12).value
        self.right_wheel_noise_std = self.declare_parameter('right_wheel_noise_std', 0.12).value
        self.imu_yaw_noise_std = self.declare_parameter('imu_yaw_noise_std', 0.06).value
        self.imu_wz_noise_std = self.declare_parameter('imu_wz_noise_std', 0.03).value
        random_seed = int(self.declare_parameter('random_seed', 42).value)
        self.rng = np.random.default_rng(random_seed)

        # Publishers
        self.gt_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info(
            'Robot simulation started. dt={:.3f}, seed={}, noise(l={:.3f}, r={:.3f}, yaw={:.3f}, wz={:.3f})'.format(
                self.dt,
                random_seed,
                self.left_wheel_noise_std,
                self.right_wheel_noise_std,
                self.imu_yaw_noise_std,
                self.imu_wz_noise_std,
            )
        )


    def update(self):
        now = self.get_clock().now().to_msg()

        # Convert command to wheel angular rates (rad/s)
        v_l = (self.v_cmd - (self.w_cmd * self.wheel_base / 2.0)) / self.wheel_radius
        v_r = (self.v_cmd + (self.w_cmd * self.wheel_base / 2.0)) / self.wheel_radius

        # Ground truth integration from true wheel rates
        v_true = self.wheel_radius * (v_r + v_l) / 2.0
        w_true = self.wheel_radius * (v_r - v_l) / self.wheel_base
        self.x += v_true * math.cos(self.yaw) * self.dt
        self.y += v_true * math.sin(self.yaw) * self.dt
        self.yaw += w_true * self.dt

        # Wheel odometry integration from noisy wheel rates
        v_l_noisy = v_l + self.rng.normal(0.0, self.left_wheel_noise_std)
        v_r_noisy = v_r + self.rng.normal(0.0, self.right_wheel_noise_std)
        v_odom = self.wheel_radius * (v_r_noisy + v_l_noisy) / 2.0
        w_odom = self.wheel_radius * (v_r_noisy - v_l_noisy) / self.wheel_base
        self.x_odom += v_odom * math.cos(self.yaw_odom) * self.dt
        self.y_odom += v_odom * math.sin(self.yaw_odom) * self.dt
        self.yaw_odom += w_odom * self.dt

        # Ground truth odometry + TF
        self.publish_odometry(
            publisher=self.gt_pub,
            stamp=now,
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            v=v_true,
            w=w_true,
            child_frame_id='base_link'
        )
        self.publish_tf(now=now, x=self.x, y=self.y, yaw=self.yaw)

        # Wheel odometry
        self.publish_odometry(
            publisher=self.odom_pub,
            stamp=now,
            x=self.x_odom,
            y=self.y_odom,
            yaw=self.yaw_odom,
            v=v_odom,
            w=w_odom,
            child_frame_id='base_link'
        )

        # IMU yaw with noise
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = 'base_link'
        imu_msg.orientation = self.yaw_to_quaternion(
            self.yaw + self.rng.normal(0.0, self.imu_yaw_noise_std)
        )
        imu_msg.angular_velocity.z = w_true + self.rng.normal(0.0, self.imu_wz_noise_std)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        self.imu_pub.publish(imu_msg)

    def publish_odometry(self, publisher, stamp, x, y, yaw, v, w, child_frame_id):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self.yaw_to_quaternion(yaw)
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        publisher.publish(msg)

    def publish_tf(self, now, x, y, yaw):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self.yaw_to_quaternion(yaw)
        self.tf_broadcaster.sendTransform(tf_msg)

    @staticmethod
    def yaw_to_quaternion(yaw):
        half = yaw * 0.5
        return Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(half),
            w=math.cos(half)
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
