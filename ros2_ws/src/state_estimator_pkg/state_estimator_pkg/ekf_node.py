import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


class EkfNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        # State: [x, y, yaw]
        self.x = np.zeros(3)
        self.P = np.diag([1.0, 1.0, 0.5])

        # Noise
        self.q_pos = 0.02
        self.q_yaw = 0.03
        self.r_odom_pos = 0.05
        self.r_imu_yaw = 0.04

        # Measurement cache
        self.last_imu_yaw = None
        self.last_odom_stamp = None
        self.raw_state = None
        self.filtered_state = None

        # Paths
        self.raw_path = Path()
        self.raw_path.header.frame_id = 'odom'
        self.filtered_path = Path()
        self.filtered_path.header.frame_id = 'odom'
        self.gt_path = Path()
        self.gt_path.header.frame_id = 'odom'
        self.path_max_points = 1200

        # Metrics
        self.sample_count = 0
        self.sum_sq_raw_pos = 0.0
        self.sum_sq_filt_pos = 0.0
        self.sum_sq_raw_yaw = 0.0
        self.sum_sq_filt_yaw = 0.0
        self.start_gt = None

        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel_odom, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu, 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_ground_truth, 20)

        self.filtered_odom_pub = self.create_publisher(Odometry, '/ekf/odom', 20)
        self.raw_path_pub = self.create_publisher(Path, '/wheel/path', 10)
        self.filtered_path_pub = self.create_publisher(Path, '/ekf/path', 10)
        self.gt_path_pub = self.create_publisher(Path, '/ground_truth/path', 10)

        self.metrics_timer = self.create_timer(5.0, self.log_metrics)
        self.get_logger().info('EKF node started.')

    def on_imu(self, msg):
        self.last_imu_yaw = yaw_from_quaternion(msg.orientation)

    def on_wheel_odom(self, msg):
        stamp = msg.header.stamp
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if self.last_odom_stamp is None:
            self.last_odom_stamp = t
            self.raw_state = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation),
            ])
            self.x = self.raw_state.copy()
            self.filtered_state = self.x.copy()
            return

        dt = max(1e-3, t - self.last_odom_stamp)
        self.last_odom_stamp = t

        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        # Predict
        yaw = self.x[2]
        self.x[0] += v * math.cos(yaw) * dt
        self.x[1] += v * math.sin(yaw) * dt
        self.x[2] = wrap_angle(self.x[2] + w * dt)

        F = np.array([
            [1.0, 0.0, -v * math.sin(yaw) * dt],
            [0.0, 1.0, v * math.cos(yaw) * dt],
            [0.0, 0.0, 1.0],
        ])
        Q = np.diag([self.q_pos * dt, self.q_pos * dt, self.q_yaw * dt])
        self.P = F @ self.P @ F.T + Q

        # Update from wheel odometry position (soft correction)
        z_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        H_pos = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        R_pos = np.diag([self.r_odom_pos, self.r_odom_pos])
        y_pos = z_pos - H_pos @ self.x
        S_pos = H_pos @ self.P @ H_pos.T + R_pos
        K_pos = self.P @ H_pos.T @ np.linalg.inv(S_pos)
        self.x = self.x + K_pos @ y_pos
        self.x[2] = wrap_angle(self.x[2])
        self.P = (np.eye(3) - K_pos @ H_pos) @ self.P

        # Update from IMU yaw (yaw-only correction)
        if self.last_imu_yaw is not None:
            z = self.last_imu_yaw
            y_residual = wrap_angle(z - self.x[2])
            p22 = self.P[2, 2]
            k_yaw = p22 / (p22 + self.r_imu_yaw)
            self.x[2] = wrap_angle(self.x[2] + k_yaw * y_residual)
            self.P[2, 2] = (1.0 - k_yaw) * p22

            # Keep position terms from being perturbed by yaw-only update.
            self.P[0, 2] = 0.0
            self.P[1, 2] = 0.0
            self.P[2, 0] = 0.0
            self.P[2, 1] = 0.0

        self.raw_state = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        ])
        self.filtered_state = self.x.copy()
        self.publish_filtered_odom(stamp, v, w)
        self.append_and_publish_paths(stamp)

    def on_ground_truth(self, msg):
        stamp = msg.header.stamp
        gt = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation),
        ])

        if self.start_gt is None:
            self.start_gt = gt.copy()

        self.append_path_point(self.gt_path, stamp, gt)
        self.gt_path_pub.publish(self.gt_path)

        if self.raw_state is None or self.filtered_state is None:
            return

        raw_pos_err = np.linalg.norm(gt[:2] - self.raw_state[:2])
        filt_pos_err = np.linalg.norm(gt[:2] - self.filtered_state[:2])
        raw_yaw_err = abs(wrap_angle(gt[2] - self.raw_state[2]))
        filt_yaw_err = abs(wrap_angle(gt[2] - self.filtered_state[2]))

        self.sum_sq_raw_pos += raw_pos_err ** 2
        self.sum_sq_filt_pos += filt_pos_err ** 2
        self.sum_sq_raw_yaw += raw_yaw_err ** 2
        self.sum_sq_filt_yaw += filt_yaw_err ** 2
        self.sample_count += 1

    def publish_filtered_odom(self, stamp, v, w):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link_ekf'
        msg.pose.pose.position.x = float(self.x[0])
        msg.pose.pose.position.y = float(self.x[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(float(self.x[2]))
        msg.twist.twist.linear.x = float(v)
        msg.twist.twist.angular.z = float(w)
        self.filtered_odom_pub.publish(msg)

    def append_path_point(self, path_msg, stamp, state):
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'odom'
        pose.pose.position.x = float(state[0])
        pose.pose.position.y = float(state[1])
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(state[2]))
        path_msg.header.stamp = stamp
        path_msg.poses.append(pose)
        if len(path_msg.poses) > self.path_max_points:
            path_msg.poses.pop(0)

    def append_and_publish_paths(self, stamp):
        if self.raw_state is not None:
            self.append_path_point(self.raw_path, stamp, self.raw_state)
            self.raw_path_pub.publish(self.raw_path)
        if self.filtered_state is not None:
            self.append_path_point(self.filtered_path, stamp, self.filtered_state)
            self.filtered_path_pub.publish(self.filtered_path)

    def log_metrics(self):
        if self.sample_count == 0 or self.start_gt is None:
            return

        rmse_raw_pos = math.sqrt(self.sum_sq_raw_pos / self.sample_count)
        rmse_filt_pos = math.sqrt(self.sum_sq_filt_pos / self.sample_count)
        rmse_raw_yaw = math.sqrt(self.sum_sq_raw_yaw / self.sample_count)
        rmse_filt_yaw = math.sqrt(self.sum_sq_filt_yaw / self.sample_count)

        rmse_reduction = 0.0
        if rmse_raw_pos > 1e-9:
            rmse_reduction = 100.0 * (rmse_raw_pos - rmse_filt_pos) / rmse_raw_pos

        current_gt_xy = self.gt_path.poses[-1].pose.position
        gt_now = np.array([current_gt_xy.x, current_gt_xy.y])
        start_xy = self.start_gt[:2]
        travel_dist = np.linalg.norm(gt_now - start_xy)

        raw_drift = np.linalg.norm(gt_now - self.raw_state[:2])
        filt_drift = np.linalg.norm(gt_now - self.filtered_state[:2])
        drift_reduction = None
        if raw_drift > 0.05:
            drift_reduction = 100.0 * (raw_drift - filt_drift) / raw_drift

        drift_msg = 'drift raw={:.3f} m filt={:.3f} m'.format(raw_drift, filt_drift)
        if drift_reduction is None:
            drift_msg += ' (reduction=n/a; raw drift near zero)'
        else:
            drift_msg += ' (reduction={:.1f}%)'.format(drift_reduction)

        self.get_logger().info(
            'RMSE pos raw={:.3f} m filt={:.3f} m (reduction={:.1f}%), '
            'RMSE yaw raw={:.3f} rad filt={:.3f} rad, {}, gt_travel={:.3f} m'.format(
                rmse_raw_pos,
                rmse_filt_pos,
                rmse_reduction,
                rmse_raw_yaw,
                rmse_filt_yaw,
                drift_msg,
                travel_dist,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
