from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim_pkg')
    rviz_config = os.path.join(pkg_share, 'rviz', 'phase1.rviz')
    default_metrics_csv = '/tmp/ekf_metrics.csv'

    declare_args = [
        DeclareLaunchArgument('dt', default_value='0.02'),
        DeclareLaunchArgument('wheel_radius', default_value='0.05'),
        DeclareLaunchArgument('wheel_base', default_value='0.30'),
        DeclareLaunchArgument('v_cmd', default_value='0.50'),
        DeclareLaunchArgument('w_cmd', default_value='0.20'),
        DeclareLaunchArgument('left_wheel_noise_std', default_value='0.12'),
        DeclareLaunchArgument('right_wheel_noise_std', default_value='0.12'),
        DeclareLaunchArgument('imu_yaw_noise_std', default_value='0.06'),
        DeclareLaunchArgument('imu_wz_noise_std', default_value='0.03'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument('q_pos', default_value='0.02'),
        DeclareLaunchArgument('q_yaw', default_value='0.03'),
        DeclareLaunchArgument('r_odom_pos', default_value='0.05'),
        DeclareLaunchArgument('r_imu_yaw', default_value='0.15'),
        DeclareLaunchArgument('use_wheel_pos_update', default_value='true'),
        DeclareLaunchArgument('metrics_period_sec', default_value='5.0'),
        DeclareLaunchArgument('metrics_csv_path', default_value=default_metrics_csv),
    ]

    sim_node = Node(
        package='robot_sim_pkg',
        executable='robot_sim_node',
        name='robot_sim_node',
        parameters=[{
            'dt': LaunchConfiguration('dt'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'v_cmd': LaunchConfiguration('v_cmd'),
            'w_cmd': LaunchConfiguration('w_cmd'),
            'left_wheel_noise_std': LaunchConfiguration('left_wheel_noise_std'),
            'right_wheel_noise_std': LaunchConfiguration('right_wheel_noise_std'),
            'imu_yaw_noise_std': LaunchConfiguration('imu_yaw_noise_std'),
            'imu_wz_noise_std': LaunchConfiguration('imu_wz_noise_std'),
            'random_seed': LaunchConfiguration('random_seed'),
        }],
        output='screen',
    )

    ekf_node = Node(
        package='state_estimator_pkg',
        executable='ekf_node',
        name='ekf_node',
        parameters=[{
            'q_pos': LaunchConfiguration('q_pos'),
            'q_yaw': LaunchConfiguration('q_yaw'),
            'r_odom_pos': LaunchConfiguration('r_odom_pos'),
            'r_imu_yaw': LaunchConfiguration('r_imu_yaw'),
            'use_wheel_pos_update': LaunchConfiguration('use_wheel_pos_update'),
            'metrics_period_sec': LaunchConfiguration('metrics_period_sec'),
            'metrics_csv_path': LaunchConfiguration('metrics_csv_path'),
        }],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription(declare_args + [sim_node, ekf_node, rviz_node])
