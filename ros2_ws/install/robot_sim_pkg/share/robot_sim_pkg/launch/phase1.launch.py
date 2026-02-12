from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_sim_pkg')
    rviz_config = os.path.join(pkg_share, 'rviz', 'phase1.rviz')

    sim_node = Node(
        package='robot_sim_pkg',
        executable='robot_sim_node',
        name='robot_sim_node',
        output='screen',
    )

    ekf_node = Node(
        package='state_estimator_pkg',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([sim_node, ekf_node, rviz_node])
