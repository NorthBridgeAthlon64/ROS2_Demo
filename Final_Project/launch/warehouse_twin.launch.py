from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='warehouse_digital_twin',
            executable='robot_simulator',
            name='robot_simulator',
            output='screen'
        ),
        Node(
            package='warehouse_digital_twin',
            executable='warehouse_controller',
            name='warehouse_controller',
            output='screen'
        ),
        Node(
            package='warehouse_digital_twin',
            executable='visualizer',
            name='visualizer',
            output='screen'
        ),
    ])
