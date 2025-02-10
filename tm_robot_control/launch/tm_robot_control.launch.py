from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tm_robot_control',
            executable='tm_arm_keyboard',
            name='tm_arm_keyboard',
            output='screen'
        ),
    ])