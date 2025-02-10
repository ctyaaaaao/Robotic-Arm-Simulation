from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tm_robot_simulation',
            executable='tm_arm_simulator',
            name='tm_arm_simulator',
            output='screen'
        ),
    ])