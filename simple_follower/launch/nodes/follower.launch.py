from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_follower',
            executable='laserTracker.py',
            name='laser_tracker',
            output='screen'
        ),
        Node(
            package='simple_follower',
            executable='follower.py',
            name='follower_controller',
            output='screen'
        )
    ])