import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define package name - ensure this matches your package.xml
    package_name = 'simple_follower'

    # 1. Laser Tracker Node
    # This node processes the LIDAR scan and finds the target
    laser_tracker_node = Node(
        package=package_name,
        executable='laserTracker.py',
        name='laser_tracker',
        parameters=[{
            'winSize': 2,
            'deltaDist': 0.2,
            'showWindow': True,
            'frame_id': 'laser_frame', # Match your robot's lidar frame
        }],
        remappings=[
            ('/scan', '/scan'), # (Internal Topic, Actual Topic)
        ],
        output='screen'
    )

    # 2. Follower Node
    # This node takes the target position and generates movement commands
    follower_node = Node(
        package=package_name,
        executable='follower.py', 
        name='follower',
        parameters=[{
            'target_dist': 0.5,     # Distance to keep from object (meters)
            'max_speed': 0.5,       # Linear speed limit
            'max_angular_speed': 1.5,
            'p_gain_dist': 1.0,     # PID Proportional gain for distance
            'p_gain_angle': 2.0,    # PID Proportional gain for steering
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/object_tracker/current_position', '/object_tracker/current_position'),
        ],
        output='screen'
    )

    return LaunchDescription([
        laser_tracker_node,
        follower_node
    ])