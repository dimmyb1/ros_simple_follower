#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from simple_follower.msg import Position
import math

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        
        self.subscription = self.create_subscription(
            Position, 
            '/object_tracker/current_position', 
            self.update_command, 
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control parameters
        self.target_dist = 0.8
        self.k_v = 0.5  # Velocity gain
        self.k_w = 1.0  # Rotation gain
        self.max_speed = 0.5
        
        self.get_logger().info('Follower Node Started')

    def update_command(self, msg):
        twist = Twist()
        
        # Angular control (turn towards object)
        # Using angle_x from our message
        twist.angular.z = self.k_w * msg.angle_x
        
        # Linear control (keep distance)
        error_dist = msg.distance - self.target_dist
        twist.linear.x = self.k_v * error_dist
        
        # Simple safety limits
        twist.linear.x = min(twist.linear.x, self.max_speed)
        twist.linear.x = max(twist.linear.x, -self.max_speed)

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    follower = Follower()
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()