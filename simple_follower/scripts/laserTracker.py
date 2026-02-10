#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from sensor_msgs.msg import LaserScan
from simple_follower.msg import Position
from std_msgs.msg import String

class LaserTracker(Node):
    def __init__(self):
        super().__init__('laser_tracker')
        
        # Parameters
        self.winSize = 2
        self.deltaDist = 0.2
        self.lastScan = None

        # QoS for sensor data (Best Effort is often used for LaserScan)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers and Publishers
        self.subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.registerScan, 
            qos_policy
        )
        self.positionPublisher = self.create_publisher(
            Position, 
            '/object_tracker/current_position', 
            10
        )
        self.infoPublisher = self.create_publisher(
            String, 
            '/object_tracker/info', 
            10
        )
        self.get_logger().info('Laser Tracker Started')

    def registerScan(self, scan_data):
        ranges = np.array(scan_data.ranges)
        # Filter infinite values
        ranges = np.where(np.isinf(ranges), scan_data.range_max, ranges)
        
        sortedIndices = np.argsort(ranges)
        minDistanceID = None
        minDistance = float('inf')

        if self.lastScan is not None:
            for i in sortedIndices:
                tempMinDistance = ranges[i]
                # Boundary checks for window
                min_idx = max(0, i - self.winSize)
                max_idx = min(len(self.lastScan), i + self.winSize + 1)
                
                window = self.lastScan[min_idx:max_idx]
                
                if np.any(np.abs(window - tempMinDistance) <= self.deltaDist):
                    minDistanceID = i
                    minDistance = ranges[minDistanceID]
                    break
            self.lastScan = ranges
        else:
            self.lastScan = ranges
            return # Wait for next scan

        if minDistanceID is None or minDistance > scan_data.range_max:
            # self.get_logger().warn('No object found') # Uncomment to debug
            msg = String()
            msg.data = 'laser:nothing found'
            self.infoPublisher.publish(msg)
        else:
            minDistanceAngle = scan_data.angle_min + minDistanceID * scan_data.angle_increment
            
            # Publish Position
            msg = Position()
            msg.angle_x = minDistanceAngle
            msg.angle_y = 0.0 # Laser is 2D, no Y angle
            msg.distance = minDistance
            self.positionPublisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    tracker = LaserTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()