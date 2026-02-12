#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
# Assuming the custom message is in the same package 'simple_follower'
# If the package name in CMakeLists.txt is different, change this line.
from simple_follower.msg import Position

class VisualTracker(Node):
    def __init__(self):
        super().__init__('visual_tracker')

        # --- Parameters ---
        # Declare parameters so they can be changed from launch files
        self.declare_parameter('target_hue_min', 0)
        self.declare_parameter('target_hue_max', 10) # Red-ish by default
        self.declare_parameter('camera_topic_rgb', '/camera/rgb/image_raw')
        self.declare_parameter('camera_topic_depth', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/rgb/camera_info')
       
        # --- Variables ---
        self.bridge = CvBridge()
        self.latest_depth_image = None
        self.camera_fov_horizontal = 60.0  # Default fallback (degrees)
        self.image_width = 640 # Default fallback
       
        # --- Subscribers ---
        # We use qos_profile_sensor_data (Best Effort) to match standard camera drivers
       
        # 1. Camera Info (to get FOV and width automatically)
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            10
        )

        # 2. RGB Image
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic_rgb').value,
            self.rgb_callback,
            qos_profile_sensor_data
        )

        # 3. Depth Image
        self.create_subscription(
            Image,
            self.get_parameter('camera_topic_depth').value,
            self.depth_callback,
            qos_profile_sensor_data
        )

        # --- Publishers ---
        self.position_publisher = self.create_publisher(
            Position,
            '/object_tracker/current_position',
            10
        )
       
        # Debug publisher (publishes the mask to see what the robot sees)
        self.debug_publisher = self.create_publisher(
            Image,
            '/object_tracker/debug_view',
            10
        )

        self.get_logger().info("Visual Tracker Node Started (ROS 2)")

    def camera_info_callback(self, msg):
        # Calculate horizontal FOV from CameraInfo K matrix (fx)
        # FOV = 2 * arctan(width / (2 * fx))
        self.image_width = msg.width
        fx = msg.k[0]
        if fx > 0:
            self.camera_fov_horizontal = 2 * np.arctan(msg.width / (2 * fx)) * (180 / np.pi)

    def depth_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            # 'passthrough' preserves the 16UC1 or 32FC1 encoding
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def rgb_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"RGB conversion error: {e}")
            return

        # 1. Convert to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 2. Define Range of color to track (Get from params)
        hue_min = self.get_parameter('target_hue_min').value
        hue_max = self.get_parameter('target_hue_max').value
       
        # Standard HSV saturation/value limits for robust color detection
        lower_limit = np.array([hue_min, 100, 100])
        upper_limit = np.array([hue_max, 255, 255])

        # 3. Create Mask
        mask = cv2.inRange(hsv_image, lower_limit, upper_limit)

        # 4. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the largest contour (assuming it's the target)
            c = max(contours, key=cv2.contourArea)
           
            # Get bounding box or moments
            M = cv2.moments(c)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # 5. Get Depth at Centroid
                distance = 0.0
                if self.latest_depth_image is not None:
                    try:
                        # Handle bounds check
                        d_y = min(max(cy, 0), self.latest_depth_image.shape[0]-1)
                        d_x = min(max(cx, 0), self.latest_depth_image.shape[1]-1)
                       
                        raw_dist = self.latest_depth_image[d_y, d_x]
                       
                        # Handle different depth encodings (mm vs meters)
                        # Usually 16UC1 is mm, 32FC1 is meters.
                        if isinstance(raw_dist, np.uint16):
                            distance = raw_dist / 1000.0
                        else:
                            distance = float(raw_dist)
                           
                    except IndexError:
                        pass
               
                # 6. Calculate Angle
                # Map x pixel (0 to width) to angle (-FOV/2 to +FOV/2)
                # 0 degrees is center of image
                angle = (cx - (self.image_width/2)) * (self.camera_fov_horizontal / self.image_width)
               
                # 7. Publish
                msg = Position()
                msg.angleX = float(angle)
                msg.angleY = 0.0 # Not calculated
                msg.distance = float(distance)
                self.position_publisher.publish(msg)

                # Draw circle on debug image
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), 2)

        # Publish debug image
        try:
            self.debug_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError:
            pass

def main(args=None):
    rclpy.init(args=args)
    tracker = VisualTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()