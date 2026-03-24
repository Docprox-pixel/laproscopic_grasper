#!/usr/bin/env python3
"""
Surgical Tissue Perception Node
===============================
Detects the red tissue (liver) in the camera feed using color segmentation.
Publishes the target pose for autonomous navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class TissuePerceptionNode(Node):
    def __init__(self):
        super().__init__('tissue_perception_node')
        self.bridge = CvBridge()
        
        # Subscribers
        self.create_subscription(Image, '/surgical_camera/image_raw', self._image_cb, 10)
        
        # Publishers
        self.target_pub = self.create_publisher(PoseStamped, '/surgical_robot/target_tissue_pose', 10)
        
        # Camera Params (fixed from SDF)
        self.cam_z = 2.0
        self.tissue_z = 1.0
        self.img_w = 640
        self.img_h = 480
        self.hfov = 1.047 # 60 deg
        
        self.get_logger().info("Tissue Perception Node - ONLINE")

    def _image_cb(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Red Tissue Detection (HSV mask)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Red ranges
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find largest contour
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    # Draw for visualization (simulating "Visualization" requirement)
                    cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                    cv2.putText(cv_image, "TARGET TISSUE", (cx + 15, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
                    # Project to 3D World Coordinates (Simplified)
                    # Camera is at (0.5, 0, 2.0) looking down.
                    # x_world = x_cam_center + (cx - img_w/2) * scale
                    
                    # Focal length in pixels
                    f = (self.img_w / 2.0) / np.tan(self.hfov / 2.0)
                    
                    dz = self.cam_z - self.tissue_z
                    world_x = 0.5 + ((cx - self.img_w/2) / f) * dz
                    world_y = 0.0 + ((cy - self.img_h/2) / f) * dz
                    
                    self._publish_target(world_x, world_y, self.tissue_z)

        # Show detection window
        cv2.imshow("Tissue Tracking", cv_image)
        cv2.waitKey(1)

    def _publish_target(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        # Fixed top-down orientation for grasper
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.707
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.707
        self.target_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TissuePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
