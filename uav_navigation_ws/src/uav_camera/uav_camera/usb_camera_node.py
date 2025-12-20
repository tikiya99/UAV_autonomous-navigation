#!/usr/bin/env python3
"""
USB Camera Node for UAV

Provides a simple ROS2 interface for USB cameras with image publishing
and optional rectification based on calibration data.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import yaml
import os


class USBCameraNode(Node):
    """
    ROS2 Node for USB camera integration
    
    Publishes:
        /camera/image_raw (sensor_msgs/Image): Raw camera images
        /camera/image_rect (sensor_msgs/Image): Rectified images (if calibration available)
        /camera/camera_info (sensor_msgs/CameraInfo): Camera calibration data
    """
    
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('camera_name', 'uav_camera')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('publish_rectified', True)
        
        self.device_id = self.get_parameter('device_id').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        self.camera_name = self.get_parameter('camera_name').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.publish_rectified = self.get_parameter('publish_rectified').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rectification_map_x = None
        self.rectification_map_y = None
        self.camera_info = CameraInfo()
        
        # Load calibration if available
        if self.calibration_file and os.path.exists(self.calibration_file):
            self.load_calibration(self.calibration_file)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device_id}')
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Get actual resolution
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera opened: {actual_width}x{actual_height} @ {actual_fps}fps')
        
        # Publishers
        self.image_raw_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.image_rect_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Create default camera info
        self.setup_camera_info(actual_width, actual_height)
        
        # Timer for frame capture
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.capture_frame)
        
        self.get_logger().info('USB Camera Node initialized')
    
    def load_calibration(self, filepath):
        """Load camera calibration from YAML file"""
        try:
            with open(filepath, 'r') as f:
                calib = yaml.safe_load(f)
            
            self.camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
            self.dist_coeffs = np.array(calib['distortion_coefficients']['data'])
            
            # Create rectification maps
            w = calib.get('image_width', self.frame_width)
            h = calib.get('image_height', self.frame_height)
            
            new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h)
            )
            
            self.rectification_map_x, self.rectification_map_y = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, None, new_camera_matrix, (w, h), cv2.CV_32FC1
            )
            
            self.get_logger().info(f'Loaded calibration from {filepath}')
            
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')
    
    def setup_camera_info(self, width, height):
        """Setup camera info message"""
        self.camera_info.header.frame_id = self.camera_frame
        self.camera_info.width = width
        self.camera_info.height = height
        
        if self.camera_matrix is not None:
            self.camera_info.k = self.camera_matrix.flatten().tolist()
            self.camera_info.d = self.dist_coeffs.flatten().tolist()
        else:
            # Default camera matrix (approximate for 640x480)
            fx = fy = 500.0
            cx = width / 2.0
            cy = height / 2.0
            self.camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Assuming no rectification/projection transforms
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [
            self.camera_info.k[0], 0.0, self.camera_info.k[2], 0.0,
            0.0, self.camera_info.k[4], self.camera_info.k[5], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
    
    def capture_frame(self):
        """Capture and publish a frame"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        # Timestamp
        stamp = self.get_clock().now().to_msg()
        
        # Publish raw image
        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.camera_frame
        self.image_raw_pub.publish(raw_msg)
        
        # Publish rectified image if calibration available
        if self.publish_rectified and self.rectification_map_x is not None:
            rectified = cv2.remap(
                frame, 
                self.rectification_map_x, 
                self.rectification_map_y, 
                cv2.INTER_LINEAR
            )
            rect_msg = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
            rect_msg.header.stamp = stamp
            rect_msg.header.frame_id = self.camera_frame
            self.image_rect_pub.publish(rect_msg)
        
        # Publish camera info
        self.camera_info.header.stamp = stamp
        self.camera_info_pub.publish(self.camera_info)
    
    def destroy_node(self):
        """Clean up camera resources"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
