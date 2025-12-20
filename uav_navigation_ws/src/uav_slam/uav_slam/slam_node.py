#!/usr/bin/env python3
"""
ORB-SLAM3 ROS2 Wrapper Node

This node provides a ROS2 interface for ORB-SLAM3 monocular SLAM.
It subscribes to camera images and publishes:
- Visual odometry as nav_msgs/Odometry
- Camera pose as geometry_msgs/PoseStamped
- Map points as sensor_msgs/PointCloud2
- Tracking state

Note: This is a wrapper that expects an external ORB-SLAM3 ROS2 wrapper to be installed.
If not available, it provides a stub implementation for development/testing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header, Int32
from tf2_ros import TransformBroadcaster

import numpy as np
import struct

# Try to import ORB-SLAM3 wrapper
try:
    # This would be the actual ORB-SLAM3 ROS2 wrapper
    # from orbslam3_ros2.orbslam3_wrapper import ORBSLAM3
    ORBSLAM3_AVAILABLE = False  # Set to True when wrapper is installed
except ImportError:
    ORBSLAM3_AVAILABLE = False


class SLAMNode(Node):
    """
    ROS2 Node for ORB-SLAM3 integration
    
    Subscribes:
        /camera/image_raw (sensor_msgs/Image): Camera images for SLAM
        
    Publishes:
        /slam/odom (nav_msgs/Odometry): Visual odometry
        /slam/pose (geometry_msgs/PoseStamped): Current camera pose
        /slam/map_points (sensor_msgs/PointCloud2): 3D map points
        /slam/tracking_state (std_msgs/Int32): Tracking quality (0=Lost, 1=OK, 2=Good)
        
    TF Broadcasts:
        odom -> base_link (if publish_tf enabled)
    """
    
    def __init__(self):
        super().__init__('slam_node')
        
        # Parameters
        self.declare_parameter('vocabulary_file', '')
        self.declare_parameter('settings_file', '')
        self.declare_parameter('publish_tf', False)  # Let localization handle TF
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        
        self.vocabulary_file = self.get_parameter('vocabulary_file').value
        self.settings_file = self.get_parameter('settings_file').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # SLAM state
        self.slam_initialized = False
        self.current_pose = None
        self.tracking_state = 0  # 0=Lost, 1=OK, 2=Good
        self.map_points = []
        
        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/slam/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.map_pub = self.create_publisher(PointCloud2, '/slam/map_points', 10)
        self.state_pub = self.create_publisher(Int32, '/slam/tracking_state', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Initialize SLAM
        if ORBSLAM3_AVAILABLE:
            self.init_orbslam3()
        else:
            self.get_logger().warn('ORB-SLAM3 not available - running in stub mode')
            self.get_logger().info('Install ORB-SLAM3 ROS2 wrapper for full functionality')
        
        # Timer for map publishing (low frequency)
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('SLAM Node initialized')
    
    def init_orbslam3(self):
        """Initialize ORB-SLAM3"""
        if not self.vocabulary_file or not self.settings_file:
            self.get_logger().error('Vocabulary and settings files required for ORB-SLAM3')
            return
        
        try:
            # self.slam = ORBSLAM3(self.vocabulary_file, self.settings_file, sensor_type='monocular')
            self.slam_initialized = True
            self.get_logger().info('ORB-SLAM3 initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ORB-SLAM3: {e}')
    
    def image_callback(self, msg: Image):
        """Process incoming camera image"""
        if ORBSLAM3_AVAILABLE and self.slam_initialized:
            self.process_image_orbslam3(msg)
        else:
            # Stub mode - simulate some output for testing
            self.process_image_stub(msg)
    
    def process_image_orbslam3(self, msg: Image):
        """Process image with ORB-SLAM3"""
        # Convert ROS image to OpenCV format
        # pose = self.slam.process_image(image, msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        # 
        # if pose is not None:
        #     self.current_pose = pose
        #     self.tracking_state = self.slam.get_tracking_state()
        #     self.publish_pose(msg.header.stamp)
        pass
    
    def process_image_stub(self, msg: Image):
        """Stub processing for development/testing"""
        # Simulate successful tracking
        self.tracking_state = 2  # Good tracking
        
        # Publish tracking state
        state_msg = Int32()
        state_msg.data = self.tracking_state
        self.state_pub.publish(state_msg)
        
        # Publish a static pose (for testing)
        self.publish_pose(msg.header.stamp)
    
    def publish_pose(self, stamp):
        """Publish current pose and odometry"""
        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.odom_frame
        
        # Use current pose or identity
        if self.current_pose is not None:
            # Extract pose from SLAM
            pass
        else:
            # Identity pose for stub mode
            pose_msg.pose.position.x = 0.0
            pose_msg.pose.position.y = 0.0
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
        
        self.pose_pub.publish(pose_msg)
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose = pose_msg.pose
        
        # Covariance (higher = less certain)
        variance = 0.1
        odom_msg.pose.covariance[0] = variance  # x
        odom_msg.pose.covariance[7] = variance  # y
        odom_msg.pose.covariance[14] = variance  # z
        odom_msg.pose.covariance[21] = variance  # roll
        odom_msg.pose.covariance[28] = variance  # pitch
        odom_msg.pose.covariance[35] = variance  # yaw
        
        self.odom_pub.publish(odom_msg)
        
        # Broadcast TF if enabled
        if self.publish_tf:
            self.broadcast_tf(odom_msg)
    
    def broadcast_tf(self, odom: Odometry):
        """Broadcast TF transform"""
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_map(self):
        """Publish map points as PointCloud2"""
        if ORBSLAM3_AVAILABLE and self.slam_initialized:
            # Get map points from ORB-SLAM3
            # self.map_points = self.slam.get_map_points()
            pass
        else:
            # Stub mode - publish empty cloud
            pass
        
        # Create PointCloud2 message
        if len(self.map_points) == 0:
            return
        
        cloud_msg = self.create_point_cloud(self.map_points)
        self.map_pub.publish(cloud_msg)
    
    def create_point_cloud(self, points):
        """Create PointCloud2 message from numpy array"""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack points into bytes
        data = []
        for p in points:
            data.append(struct.pack('fff', p[0], p[1], p[2]))
        msg.data = b''.join(data)
        
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = SLAMNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
