#!/usr/bin/env python3
"""
Vehicle State Publisher Node

Publishes aggregated vehicle state information from PX4 to ROS2 topics
for consumption by other nodes (navigation, visualization, logging).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
import math

try:
    from px4_msgs.msg import (
        VehicleLocalPosition,
        VehicleGlobalPosition,
        VehicleAttitude,
        VehicleAngularVelocity,
        SensorCombined,
        VehicleStatus
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False


class VehicleStateNode(Node):
    """
    Converts PX4 vehicle state messages to standard ROS2 messages.
    
    Subscribes to PX4 topics:
        /fmu/out/vehicle_local_position
        /fmu/out/vehicle_global_position  
        /fmu/out/vehicle_attitude
        /fmu/out/sensor_combined
        
    Publishes standard ROS2 topics:
        /odom (nav_msgs/Odometry)
        /pose (geometry_msgs/PoseStamped)
        /imu (sensor_msgs/Imu)
        /gps/fix (sensor_msgs/NavSatFix)
        /altitude (std_msgs/Float32)
        
    Broadcasts TF:
        odom -> base_link
    """
    
    def __init__(self):
        super().__init__('vehicle_state_node')
        
        # Parameters
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        
        # QoS for PX4
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # State storage
        self.local_position = None
        self.attitude = None
        self.angular_velocity = None
        
        # TF Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.altitude_pub = self.create_publisher(Float32, '/altitude', 10)
        
        if PX4_MSGS_AVAILABLE:
            # PX4 Subscribers
            self.local_pos_sub = self.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position',
                self.local_position_callback,
                px4_qos
            )
            
            self.global_pos_sub = self.create_subscription(
                VehicleGlobalPosition,
                '/fmu/out/vehicle_global_position',
                self.global_position_callback,
                px4_qos
            )
            
            self.attitude_sub = self.create_subscription(
                VehicleAttitude,
                '/fmu/out/vehicle_attitude',
                self.attitude_callback,
                px4_qos
            )
            
            self.angular_vel_sub = self.create_subscription(
                VehicleAngularVelocity,
                '/fmu/out/vehicle_angular_velocity',
                self.angular_velocity_callback,
                px4_qos
            )
            
            self.sensor_sub = self.create_subscription(
                SensorCombined,
                '/fmu/out/sensor_combined',
                self.sensor_callback,
                px4_qos
            )
        
        self.get_logger().info('Vehicle State Node initialized')
        
        if not PX4_MSGS_AVAILABLE:
            self.get_logger().warn('px4_msgs not available - running in stub mode')
    
    def local_position_callback(self, msg):
        """Process local position and publish odometry"""
        self.local_position = msg
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position (convert NED to ENU for ROS)
        odom.pose.pose.position.x = msg.y   # East -> X
        odom.pose.pose.position.y = msg.x   # North -> Y  
        odom.pose.pose.position.z = -msg.z  # Down -> Up (Z)
        
        # Orientation from attitude if available
        if self.attitude:
            odom.pose.pose.orientation.w = self.attitude.q[0]
            odom.pose.pose.orientation.x = self.attitude.q[1]
            odom.pose.pose.orientation.y = self.attitude.q[2]
            odom.pose.pose.orientation.z = self.attitude.q[3]
        
        # Velocity (convert NED to ENU)
        odom.twist.twist.linear.x = msg.vy   # East velocity
        odom.twist.twist.linear.y = msg.vx   # North velocity
        odom.twist.twist.linear.z = -msg.vz  # Up velocity
        
        # Angular velocity
        if self.angular_velocity:
            odom.twist.twist.angular.x = self.angular_velocity.xyz[0]
            odom.twist.twist.angular.y = self.angular_velocity.xyz[1]
            odom.twist.twist.angular.z = self.angular_velocity.xyz[2]
        
        self.odom_pub.publish(odom)
        
        # Publish altitude
        alt_msg = Float32()
        alt_msg.data = -msg.z  # NED to altitude
        self.altitude_pub.publish(alt_msg)
        
        # Broadcast TF
        if self.publish_tf:
            self.broadcast_transform(odom)
    
    def global_position_callback(self, msg):
        """Process global position (GPS) and publish NavSatFix"""
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = 'gps_link'
        
        gps.latitude = msg.lat
        gps.longitude = msg.lon
        gps.altitude = msg.alt
        
        # Covariance (approximate)
        gps.position_covariance = [
            msg.eph ** 2, 0.0, 0.0,
            0.0, msg.eph ** 2, 0.0,
            0.0, 0.0, msg.epv ** 2
        ]
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_pub.publish(gps)
    
    def attitude_callback(self, msg):
        """Store attitude for use in odometry"""
        self.attitude = msg
        
        # Publish pose
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.odom_frame
        
        if self.local_position:
            pose.pose.position.x = self.local_position.y
            pose.pose.position.y = self.local_position.x
            pose.pose.position.z = -self.local_position.z
        
        pose.pose.orientation.w = msg.q[0]
        pose.pose.orientation.x = msg.q[1]
        pose.pose.orientation.y = msg.q[2]
        pose.pose.orientation.z = msg.q[3]
        
        self.pose_pub.publish(pose)
    
    def angular_velocity_callback(self, msg):
        """Store angular velocity for use in odometry"""
        self.angular_velocity = msg
    
    def sensor_callback(self, msg):
        """Process sensor data and publish IMU message"""
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'
        
        # Orientation from attitude
        if self.attitude:
            imu.orientation.w = self.attitude.q[0]
            imu.orientation.x = self.attitude.q[1]
            imu.orientation.y = self.attitude.q[2]
            imu.orientation.z = self.attitude.q[3]
        else:
            imu.orientation_covariance[0] = -1  # Unknown
        
        # Angular velocity (convert to ROS FLU)
        imu.angular_velocity.x = msg.gyro_rad[0]
        imu.angular_velocity.y = -msg.gyro_rad[1]
        imu.angular_velocity.z = -msg.gyro_rad[2]
        
        # Linear acceleration (convert to ROS FLU)
        imu.linear_acceleration.x = msg.accelerometer_m_s2[0]
        imu.linear_acceleration.y = -msg.accelerometer_m_s2[1]
        imu.linear_acceleration.z = -msg.accelerometer_m_s2[2]
        
        self.imu_pub.publish(imu)
    
    def broadcast_transform(self, odom: Odometry):
        """Broadcast TF transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        
        t.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = VehicleStateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
