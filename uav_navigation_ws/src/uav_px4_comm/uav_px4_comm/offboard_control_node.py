#!/usr/bin/env python3
"""
Offboard Control Node for PX4 via ROS2 and Micro XRCE-DDS

This node provides the interface between ROS2 velocity commands and PX4's
offboard control mode. It handles:
- Velocity command conversion to PX4 TrajectorySetpoint
- OffboardControlMode publishing for maintaining offboard control
- Vehicle arming/disarming via VehicleCommand
- Flight mode switching
- Safety heartbeat and failsafe handling
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import math
import numpy as np

# PX4 messages - these will be available after px4_msgs is built
try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
        VehicleOdometry
    )
    PX4_MSGS_AVAILABLE = True
except ImportError:
    PX4_MSGS_AVAILABLE = False
    print("WARNING: px4_msgs not found. Using mock messages for development.")


class OffboardControlNode(Node):
    """
    ROS2 Node for PX4 Offboard Control via Micro XRCE-DDS
    
    Subscribes to:
        /cmd_vel (Twist): Velocity commands from teleop or navigation
        /uav/arm (Bool): Arm/disarm commands
        /uav/takeoff (Bool): Takeoff trigger
        /uav/land (Bool): Land trigger
        
    Publishes to:
        /fmu/in/offboard_control_mode (OffboardControlMode)
        /fmu/in/trajectory_setpoint (TrajectorySetpoint)
        /fmu/in/vehicle_command (VehicleCommand)
        /uav/status (String): Human-readable status
    """
    
    def __init__(self):
        super().__init__('offboard_control_node')
        
        # Parameters
        self.declare_parameter('offboard_setpoint_rate', 20.0)  # Hz
        self.declare_parameter('max_linear_velocity', 2.0)  # m/s
        self.declare_parameter('max_vertical_velocity', 1.0)  # m/s
        self.declare_parameter('max_yaw_rate', 1.0)  # rad/s
        self.declare_parameter('takeoff_altitude', 2.0)  # meters
        self.declare_parameter('heartbeat_timeout', 0.5)  # seconds
        
        self.offboard_rate = self.get_parameter('offboard_setpoint_rate').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_vertical_vel = self.get_parameter('max_vertical_velocity').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout').value
        
        # QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # State variables
        self.is_armed = False
        self.is_offboard = False
        self.vehicle_status = None
        self.local_position = None
        self.offboard_setpoint_counter = 0
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Target setpoints
        self.target_velocity = [0.0, 0.0, 0.0]  # vx, vy, vz (NED)
        self.target_yaw_rate = 0.0
        
        if PX4_MSGS_AVAILABLE:
            # PX4 Publishers
            self.offboard_control_mode_pub = self.create_publisher(
                OffboardControlMode,
                '/fmu/in/offboard_control_mode',
                qos_profile
            )
            
            self.trajectory_setpoint_pub = self.create_publisher(
                TrajectorySetpoint,
                '/fmu/in/trajectory_setpoint',
                qos_profile
            )
            
            self.vehicle_command_pub = self.create_publisher(
                VehicleCommand,
                '/fmu/in/vehicle_command',
                qos_profile
            )
            
            # PX4 Subscribers
            self.vehicle_status_sub = self.create_subscription(
                VehicleStatus,
                '/fmu/out/vehicle_status',
                self.vehicle_status_callback,
                qos_profile
            )
            
            self.local_position_sub = self.create_subscription(
                VehicleLocalPosition,
                '/fmu/out/vehicle_local_position',
                self.local_position_callback,
                qos_profile
            )
        
        # ROS2 Subscribers (always available)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.arm_sub = self.create_subscription(
            Bool,
            '/uav/arm',
            self.arm_callback,
            10
        )
        
        self.takeoff_sub = self.create_subscription(
            Bool,
            '/uav/takeoff',
            self.takeoff_callback,
            10
        )
        
        self.land_sub = self.create_subscription(
            Bool,
            '/uav/land',
            self.land_callback,
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/uav/status', 10)
        
        # Timer for offboard setpoint publishing
        timer_period = 1.0 / self.offboard_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Offboard Control Node initialized')
        self.get_logger().info(f'  Max linear velocity: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max vertical velocity: {self.max_vertical_vel} m/s')
        self.get_logger().info(f'  Takeoff altitude: {self.takeoff_altitude} m')
        
        if not PX4_MSGS_AVAILABLE:
            self.get_logger().warn('Running in development mode - px4_msgs not available')
    
    def vehicle_status_callback(self, msg):
        """Handle vehicle status updates from PX4"""
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.is_offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    
    def local_position_callback(self, msg):
        """Handle local position updates from PX4"""
        self.local_position = msg
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Handle velocity commands from teleop or navigation
        
        Input (ROS convention - FLU):
            linear.x: forward velocity
            linear.y: left velocity  
            linear.z: up velocity
            angular.z: yaw rate (CCW positive)
            
        Output (PX4 NED):
            vx: North (forward)
            vy: East (right, so negate)
            vz: Down (so negate)
            yaw_rate: CW positive (so negate)
        """
        self.last_cmd_vel_time = self.get_clock().now()
        
        # Clamp velocities
        vx = np.clip(msg.linear.x, -self.max_linear_vel, self.max_linear_vel)
        vy = np.clip(msg.linear.y, -self.max_linear_vel, self.max_linear_vel)
        vz = np.clip(msg.linear.z, -self.max_vertical_vel, self.max_vertical_vel)
        yaw_rate = np.clip(msg.angular.z, -self.max_yaw_rate, self.max_yaw_rate)
        
        # Convert FLU to NED
        self.target_velocity = [vx, -vy, -vz]
        self.target_yaw_rate = -yaw_rate
    
    def arm_callback(self, msg: Bool):
        """Handle arm/disarm commands"""
        if msg.data:
            self.arm()
        else:
            self.disarm()
    
    def takeoff_callback(self, msg: Bool):
        """Handle takeoff command"""
        if msg.data:
            self.takeoff()
    
    def land_callback(self, msg: Bool):
        """Handle land command"""
        if msg.data:
            self.land()
    
    def timer_callback(self):
        """Main control loop - publishes offboard setpoints at fixed rate"""
        
        # Check for command timeout (heartbeat)
        time_since_cmd = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if time_since_cmd > self.heartbeat_timeout:
            # No recent commands - hover in place
            self.target_velocity = [0.0, 0.0, 0.0]
            self.target_yaw_rate = 0.0
        
        if PX4_MSGS_AVAILABLE:
            # Publish offboard control mode
            self.publish_offboard_control_mode()
            
            # Publish trajectory setpoint
            self.publish_trajectory_setpoint()
            
            # Handle offboard mode entry
            if self.offboard_setpoint_counter == 10:
                # After streaming setpoints for a bit, engage offboard mode
                self.engage_offboard_mode()
            
            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1
        
        # Publish status
        self.publish_status()
    
    def publish_offboard_control_mode(self):
        """Publish offboard control mode message"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint with velocity commands"""
        msg = TrajectorySetpoint()
        
        # Set velocities (NED frame)
        msg.velocity[0] = self.target_velocity[0]  # North
        msg.velocity[1] = self.target_velocity[1]  # East
        msg.velocity[2] = self.target_velocity[2]  # Down
        
        # Set yaw rate
        msg.yawspeed = self.target_yaw_rate
        
        # Position set to NaN (not controlling position)
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        
        # Yaw set to NaN (controlling yaw rate instead)
        msg.yaw = float('nan')
        
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.trajectory_setpoint_pub.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command to PX4"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.vehicle_command_pub.publish(msg)
    
    def arm(self):
        """Send arm command to PX4"""
        self.get_logger().info('Sending ARM command')
        if PX4_MSGS_AVAILABLE:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0  # 1.0 = arm
            )
    
    def disarm(self):
        """Send disarm command to PX4"""
        self.get_logger().info('Sending DISARM command')
        if PX4_MSGS_AVAILABLE:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=0.0  # 0.0 = disarm
            )
    
    def engage_offboard_mode(self):
        """Switch to offboard flight mode"""
        self.get_logger().info('Engaging OFFBOARD mode')
        if PX4_MSGS_AVAILABLE:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,  # Custom mode
                param2=6.0   # Offboard mode
            )
    
    def takeoff(self):
        """Execute takeoff sequence"""
        self.get_logger().info(f'Initiating takeoff to {self.takeoff_altitude}m')
        # Reset setpoint counter to ensure offboard mode is engaged
        self.offboard_setpoint_counter = 0
        # Set upward velocity
        self.target_velocity = [0.0, 0.0, -0.5]  # Negative Z = up in NED
    
    def land(self):
        """Execute landing sequence"""
        self.get_logger().info('Initiating landing')
        if PX4_MSGS_AVAILABLE:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )
    
    def publish_status(self):
        """Publish human-readable status"""
        status_msg = String()
        armed_str = "ARMED" if self.is_armed else "DISARMED"
        mode_str = "OFFBOARD" if self.is_offboard else "MANUAL"
        
        if self.local_position:
            alt = -self.local_position.z  # Convert NED to altitude
            status_msg.data = f"{armed_str} | {mode_str} | Alt: {alt:.1f}m | Vel: [{self.target_velocity[0]:.1f}, {self.target_velocity[1]:.1f}, {self.target_velocity[2]:.1f}]"
        else:
            status_msg.data = f"{armed_str} | {mode_str} | No position data"
        
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
