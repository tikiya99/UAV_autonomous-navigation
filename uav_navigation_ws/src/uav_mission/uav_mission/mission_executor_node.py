#!/usr/bin/env python3
"""
Mission Executor Node

State machine for autonomous UAV mission execution.
Manages waypoint navigation, takeoff, landing, and safety monitoring.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import FollowWaypoints, NavigateToPose

import yaml
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional


class MissionState(Enum):
    """Mission execution states"""
    IDLE = 0
    PREFLIGHT_CHECK = 1
    ARMING = 2
    TAKEOFF = 3
    NAVIGATING = 4
    AT_WAYPOINT = 5
    RETURNING_HOME = 6
    LANDING = 7
    LANDED = 8
    ERROR = 9
    EMERGENCY = 10


@dataclass
class Waypoint:
    """GPS waypoint with metadata"""
    name: str
    latitude: float
    longitude: float
    altitude: float
    action: str = "navigate"  # navigate, takeoff, land, loiter


class MissionExecutorNode(Node):
    """
    ROS2 Node for autonomous mission execution
    
    Subscribes:
        /uav/status (String): Vehicle status
        /gps/fix (NavSatFix): Current GPS position
        
    Publishes:
        /uav/arm (Bool): Arm/disarm commands
        /uav/takeoff (Bool): Takeoff trigger
        /uav/land (Bool): Land trigger
        /mission/state (String): Current mission state
        
    Actions:
        Uses Nav2 FollowWaypoints or NavigateToPose for navigation
    """
    
    def __init__(self):
        super().__init__('mission_executor_node')
        
        # Parameters
        self.declare_parameter('mission_file', '')
        self.declare_parameter('auto_start', False)
        self.declare_parameter('home_altitude', 2.0)
        
        self.mission_file = self.get_parameter('mission_file').value
        self.auto_start = self.get_parameter('auto_start').value
        self.home_altitude = self.get_parameter('home_altitude').value
        
        # State
        self.state = MissionState.IDLE
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_idx = 0
        self.home_position: Optional[NavSatFix] = None
        self.is_armed = False
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Publishers
        self.arm_pub = self.create_publisher(Bool, '/uav/arm', 10)
        self.takeoff_pub = self.create_publisher(Bool, '/uav/takeoff', 10)
        self.land_pub = self.create_publisher(Bool, '/uav/land', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/uav/status',
            self.status_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action clients for Nav2
        self.navigate_action = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )
        
        # Mission control timer
        self.timer = self.create_timer(1.0, self.mission_tick, callback_group=self.callback_group)
        
        # Load mission if specified
        if self.mission_file:
            self.load_mission(self.mission_file)
        
        self.get_logger().info('Mission Executor Node initialized')
    
    def load_mission(self, filepath: str):
        """Load mission waypoints from YAML file"""
        try:
            with open(filepath, 'r') as f:
                mission_data = yaml.safe_load(f)
            
            self.waypoints = []
            for wp_data in mission_data.get('waypoints', []):
                wp = Waypoint(
                    name=wp_data.get('name', 'unnamed'),
                    latitude=wp_data['latitude'],
                    longitude=wp_data['longitude'],
                    altitude=wp_data.get('altitude', 2.0),
                    action=wp_data.get('action', 'navigate')
                )
                self.waypoints.append(wp)
            
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {filepath}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load mission: {e}')
    
    def status_callback(self, msg: String):
        """Update vehicle status"""
        self.is_armed = "ARMED" in msg.data
    
    def gps_callback(self, msg: NavSatFix):
        """Store current GPS for home position"""
        if self.home_position is None and msg.status.status >= 0:
            self.home_position = msg
            self.get_logger().info(f'Home position set: {msg.latitude:.6f}, {msg.longitude:.6f}')
    
    def mission_tick(self):
        """Main mission state machine tick"""
        # Publish current state
        state_msg = String()
        state_msg.data = f"{self.state.name}"
        if self.state == MissionState.NAVIGATING and self.current_waypoint_idx < len(self.waypoints):
            state_msg.data += f" -> {self.waypoints[self.current_waypoint_idx].name}"
        self.state_pub.publish(state_msg)
        
        # State machine
        if self.state == MissionState.IDLE:
            if self.auto_start and self.waypoints:
                self.transition_to(MissionState.PREFLIGHT_CHECK)
        
        elif self.state == MissionState.PREFLIGHT_CHECK:
            if self.home_position is not None:
                self.get_logger().info('Preflight check passed')
                self.transition_to(MissionState.ARMING)
            else:
                self.get_logger().warn('Waiting for GPS fix...')
        
        elif self.state == MissionState.ARMING:
            if not self.is_armed:
                self.arm()
            else:
                self.transition_to(MissionState.TAKEOFF)
        
        elif self.state == MissionState.TAKEOFF:
            if self.current_waypoint_idx < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint_idx]
                if wp.action == 'takeoff':
                    self.takeoff()
                    self.current_waypoint_idx += 1
                self.transition_to(MissionState.NAVIGATING)
        
        elif self.state == MissionState.NAVIGATING:
            # Navigation handled by action client callbacks
            pass
        
        elif self.state == MissionState.AT_WAYPOINT:
            # Process current waypoint action
            if self.current_waypoint_idx >= len(self.waypoints):
                self.transition_to(MissionState.RETURNING_HOME)
            else:
                wp = self.waypoints[self.current_waypoint_idx]
                if wp.action == 'land':
                    self.transition_to(MissionState.LANDING)
                else:
                    self.current_waypoint_idx += 1
                    self.navigate_to_next_waypoint()
        
        elif self.state == MissionState.RETURNING_HOME:
            self.transition_to(MissionState.LANDING)
        
        elif self.state == MissionState.LANDING:
            self.land()
            self.transition_to(MissionState.LANDED)
        
        elif self.state == MissionState.LANDED:
            self.disarm()
            self.transition_to(MissionState.IDLE)
    
    def transition_to(self, new_state: MissionState):
        """Transition to a new mission state"""
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
    
    def arm(self):
        """Send arm command"""
        msg = Bool()
        msg.data = True
        self.arm_pub.publish(msg)
    
    def disarm(self):
        """Send disarm command"""
        msg = Bool()
        msg.data = False
        self.arm_pub.publish(msg)
    
    def takeoff(self):
        """Send takeoff command"""
        msg = Bool()
        msg.data = True
        self.takeoff_pub.publish(msg)
    
    def land(self):
        """Send land command"""
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
    
    def navigate_to_next_waypoint(self):
        """Start navigation to next waypoint"""
        if self.current_waypoint_idx >= len(self.waypoints):
            return
        
        wp = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(f'Navigating to: {wp.name} ({wp.latitude}, {wp.longitude})')
        
        # Create goal pose (would need GPS to local conversion)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        # Note: In real implementation, convert GPS to local coordinates
        goal.pose.pose.position.x = 0.0  # Placeholder
        goal.pose.pose.position.y = 0.0  # Placeholder
        goal.pose.pose.position.z = wp.altitude
        goal.pose.pose.orientation.w = 1.0
        
        self.navigate_action.send_goal_async(
            goal,
            feedback_callback=self.navigation_feedback_callback
        ).add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future):
        """Handle navigation goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Navigation goal rejected')
            return
        
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        # Could update progress tracking here
        pass
    
    def navigation_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.get_logger().info(f'Navigation completed')
        self.transition_to(MissionState.AT_WAYPOINT)


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
