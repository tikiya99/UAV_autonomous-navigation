#!/usr/bin/env python3
"""
Keyboard Teleoperation Node for UAV Control

Provides keyboard-based control for manual UAV flying via ROS2.
Publishes velocity commands and control signals for arming, takeoff, landing.

Key Mappings:
    W/S: Forward/Backward
    A/D: Left/Right
    I/K: Up/Down (Altitude)
    J/L: Yaw Left/Right
    SPACE: Toggle Arm/Disarm
    T: Takeoff
    G: Land
    ESC/Q: Emergency Stop / Quit
"""

import sys
import tty
import termios
import select
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


# Key bindings
MOVE_BINDINGS = {
    'w': (1.0, 0.0, 0.0, 0.0),   # Forward
    's': (-1.0, 0.0, 0.0, 0.0),  # Backward
    'a': (0.0, 1.0, 0.0, 0.0),   # Left
    'd': (0.0, -1.0, 0.0, 0.0),  # Right
    'i': (0.0, 0.0, 1.0, 0.0),   # Up
    'k': (0.0, 0.0, -1.0, 0.0),  # Down
    'j': (0.0, 0.0, 0.0, 1.0),   # Yaw Left
    'l': (0.0, 0.0, 0.0, -1.0),  # Yaw Right
    # Diagonal combinations
    'q': (1.0, 1.0, 0.0, 0.0),   # Forward-Left
    'e': (1.0, -1.0, 0.0, 0.0),  # Forward-Right
    'z': (-1.0, 1.0, 0.0, 0.0),  # Backward-Left
    'c': (-1.0, -1.0, 0.0, 0.0), # Backward-Right
}

SPEED_BINDINGS = {
    '1': 0.1,
    '2': 0.2,
    '3': 0.3,
    '4': 0.4,
    '5': 0.5,
    '6': 0.6,
    '7': 0.7,
    '8': 0.8,
    '9': 0.9,
    '0': 1.0,
}

HELP_MSG = """
╔═══════════════════════════════════════════════════════════════╗
║             UAV Keyboard Teleoperation Control                 ║
╠═══════════════════════════════════════════════════════════════╣
║  Movement:                    Altitude & Yaw:                  ║
║       W                            I (Up)                      ║
║     A S D                        J   K  L                      ║
║   (Left/Back/Right)          (Yaw Left/Down/Yaw Right)        ║
║                                                                ║
║  Diagonal: Q (FL), E (FR), Z (BL), C (BR)                     ║
║                                                                ║
║  Speed Control: 1-0 keys (0.1 to 1.0 multiplier)              ║
║                                                                ║
║  Commands:                                                     ║
║    SPACE : Toggle Arm/Disarm                                   ║
║    T     : Takeoff                                             ║
║    G     : Land                                                ║
║    H     : Hover (stop all movement)                           ║
║    ESC/X : Emergency Stop (disarm + exit offboard)             ║
║                                                                ║
║  Current Speed: {speed:.1f} m/s | Yaw Rate: {yaw_rate:.1f} rad/s          ║
╚═══════════════════════════════════════════════════════════════╝
"""


class TeleopKeyboardNode(Node):
    """
    ROS2 Node for keyboard teleoperation of UAV
    """
    
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('vertical_speed', 0.3)
        self.declare_parameter('yaw_rate', 0.3)
        self.declare_parameter('publish_rate', 20.0)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.vertical_speed = self.get_parameter('vertical_speed').value
        self.yaw_rate = self.get_parameter('yaw_rate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Speed multiplier
        self.speed_multiplier = 1.0
        
        # Current velocity state
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        
        # Armed state
        self.is_armed = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(Bool, '/uav/arm', 10)
        self.takeoff_pub = self.create_publisher(Bool, '/uav/takeoff', 10)
        self.land_pub = self.create_publisher(Bool, '/uav/land', 10)
        
        # Status subscriber
        self.status_sub = self.create_subscription(
            String,
            '/uav/status',
            self.status_callback,
            10
        )
        
        self.current_status = "Waiting for status..."
        
        # Timer for velocity publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        
        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Running flag
        self.running = True
        
        self.get_logger().info('Teleop Keyboard Node initialized')
    
    def status_callback(self, msg):
        """Update current status from UAV"""
        self.current_status = msg.data
    
    def get_key(self, timeout=0.1):
        """Get keyboard input with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def print_status(self):
        """Print current status to terminal"""
        # Clear screen and move cursor to top
        print('\033[2J\033[H', end='')
        print(HELP_MSG.format(
            speed=self.linear_speed * self.speed_multiplier,
            yaw_rate=self.yaw_rate * self.speed_multiplier
        ))
        print(f"\n  Status: {self.current_status}")
        print(f"  Velocity: x={self.vx:.2f}, y={self.vy:.2f}, z={self.vz:.2f}, yaw={self.vyaw:.2f}")
        print(f"  Armed: {'YES' if self.is_armed else 'NO'}")
        print("\n  Press 'x' or ESC to quit")
    
    def publish_velocity(self):
        """Publish current velocity command"""
        msg = Twist()
        msg.linear.x = self.vx * self.linear_speed * self.speed_multiplier
        msg.linear.y = self.vy * self.linear_speed * self.speed_multiplier
        msg.linear.z = self.vz * self.vertical_speed * self.speed_multiplier
        msg.angular.z = self.vyaw * self.yaw_rate * self.speed_multiplier
        
        self.cmd_vel_pub.publish(msg)
    
    def toggle_arm(self):
        """Toggle arm/disarm state"""
        self.is_armed = not self.is_armed
        msg = Bool()
        msg.data = self.is_armed
        self.arm_pub.publish(msg)
        self.get_logger().info(f"{'Arming' if self.is_armed else 'Disarming'}...")
    
    def takeoff(self):
        """Send takeoff command"""
        msg = Bool()
        msg.data = True
        self.takeoff_pub.publish(msg)
        self.get_logger().info("Takeoff command sent")
    
    def land(self):
        """Send land command"""
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
        self.get_logger().info("Land command sent")
    
    def emergency_stop(self):
        """Emergency stop - zero velocity and disarm"""
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        self.is_armed = False
        
        # Send zero velocity
        self.publish_velocity()
        
        # Disarm
        msg = Bool()
        msg.data = False
        self.arm_pub.publish(msg)
        
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")
    
    def hover(self):
        """Stop all movement (hover)"""
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vyaw = 0.0
        self.get_logger().info("Hover mode - all velocities zeroed")
    
    def run(self):
        """Main loop for keyboard input handling"""
        self.print_status()
        
        try:
            while self.running and rclpy.ok():
                key = self.get_key()
                
                if key == '':
                    # No input - decay velocities towards zero
                    self.vx *= 0.8
                    self.vy *= 0.8
                    self.vz *= 0.8
                    self.vyaw *= 0.8
                    
                    # Zero out small values
                    if abs(self.vx) < 0.01:
                        self.vx = 0.0
                    if abs(self.vy) < 0.01:
                        self.vy = 0.0
                    if abs(self.vz) < 0.01:
                        self.vz = 0.0
                    if abs(self.vyaw) < 0.01:
                        self.vyaw = 0.0
                        
                elif key.lower() in MOVE_BINDINGS:
                    binding = MOVE_BINDINGS[key.lower()]
                    self.vx = binding[0]
                    self.vy = binding[1]
                    self.vz = binding[2]
                    self.vyaw = binding[3]
                    
                elif key in SPEED_BINDINGS:
                    self.speed_multiplier = SPEED_BINDINGS[key]
                    
                elif key == ' ':  # Spacebar
                    self.toggle_arm()
                    
                elif key.lower() == 't':
                    self.takeoff()
                    
                elif key.lower() == 'g':
                    self.land()
                    
                elif key.lower() == 'h':
                    self.hover()
                    
                elif key == '\x1b' or key.lower() == 'x':  # ESC or X
                    self.emergency_stop()
                    self.running = False
                    
                self.print_status()
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
            # Final emergency stop
            self.emergency_stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()
    
    # Create a thread for ROS2 spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
