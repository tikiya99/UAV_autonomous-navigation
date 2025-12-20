# UAV Navigation System - Detailed Guide

This guide provides comprehensive documentation for setting up, configuring, and operating the UAV Navigation System.

## Table of Contents

1. [System Setup](#system-setup)
2. [Hardware Configuration](#hardware-configuration)
3. [Camera Calibration](#camera-calibration)
4. [PX4 Configuration](#px4-configuration)
5. [Teleop Operation](#teleop-operation)
6. [SLAM Configuration](#slam-configuration)
7. [Autonomous Navigation](#autonomous-navigation)
8. [Troubleshooting](#troubleshooting)
9. [Safety Procedures](#safety-procedures)

---

## System Setup

### Initial Environment Setup

After cloning the repository, set up your environment:

```bash
# Navigate to workspace
cd ~/Documents/Code/UAV\ Navigation/uav_navigation_ws

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc for persistent sourcing
echo "source ~/Documents/Code/UAV\ Navigation/uav_navigation_ws/install/setup.bash" >> ~/.bashrc
```

### Verify Installation

```bash
# Check all packages are built
ros2 pkg list | grep uav_

# Expected output:
# uav_bringup
# uav_camera
# uav_description
# uav_localization
# uav_mission
# uav_navigation
# uav_px4_comm
# uav_slam
# uav_teleop
```

---

## Hardware Configuration

### Companion Computer to CUAV V5+ Connection

#### Serial Connection (Recommended)
Connect the companion computer to CUAV V5+ via TELEM2 port:

| CUAV V5+ TELEM2 | Companion Computer |
|-----------------|-------------------|
| TX | RX (GPIO 15) |
| RX | TX (GPIO 14) |
| GND | GND |

> [!WARNING]
> Ensure voltage levels match! CUAV V5+ uses 3.3V logic.

#### USB Connection (Alternative)
Use a USB-to-Serial adapter:
```bash
# Check connected device
ls /dev/ttyUSB*
# or
ls /dev/ttyACM*
```

### Configure Serial Port

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Set permissions (if needed)
sudo chmod 666 /dev/ttyUSB0
```

---

## Camera Calibration

### Capture Calibration Images

1. Print a checkerboard pattern (9x6 internal corners, 25mm squares)
2. Run calibration capture:

```bash
ros2 run uav_camera calibration_capture --ros-args \
  -p image_count:=30 \
  -p checkerboard_size:="9x6" \
  -p square_size:=0.025
```

### Perform Calibration

```bash
ros2 run camera_calibration cameracalibrator \
  --size 9x6 \
  --square 0.025 \
  image:=/camera/image_raw
```

### Save Calibration

The calibration file will be saved to:
```
uav_navigation_ws/src/uav_camera/config/camera_calibration.yaml
```

---

## PX4 Configuration

### QGroundControl Parameters

Set the following parameters in QGroundControl:

| Parameter | Value | Description |
|-----------|-------|-------------|
| `UXRCE_DDS_CFG` | TELEM2 | Enable DDS on TELEM2 |
| `SER_TEL2_BAUD` | 921600 | Serial baud rate |
| `COM_RC_IN_MODE` | Joystick | Allow offboard without RC |
| `COM_ARM_WO_GPS` | Allow | For indoor testing |

### Start Micro XRCE-DDS Agent

```bash
# For serial connection
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

# For UDP (simulation)
MicroXRCEAgent udp4 -p 8888
```

### Verify Connection

```bash
# Check for PX4 topics
ros2 topic list | grep fmu

# Should see topics like:
# /fmu/out/vehicle_status
# /fmu/out/vehicle_local_position
# /fmu/in/offboard_control_mode
```

---

## Teleop Operation

### Starting Teleop Mode

```bash
# Terminal 1: DDS Agent
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600

# Terminal 2: Teleop Launch
ros2 launch uav_bringup uav_teleop_only.launch.py
```

### Control Reference

| Key | Action | Value |
|-----|--------|-------|
| W | Move Forward | +0.5 m/s |
| S | Move Backward | -0.5 m/s |
| A | Move Left | +0.5 m/s |
| D | Move Right | -0.5 m/s |
| I | Ascend | +0.3 m/s |
| K | Descend | -0.3 m/s |
| J | Yaw Left | +0.3 rad/s |
| L | Yaw Right | -0.3 rad/s |
| SPACE | Arm/Disarm | Toggle |
| T | Takeoff | 2m altitude |
| G | Land | Descend to ground |
| ESC | Emergency Stop | Return to Manual |

### Adjusting Velocity Limits

Edit `uav_navigation_ws/src/uav_teleop/config/teleop_params.yaml`:

```yaml
teleop_node:
  ros__parameters:
    max_linear_velocity: 1.0    # m/s
    max_vertical_velocity: 0.5  # m/s
    max_yaw_rate: 0.5          # rad/s
    velocity_step: 0.1         # m/s increment
```

---

## SLAM Configuration

### ORB-SLAM3 Setup

Ensure ORB-SLAM3 vocabulary file is in place:
```bash
ls src/uav_slam/config/ORBvoc.txt
```

### Running SLAM

```bash
# Launch camera and SLAM
ros2 launch uav_slam slam.launch.py

# View map in RViz2
ros2 launch uav_description display.launch.py
```

### SLAM Parameters

Edit `uav_navigation_ws/src/uav_slam/config/orb_slam3_mono.yaml`:

```yaml
# Camera Parameters (from calibration)
Camera.fx: 500.0
Camera.fy: 500.0
Camera.cx: 320.0
Camera.cy: 240.0

# Distortion coefficients
Camera.k1: 0.0
Camera.k2: 0.0
Camera.p1: 0.0
Camera.p2: 0.0

# ORB Parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
```

---

## Autonomous Navigation

### GPS Waypoint Navigation

1. Create a mission file:

```yaml
# missions/sample_mission.yaml
waypoints:
  - name: "Takeoff"
    latitude: 7.2906
    longitude: 80.6337
    altitude: 5.0
    action: takeoff
    
  - name: "Waypoint 1"
    latitude: 7.2908
    longitude: 80.6340
    altitude: 10.0
    action: navigate
    
  - name: "Return Home"
    latitude: 7.2906
    longitude: 80.6337
    altitude: 5.0
    action: land
```

2. Launch autonomous mode:

```bash
ros2 launch uav_bringup uav_autonomous.launch.py \
  mission_file:=missions/sample_mission.yaml
```

### Nav2 Configuration

Key parameters in `uav_navigation_ws/src/uav_navigation/config/nav2_params.yaml`:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      rolling_window: true
      width: 100
      height: 100
      resolution: 1.0

controller_server:
  ros__parameters:
    controller_frequency: 10.0
```

---

## Troubleshooting

### Common Issues

#### 1. No PX4 Topics Visible

```bash
# Check if DDS agent is running
ps aux | grep MicroXRCE

# Check serial connection
dmesg | tail -20

# Verify PX4 DDS configuration
# In QGC: Parameters > UXRCE_DDS_CFG should be set
```

#### 2. Camera Not Working

```bash
# List video devices
ls /dev/video*

# Test camera
ros2 run usb_cam usb_cam_node_exe

# Check permissions
sudo chmod 666 /dev/video0
```

#### 3. SLAM Not Tracking

- Ensure good lighting
- Check camera calibration
- Increase `ORBextractor.nFeatures`
- Verify camera topic is publishing

#### 4. Build Errors

```bash
# Clean and rebuild
cd uav_navigation_ws
rm -rf build install log
colcon build --symlink-install
```

### Log Analysis

```bash
# View node logs
ros2 run rqt_console rqt_console

# Check specific node
ros2 run uav_px4_comm offboard_control_node --ros-args --log-level debug
```

---

## Safety Procedures

### Pre-Flight Checklist

- [ ] Battery fully charged and secure
- [ ] All propellers properly mounted
- [ ] RC controller connected and failsafe tested
- [ ] GPS fix acquired (outdoor)
- [ ] Clear flight area
- [ ] Companion computer connected
- [ ] DDS communication verified
- [ ] Teleop control tested

### Emergency Procedures

1. **Loss of Control**: Flip RC switch to MANUAL mode
2. **Communication Loss**: PX4 will trigger RTL (Return to Launch)
3. **Low Battery**: PX4 will auto-land when critical
4. **Crash**: Immediately disarm via RC or ESC key

### Failsafe Configuration

Configure in QGroundControl:

| Failsafe | Action |
|----------|--------|
| RC Loss | Return to Launch |
| Data Link Loss | Land |
| Low Battery | Return to Launch |
| Geofence Breach | Land |

---

## Appendix

### ROS2 Launch Arguments

```bash
# uav_teleop_only.launch.py
ros2 launch uav_bringup uav_teleop_only.launch.py \
  use_sim:=false \
  serial_port:=/dev/ttyUSB0 \
  baudrate:=921600

# uav_autonomous.launch.py
ros2 launch uav_bringup uav_autonomous.launch.py \
  use_sim:=false \
  enable_slam:=true \
  enable_gps:=true \
  mission_file:=missions/sample.yaml
```

### Useful ROS2 Commands

```bash
# Monitor topics
ros2 topic echo /fmu/out/vehicle_status

# Check TF tree
ros2 run tf2_tools view_frames

# Node info
ros2 node info /offboard_control_node

# Record bag
ros2 bag record -a
```
