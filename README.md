# UAV Navigation System

A comprehensive ROS2-based autonomous quadcopter navigation system for DJI F450 frame with CUAV V5+ flight controller running PX4 firmware.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![PX4](https://img.shields.io/badge/PX4-v1.14+-orange)
![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-purple)

## 🚁 Overview

This project implements a complete autonomous UAV navigation stack featuring:

- **PX4-ROS2 Communication** via Micro XRCE-DDS
- **Monocular Visual SLAM** using ORB-SLAM3
- **Sensor Fusion** with GPS, Visual Odometry, and IMU
- **Autonomous Navigation** using Nav2 stack
- **Keyboard Teleoperation** for manual control

## 📋 Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Package Description](#package-description)
- [Usage](#usage)
- [Architecture](#architecture)
- [Contributing](#contributing)

## 🔧 Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Frame | DJI F450 (450mm diagonal, X-config) |
| Flight Controller | CUAV V5+ with PX4 firmware |
| Motors | A2212 1000kV (x4) |
| Propellers | 1045 (10" x 4.5 pitch) |
| ESCs | 30A (x4) |
| Companion Computer | Raspberry Pi 4/5 or Ubuntu Laptop |
| Camera | USB Monocular Camera |
| GPS | Compatible GPS module |
| Battery | 3S/4S LiPo |

## 💻 Software Dependencies

### Operating System
- Ubuntu 22.04 LTS

### ROS2 & PX4
- ROS2 Humble Hawksbill
- PX4 Autopilot v1.14+
- Micro XRCE-DDS Agent
- px4_msgs

### Navigation & SLAM
- Nav2 (Navigation2 Stack)
- ORB-SLAM3
- robot_localization

### Additional
- OpenCV 4.x
- Eigen3
- Pangolin (for ORB-SLAM3)

## 📦 Installation

### 1. Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Install PX4 Dependencies

```bash
# Clone PX4-Autopilot (for message definitions)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
```

### 3. Install Micro XRCE-DDS Agent

```bash
# Install from source
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 4. Install Nav2 and Dependencies

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
sudo apt install ros-humble-robot-localization -y
sudo apt install ros-humble-usb-cam -y
sudo apt install ros-humble-cv-bridge -y
```

### 5. Clone and Build This Workspace

```bash
cd ~/Documents/Code/UAV\ Navigation
cd uav_navigation_ws

# Clone px4_msgs
cd src
git clone https://github.com/PX4/px4_msgs.git

# Build
cd ..
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Quick Start

### Teleop Mode (Manual Control)

```bash
# Terminal 1: Start Micro XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: Launch teleop
cd ~/Documents/Code/UAV\ Navigation/uav_navigation_ws
source install/setup.bash
ros2 launch uav_bringup uav_teleop_only.launch.py
```

### Autonomous Mode

```bash
# Terminal 1: Start Micro XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 2: Launch full system
cd ~/Documents/Code/UAV\ Navigation/uav_navigation_ws
source install/setup.bash
ros2 launch uav_bringup uav_autonomous.launch.py
```

## 📁 Package Description

| Package | Description |
|---------|-------------|
| `uav_description` | URDF/Xacro robot model for DJI F450 |
| `uav_px4_comm` | PX4 communication via Micro XRCE-DDS |
| `uav_teleop` | Keyboard teleoperation control |
| `uav_camera` | Camera driver and calibration |
| `uav_slam` | ORB-SLAM3 integration |
| `uav_localization` | Sensor fusion (GPS + Visual + IMU) |
| `uav_navigation` | Nav2 configuration for UAV |
| `uav_mission` | Mission planning and execution |
| `uav_bringup` | Launch files and configurations |

## 🎮 Usage

### Keyboard Controls

```
        W                      I
   A    S    D            J    K    L
   
W/S: Forward/Backward      I/K: Altitude Up/Down
A/D: Left/Right            J/L: Yaw Left/Right

SPACE: Arm/Disarm
ESC: Emergency Stop
T: Takeoff
G: Land
```

### ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Velocity commands |
| `/odom` | `Odometry` | Fused odometry |
| `/camera/image_raw` | `Image` | Camera feed |
| `/slam/pose` | `PoseStamped` | SLAM pose estimate |
| `/gps/fix` | `NavSatFix` | GPS data |

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Companion Computer                        │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐   │
│  │  Camera  │───▶│ ORB-SLAM3│───▶│  Robot Localization  │   │
│  └──────────┘    └──────────┘    │    (EKF Fusion)      │   │
│                                   └──────────┬───────────┘   │
│  ┌──────────┐                                │              │
│  │   GPS    │────────────────────────────────┘              │
│  └──────────┘                                │              │
│                                              ▼              │
│  ┌──────────┐    ┌──────────┐    ┌──────────────────────┐   │
│  │  Teleop  │───▶│ Cmd Mux  │◀───│      Nav2 Stack      │   │
│  └──────────┘    └────┬─────┘    └──────────────────────┘   │
│                       │                                      │
│                       ▼                                      │
│              ┌────────────────┐                             │
│              │ Offboard Node  │                             │
│              └───────┬────────┘                             │
│                      │ Micro XRCE-DDS                       │
└──────────────────────┼──────────────────────────────────────┘
                       │
┌──────────────────────┼──────────────────────────────────────┐
│                      ▼                                       │
│              ┌────────────────┐                             │
│              │  CUAV V5+ PX4  │                             │
│              └───────┬────────┘                             │
│                      │                                       │
│    ┌─────────┬───────┴───────┬─────────┐                   │
│    ▼         ▼               ▼         ▼                   │
│  Motor 1   Motor 2       Motor 3   Motor 4                 │
└─────────────────────────────────────────────────────────────┘
```

## 📖 Documentation

For detailed documentation, see [GUIDE.md](GUIDE.md).

## ⚠️ Safety

> [!CAUTION]
> **Always test in simulation first before real flight!**
> - Keep a manual RC controller ready for override
> - Start with indoor/tethered testing
> - Follow local drone regulations
> - Never fly over people or restricted areas

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🤝 Contributing

Contributions are welcome! Please read the contributing guidelines before submitting a pull request.
