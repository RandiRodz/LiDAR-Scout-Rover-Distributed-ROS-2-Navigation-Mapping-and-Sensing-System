# LiDAR-Scout-Rover-Distributed-ROS-2-Navigation-Mapping-and-Sensing-System

Software repository for a distributed ROS 2 rover system built around 3D LiDAR, gas sensing, and autonomous navigation.

This repository focuses on the rover software stack only:
- Arduino Mega motor and gas sensor code
- Raspberry Pi 5 bridge and sensor-side scripts
- ROS 2 Jazzy workstation navigation, mapping, and visualization stack

The website/dashboard was moved to a separate repository and is intentionally not covered here.

## Overview

LiDAR Scout Rover is a multi-computer robotics system designed to:
- drive a tank-style rover with ROS 2 velocity commands
- estimate motion with LiDAR-inertial odometry
- build a live map of the environment
- avoid obstacles while navigating to goals
- publish gas sensor readings from the embedded layer

The software is split across three compute layers:
- `Arduino Mega`
  low-level motor control and gas sensor sampling
- `Raspberry Pi 5`
  LiDAR publishing, IMU publishing, serial bridge to the Arduino
- `Workstation / Intel NUC / laptop`
  Point-LIO, Nav2, obstacle processing, RViz, and the main autonomy stack

## Current Working Version

The current working version in this repository is **Version 3**.

Version 3 uses:
- native Point-LIO local odometry behavior
- `slam_toolbox` map correction during live operation
- a 360-degree point-cloud obstacle pipeline for Nav2
- tank-drive tuned Nav2 control parameters

This is the version that tested best as a complete software path in the rover project.

## Software Architecture

### Embedded layer
- Arduino receives motor commands over serial
- Arduino controls two DC motors through the motor driver shield
- Arduino reads gas sensor raw values and sends them back to the Pi

### Robot-side compute layer
- Pi 5 runs the Unitree LiDAR driver
- Pi publishes:
  - `/unilidar/cloud`
  - `/unilidar/imu`
- Pi bridges `/cmd_vel` to the Arduino motor serial protocol

### Workstation autonomy layer
- Point-LIO provides the local motion estimate
- custom bridges normalize odometry and TF for Nav2
- point-cloud processing nodes generate:
  - filtered obstacle views
  - a 360 obstacle cloud for Nav2
  - safety / proximity responses
- Nav2 performs path planning and local control
- RViz is used for live visualization and operator interaction

## Main ROS 2 Components

- `ROS 2 Jazzy`
- `Point-LIO`
- `Nav2`
- `slam_toolbox`
- `pointcloud_to_laserscan`
- `PCL`
- `RViz2`

## Repository Layout

- `arduino/`
  Arduino Mega motor and gas sensor code
- `pi5/`
  Raspberry Pi bridge and robot-side scripts
- `point_cloud_processing/`
  point cloud processing package history and references
- `lsr_description_v1/`
  robot description, URDF/Xacro, and visualization assets
- `docs/`
  diagrams, defense notes, tuning guides, papers, and architecture writeups
- `staging/`
  workspace staging area used while developing and patching V2/V3

## Main V3 Files

The Version 3 stack is centered around these files:

- `staging/ws_lsr_nav2_v2/src/lsr_nav2/launch/rover_nav2_v3.launch.py`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/config/nav2_v3_params.yaml`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/config/slam_toolbox_v3_online_async.yaml`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/rviz/rover_nav2_v3_view.rviz`

Important supporting files:

- `staging/ws_lsr_nav2_v2/src/lsr_nav2/src/nav2_obstacle_cloud.cpp`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/src/advanced_proximity_guard.cpp`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/src/cmd_vel_mux.cpp`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/src/odom_retimestamp_bridge.cpp`
- `staging/ws_lsr_nav2_v2/src/lsr_nav2/src/lidar_retimestamp_bridge.cpp`

## How To Run The Current Working V3 Stack

### 1. Start the LiDAR driver on the Pi

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### 2. Start the workstation autonomy stack

```bash
source /opt/ros/jazzy/setup.bash
source /home/ros2/ws_pointlio/install/setup.bash
source /home/ros2/ws_lsr_nav2_v2/install/setup.bash
ros2 launch point_cloud_processing rover_nav2_v3.launch.py
```

### 3. Optional keyboard teleop

```bash
source /opt/ros/jazzy/setup.bash
source /home/ros2/ws_lsr_nav2_v2/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

## What V3 Improves

Compared with earlier rover software branches, V3 improves:
- obstacle awareness around the full rover body
- long-run map consistency
- local control tuning for a tank-drive platform
- separation between raw LiDAR data, filtered obstacle data, and navigation control

## Documentation

Detailed project documentation is kept under `docs/`.

Useful starting points:

- `docs/presentation/v3_nav2_code_and_tuning_guide.txt`
- `docs/repository_handoff/development_process_detailed.txt`
- `docs/repository_handoff/code_explanation_broad.txt`
- `docs/presentation/rover_nav2_v3_flow.png`
- `docs/presentation/rover_nav2_all_versions_flow.png`

## Current Status

This repository reflects the rover software as developed and tested during the project.

The main working autonomy path is the live V3 navigation and mapping configuration. Some experimental follow-on work, such as saved-map localization refinements, is still under development and should be treated as secondary to the main V3 flow above.

## Author

Randy Rodriguez  
LiDAR Scout Rover software architecture, ROS 2 integration, navigation, mapping, sensing, and system bringup

## License

Distributed under the GNU-GPL License. See `LICENSE` for more information.
