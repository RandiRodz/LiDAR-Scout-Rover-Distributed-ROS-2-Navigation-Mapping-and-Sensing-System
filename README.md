# LiDAR-Scout-Rover-Distributed-ROS-2-Navigation-Mapping-and-Sensing-System

Software repository for a distributed ROS 2 rover system built around Unitree LiDAR L1, gas sensing, and autonomous navigation.
This repository focuses on the rover software stack only:
- Arduino Mega motor and gas sensor code
- Raspberry Pi 5 bridge and sensor-side scripts
- ROS 2 Jazzy workstation navigation, mapping, and visualization stack

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
  LiDAR publishing, IMU publishing, serial bridge to the Arduino, Cammera publishing
- `Workstation / Intel NUC / laptop`
  Point-LIO, Nav2, obstacle processing, RViz, and the main autonomy stack

  
## POINT-LIO UHCL BAYOU 

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(1)" src="https://github.com/user-attachments/assets/a9d678ab-add3-49fb-ae8e-0d421267d65e" />

## POINT-LIO UHCL DELTA

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(2)" src="https://github.com/user-attachments/assets/2aab6bdd-c61d-4232-a197-d264dd5c98e4" />

## NAV2 V2

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(3)" src="https://github.com/user-attachments/assets/883cee8c-5236-4894-9ea9-8c49a6e4c2e1" />

## NAV2 V3


## GAS SENSORS

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(4)" src="https://github.com/user-attachments/assets/a5b07722-dcce-43e2-8cfb-47f8dfe89325" />

## Current Working Version

The current working version in this repository is **NAV 2 Version 3**.

<img width="800" height="402" alt="ezgif com-video-to-gif-converter" src="https://github.com/user-attachments/assets/85719687-1e01-47b6-9df6-9bb7ac7e8307" />

Version 3 uses:
- native Point-LIO local odometry behavior

  <img width="803" height="365" alt="DELTA OUTSIDE POINT LIO" src="https://github.com/user-attachments/assets/aba30db1-66b8-4617-a7da-83828b8a5dfe" />

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
- Pi runs the Cammera node
- Pi publishes:
  - `/image_raw`

### Workstation autonomy layer

<img width="986" height="894" alt="image" src="https://github.com/user-attachments/assets/557ee575-0526-453b-a64d-7f7c093e4e55" />

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

- `MEGA/`
  Arduino Mega motor and gas sensor code
- `PI 5/`
  Raspberry Pi bridge and robot-side scripts
- `point_cloud_processing/`
  point cloud processing package history and references
- `docs/`
  diagrams, defense notes, tuning guides, papers, and architecture writeups


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

<img width="3287" height="1047" alt="rover_nav2_v3_flow" src="https://github.com/user-attachments/assets/1b7cfa54-c953-4fc2-984c-f589c9932b48" />

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

<img width="2035" height="1130" alt="rover_nav2_all_versions_flow" src="https://github.com/user-attachments/assets/15f39837-585e-40c9-92c0-8235dde3a5a2" />

Compared with earlier rover software branches, V3 improves:
- obstacle awareness around the full rover body
- long-run map consistency
- local control tuning for a tank-drive platform
- separation between raw LiDAR data, filtered obstacle data, and navigation control

## Documentation

Detailed project documentation is kept under `docs/`.

Useful starting points:


- `DOCS/rover_nav2_v3_flow.png`
- `DOCS/rover_nav2_all_versions_flow.png`

## Current Status

This repository reflects the rover software as developed and tested during the project.

The main working autonomy path is the live V3 navigation and mapping configuration. Some experimental follow-on work, such as saved-map localization refinements, is still under development and should be treated as secondary to the main V3 flow above.

## Author

Randy Rodriguez  
LiDAR Scout Rover software architecture, ROS 2 integration, navigation, mapping, sensing, and system bringup


