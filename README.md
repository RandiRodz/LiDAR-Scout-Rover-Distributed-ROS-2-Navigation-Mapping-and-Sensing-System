# LiDAR-Scout-Rover-Distributed-ROS-2-Navigation-Mapping-and-Sensing-System

Software repository for a distributed ROS 2 rover system built around Unitree LiDAR L1, gas sensing, and autonomous navigation.
This repository focuses on the rover software stack only:
- Arduino Mega motor and gas sensor code
- Raspberry Pi 5 bridge and sensor-side scripts
- ROS 2 Jazzy workstation navigation, mapping, and visualization stack

Here is the main repository link
[![Lidar Scout Rover](https://shields.io)](https://github.com/RandiRodz/Lidar-Scout-Rover
)


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

 ## DEMO Videos

[![YouTube](https://shields.io)](https://www.youtube.com/channel/UCsQoXkuwZl-ijHmh91t0s0w)

## POINT-LIO UHCL BAYOU 

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(1)" src="https://github.com/user-attachments/assets/a9d678ab-add3-49fb-ae8e-0d421267d65e" />

## POINT-LIO UHCL DELTA

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(2)" src="https://github.com/user-attachments/assets/2aab6bdd-c61d-4232-a197-d264dd5c98e4" />

## NAV2 V2

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(3)" src="https://github.com/user-attachments/assets/883cee8c-5236-4894-9ea9-8c49a6e4c2e1" />

## NAV2 V3

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(5)" src="https://github.com/user-attachments/assets/2d466143-c165-4855-b4a9-49adb898744a" />

## GAS SENSORS

<img width="800" height="450" alt="ezgif com-video-to-gif-converter(4)" src="https://github.com/user-attachments/assets/a5b07722-dcce-43e2-8cfb-47f8dfe89325" />

## Current Working Version

The current working version in this repository is [![**NAV 2 Version 3**](https://shields.io)](https://github.com/RandiRodz/LiDAR-Scout-Rover-Distributed-ROS-2-Navigation-Mapping-and-Sensing-System/tree/main/ws_lsr_nav2_v3
)



Version 3 uses:
- native Point-LIO local odometry behavior
- `slam_toolbox` map correction during live operation
- a 360-degree point-cloud obstacle pipeline for Nav2
- tank-drive tuned Nav2 control parameters

This is the version that tested best as a complete software path in the rover project.

## Software Architecture

<img width="3487" height="793" alt="lidar_scout_rover_software_pipeline" src="https://github.com/user-attachments/assets/d8ce97e1-bd6f-448d-96c3-f6c25b45a4a5" />


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
- Pi runs the Camera node
- Pi publishes:
  - `/image_raw`

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

- `MEGA/`
  Arduino Mega motor and gas sensor code
- `PI 5/`
  Raspberry Pi bridge and robot-side scripts
- `DOCS/`
  diagrams, defense notes, tuning guides, papers, and architecture writeups
- `DOCS/metrics_logs/cpu_comparison_report.md` 
[![cpu_comparison_report.md](https://shields.io)](https://github.com/RandiRodz/LiDAR-Scout-Rover-Distributed-ROS-2-Navigation-Mapping-and-Sensing-System/blob/main/DOCS/metrics_logs/cpu_comparison_report.md)


## Main V3 Files

The Version 3 stack is centered around these files:

- `/ws_lsr_nav2_v3/src/lsr_nav2/launch/rover_nav2_v3.launch.py`
- `/ws_lsr_nav2_v3/src/lsr_nav2/config/nav2_v3_params.yaml`
- `/ws_lsr_nav2_v3/src/lsr_nav2/config/slam_toolbox_v3_online_async.yaml`
- `/ws_lsr_nav2_v3/src/lsr_nav2/rviz/rover_nav2_v3_view.rviz`

Important supporting files:

- `/ws_lsr_nav2_v3/src/lsr_nav2/src/nav2_obstacle_cloud.cpp`
- `/ws_lsr_nav2_v3/src/lsr_nav2/src/advanced_proximity_guard.cpp`
- `/ws_lsr_nav2_v3/src/lsr_nav2/src/cmd_vel_mux.cpp`
- `/ws_lsr_nav2_v3/src/lsr_nav2/src/odom_retimestamp_bridge.cpp`
- `/ws_lsr_nav2_v3/src/lsr_nav2/src/lidar_retimestamp_bridge.cpp`

## How To Run The Current Working V3 Stack

<img width="3287" height="1047" alt="rover_nav2_v3_flow" src="https://github.com/user-attachments/assets/1b7cfa54-c953-4fc2-984c-f589c9932b48" />

### 1. Start the LiDAR driver on the Pi

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```
### 2. Start Point-LIO

```bash
cd ws_pointlio
source ~/ws_pointlio/install/setup.bash
ros2 launch point_lio mapping_unilidar_l1.launch.py rviz:=false
```
### 3. Start the workstation autonomy stack

```bash
source /opt/ros/jazzy/setup.bash
source /home/ros2/ws_pointlio/install/setup.bash
source /home/ros2/ws_lsr_nav2_v3/install/setup.bash
ros2 launch point_cloud_processing rover_nav2_v3.launch.py
```

### 4. Optional keyboard teleop

```bash
source /opt/ros/jazzy/setup.bash
source /home/ros2/ws_lsr_nav2_v3/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_teleop
```

### 5. Optional Camera

```bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
-p video_device:="/dev/video0" \
-p image_size:="[640,480]" \
-p output_encoding:="rgb8"
```
### 6. Network & DDS Setup (Ethernet / Wi-Fi Router)

To enable reliable, low-latency ROS 2 Jazzy node discovery across multiple devices (e.g., PC/NUC and Raspberry Pi) over a router or direct Ethernet link, CycloneDDS must be configured to bind directly to the active network interface and use static peer discovery.

#### Configure DDS Environment Variables

Add the following configuration to your `~/.bashrc` file on your main host system:

```bash
# Open bash configuration
nano ~/.bashrc
```

Append these lines at the bottom:
```bash

# ==========================================
# ROS 2 Workspace Sourcing
# ==========================================
source /opt/ros/jazzy/setup.bash
source ~/point_lio_ws/install/setup.bash
source ~/ws_lsr_nav2_v2/install/setup.bash

# ==========================================
# DDS Network Configuration
# ==========================================
# Domain ID isolation
export ROS_DOMAIN_ID=17

# Force CycloneDDS middleware implementation
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Define static peer IP addresses (localhost and remote Pi IP)
export ROS_STATIC_PEERS="127.0.0.1;192.168.0.225"

# Bind CycloneDDS directly to your active Ethernet interface (replace 'enp89s0' with your interface name)
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp89s0"/></Interfaces></General></Domain></CycloneDDS>'

# Disable automatic discovery range to restrict traffic to specified static peers
unset ROS_AUTOMATIC_DISCOVERY_RANGE
```

Apply the changes to your current terminal session:
```bash
source ~/.bashrc
```
#### Hardware Connection Overview

  Intel NUC (PC): Connects directly to the router via a high-speed Ethernet cable 
  on interface enp89s0 for maximum processing bandwidth and point-cloud throughput.

  Raspberry Pi 5: Automatically connects to the router's Wi-Fi network on boot, 
  allowing wireless telemetry, sensor data relay, and motor control routing.
    
## What V3 Improves

<img width="2035" height="1130" alt="rover_nav2_all_versions_flow" src="https://github.com/user-attachments/assets/15f39837-585e-40c9-92c0-8235dde3a5a2" />

Compared with earlier rover software branches, V3 improves:
- obstacle awareness around the full rover body
- long-run map consistency
- local control tuning for a tank-drive platform
- separation between raw LiDAR data, filtered obstacle data, and navigation control
  
## Odometry Drifting

Odometry drift is the gradual accumulation of error in a robot’s estimated position and orientation over time. In mobile robotics, this causes the reported pose to slowly diverge from the rover’s true position, even when the system appears to be functioning normally.
<img width="745" height="217" alt="ezgif com-crop" src="https://github.com/user-attachments/assets/6bbb4615-04d1-43af-99ef-ebf1f774d2fc" />

The GIF above shows version 2 with odometry drifting when Stationary.
<img width="774" height="362" alt="ezgif com-crop(1)" src="https://github.com/user-attachments/assets/9183c86d-9b86-44cd-8a7a-66a8f29b39a1" />

The GIF above shows version 2 with odometry drifting when Moving.

<img width="1872" height="1048" alt="Screenshot from 2026-05-12 23-54-44" src="https://github.com/user-attachments/assets/6f9fbc3b-e867-4fcc-8dcd-3f24413f5e29" />
The figure above shows IMU orientation data from the /unilidar/imu topic visualized in PlotJuggler.(Moving)

<img width="1872" height="1048" alt="Screenshot from 2026-05-12 23-54-53" src="https://github.com/user-attachments/assets/3f9cc39a-a807-46f7-8a40-7dc605bb1094" />
Even while the rover was stationary, the signals showed high-frequency oscillations and slow drift over time.(Stationary)



This behavior is consistent with vibration coupling from the spinning LiDAR assembly into the built-in IMU. Because the LiDAR physically rotates during operation, mechanical vibration from the internal motor can be transferred directly into the IMU measurements. This produces high-frequency oscillations that appear as pitch and roll jitter even when the rover is not moving. Since the IMU is integrated inside the LiDAR unit, small imbalances in the rotating mechanism may also introduce a persistent bias. Over time, that bias can be interpreted by the odometry or state-estimation pipeline as slow motion or rotation, which contributes to yaw drift and instability in the estimated pose.

As a result, the rover may appear to “shiver” in RViz through /odom or /tf despite remaining physically stationary. To reduce the long-term impact of this effect, map-based correction was integrated in Version 3 so that localization could be periodically realigned against the environment rather than relying only on odometry.

Potential Fixes

Mechanical damping: Use rubber vibration isolators between the LiDAR mount and the rover chassis to reduce the transmission of motor-induced vibration into the frame and IMU.

Zero-velocity updates: Configure the state-estimation system to detect when the rover is stationary so that IMU drift can be suppressed or weighted less heavily while the wheels are not moving.

Additional filtering or tuning: Tune the odometry or sensor-fusion pipeline to better reject the dominant vibration frequency and reduce the effect of IMU noise on /odom.

## Current Status

This repository reflects the rover software as developed and tested during the project.

The main working autonomy path is the live V3 navigation and mapping configuration. Some experimental follow-on work, such as saved-map localization refinements, is still under development and should be treated as secondary to the main V3 flow above.


## Author

Randy Rodriguez  
LiDAR Scout Rover software architecture, ROS 2 integration, navigation, mapping, sensing, and system bringup
  <img width="803" height="365" alt="DELTA OUTSIDE POINT LIO" src="https://github.com/user-attachments/assets/aba30db1-66b8-4617-a7da-83828b8a5dfe" />



