# AUTONOMOUS SHRIMP PLATFORM

Autonomous exploration robotic platform developed using ROS2, Nav2 and Gazebo for autonomous navigation, obstacle avoidance and intelligent perception using stereo vision.

## Project Overview

This project consists of a 4-wheel drive autonomous robotic platform designed for:

- Autonomous navigation
- Obstacle avoidance
- Path planning and trajectory tracking
- Stereo vision integration
- Depth map generation
- ArUco marker detection
- Intelligent navigation based on uncertainty estimation

The system is being developed as a modular robotics engineering project integrating:

- Control systems
- Embedded electronics
- Artificial intelligence
- Computer vision
- Autonomous robotics

---

## Hardware

Main hardware components:

- NVIDIA Jetson Nano 4GB Developer Kit
- Waveshare Stereo Camera Dual IMX219 8MP
- 4WD robotic chassis
- JGB-37520 DC motors with encoders
- Custom motor driver stage
- Li-Ion battery system

---

## Software Stack

Main technologies used in this project:

- ROS2 Humble
- Nav2 Navigation Stack
- Gazebo Simulation
- Foxglove Studio
- OpenCV
- Python
- Ubuntu 22.04

---

## Current Features

- Differential drive / skid steering control
- Custom odometry node
- Autonomous navigation using Nav2
- MPPI local planner integration
- Global path planning
- Obstacle costmaps
- Trajectory visualization in Foxglove
- Gazebo simulation environment
- Stereo vision calibration
- Depth map generation
- ArUco marker detection

---

## Project Architecture

Main ROS2 nodes:

- `shrimp_controller`
- `odom_tf_node`
- `navigation_launch`
- `stereo_camera_node`
- `aruco_detector`
- `depth_processing_node`

---

## Navigation System

The robot currently uses:

- Nav2 Stack
- MPPI Controller
- Smac Hybrid Planner
- Costmaps with obstacle inflation
- Custom low-level controller

Visualization is performed using Foxglove Studio instead of RViz.

---

## Future Work

Planned features include:

- Extended Kalman Filter (EKF)
- Covariance-based adaptive trajectory control
- Intelligent trajectory correction
- Vision-based obstacle avoidance
- Autonomous exploration behaviors
- Sensor fusion
- Real-time uncertainty estimation

---

## Simulation

The project includes Gazebo simulation support for:

- Robot dynamics
- Autonomous navigation
- Sensor validation
- Stereo camera testing
- Path planning evaluation

---

## Build Instructions

Clone the repository inside your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Ivant44/AUTONOMUS_SHRIMP_PLATFORM.git
```

Build the workspace:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Launch

Example launch command:

```bash
ros2 launch shrimp_bringup navigation.launch.py
```

---

## Author

Ivan Israel Torres Ortiz

Robotics and Autonomous Systems Engineering Project.
