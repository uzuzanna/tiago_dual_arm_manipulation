# Autonomous Dual-Arm Manipulation with TIAGo Pro

Project for Engineering Thesis: **"Chwytanie i przenoszenie obiektów z
użyciem dwóch ramion robota Tiago Pro"**

## Overview
This repository contains ROS 2 packages developed for autonomous object manipulation using the TIAGo Pro robot. The system integrates RGB-D perception (Open3D) with MoveIt 2 motion planning to perform a "handover" task (passing a cube between left and right arms).

## Key Features
* **Perception:** Offline point cloud processing using Open3D (RANSAC segmentation, DBSCAN clustering).
* **Control:** Custom C++ MoveIt 2 controller handling dual-arm coordination.
* **Simulation:** Gazebo integration with `IFRA_LinkAttacher` for stable grasping.
* **Safety:** Dynamic planning group switching (`*_no_torso`) to avoid self-collisions.

## Tech Stack
* **Robot:** TIAGo Pro (PAL Robotics)
* **OS:** Ubuntu 22.04.5 LTS
* **Middleware:** ROS 2 Humble
* **Libraries:** MoveIt 2, Open3D, RCLCPP

## Structure
* `src/tiago_auto_move`: Main C++ controller logic (`move_to_target.cpp`).
* `src/object_segmentation`: Python perception scripts (`object_segmentation.py`).
* `src/tiago_target_pose`: Helper node for publishing grasp targets.

## How to run
1. Launch the simulation and MoveIt:
   ```bash
   ros2 launch tiago_auto_move system_bringup.launch.py
2. The system will automatically:
	- Spawn the table and cube.
	- Process perception data.
	- Execute the pick-and-place sequence.
	
## Author

Zuzanna Urbaniak Poznan University of Technology (Politechnika Poznańska)
