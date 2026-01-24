# Autonomous Dual-Arm Manipulation with TIAGo Pro

> **Repository for Engineering Thesis:** > *"Chwytanie i przenoszenie obiektów z użyciem dwóch ramion robota Tiago Pro"* > **Author:** Zuzanna Urbaniak  
> **Institution:** Poznan University of Technology (Politechnika Poznańska)

![System Simulation](figures/gazebo_view.png)
*(Optional: Place a screenshot or GIF of the simulation here)*

## Overview
This repository contains ROS 2 packages developed for autonomous object manipulation using the TIAGo Pro robot. The system integrates RGB-D perception (Open3D) with MoveIt 2 motion planning to perform a "handover" task (passing a cube between left and right arms) in a simulated environment.

## Key Features
* **Perception:** Offline point cloud processing using Open3D (RANSAC segmentation, DBSCAN clustering).
* **Control:** Custom C++ MoveIt 2 controller handling dual-arm coordination.
* **Simulation:** Gazebo integration with `IFRA_LinkAttacher` for stable grasping physics.
* **Safety:** Dynamic planning group switching (`*_no_torso`) to avoid self-collisions during manipulation.

## Tech Stack
* **Robot:** TIAGo Pro (PAL Robotics)
* **OS:** Ubuntu 22.04.5 LTS (Jammy Jellyfish)
* **Middleware:** ROS 2 Humble Hawksbill
* **Libraries:** MoveIt 2, Open3D, RCLCPP, Gazebo Classic 11

## Repository Structure
* `src/tiago_auto_move`: Main C++ controller logic (`move_to_target.cpp`).
* `src/object_segmentation`: Python perception scripts (`object_segmentation.py`).
* `src/tiago_target_pose`: Helper node for publishing grasp targets.

## Installation & Build
1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone [https://github.com/YourUsername/YourRepo.git](https://github.com/YourUsername/YourRepo.git)
