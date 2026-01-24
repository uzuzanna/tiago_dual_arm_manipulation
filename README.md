# Autonomous Dual-Arm Manipulation with TIAGo Pro

> **Repository for Engineering Thesis:** > *"Chwytanie i przenoszenie obiektów z użyciem dwóch ramion robota Tiago Pro"* > **Author:** Zuzanna Urbaniak  
> **Institution:** Poznan University of Technology (Politechnika Poznańska)

![System Simulation](tiago_pro.mp4)
*(Note: To view the simulation, download the video or view it in the repository)*

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
   git clone https://github.com/uzuzanna/tiago_dual_arm_manipulation.git
   
2. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   
3. Build the packages:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   
## How to Run
1. Launch the complete simulation stack (Gazebo + MoveIt + Rviz):
   ```bash
   ros2 launch tiago_auto_move tiago.launch.py

2. The system will automatically:
* Spawn the table and cube objects.
* Process perception data to find the cube's centroid.
* Execute the pick-and-place sequence (Right Hand -> Handover -> Left Hand -> Table).

## License
MIT

