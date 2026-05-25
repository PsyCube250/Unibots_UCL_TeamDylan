# Autonomous Robotics Platform with Sensor Fusion and Motor Control

A ROS 2-powered robotics project developed by **Unibots_UCL_TeamDylan**, integrating LiDAR, vision, and encoder feedback for navigation and control on the Jetson Orin Nano.

---

## Description

This repository contains the full stack for a mobile robot designed for autonomous navigation and environment perception. Built for scalability and modularity, the system supports advanced sensor fusion, dynamic path planning, and remote operation.

## Hardware Architecture

| Component | Function |
| :--- | :--- |
| **Jetson Orin Nano** | Main compute unit; runs ROS 2, handles Object Detection and Lidar, and processes vision/sensor data. |
| **Arduino Uno R4 WiFi** | Microcontroller for low-level motor control via serial communication. |
| **HW-627 Motor Driver** | Manages power distribution and control signals to the motors. |
| **STL27L 360° LiDAR** | Provides 2D point cloud data for real-time SLAM and obstacle detection. |
| **Pi Cam** | Captures visual data for object recognition and environment monitoring. |
| **4× Geared Metal Motors** | Provides locomotion, equipped with encoders for precise odometry and movement feedback. |

## Core Capabilities

* **Sensor Fusion:** Combines LiDAR odometry, visual data, and wheel encoder feedback for robust state estimation.
* **Real-Time Detection and Actuation:** Simultaneous Localization and Mapping using 360° spatial data.
* **Modular ROS 2 Stack:** Designed using ROS 2 nodes for decoupled, scalable hardware and software integration.
