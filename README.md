# Autonomous Robotics Platform: Unibots_UCL_TeamDylan

A ROS 2-powered autonomous mobile robot developed by **Unibots_UCL_TeamDylan**, designed for high-speed object retrieval and environment navigation within strict competition time limits. 

---

## Description

This repository contains the full software stack for an autonomous robotic platform built on the NVIDIA Jetson Orin Nano. The system utilizes edge-accelerated machine learning (TensorRT YOLO) and fiducial marker tracking (AprilTags) to execute a continuous search-and-collect state machine. Low-level actuation is handled by an STM32 microcontroller driving a mecanum chassis via a custom UART serial bridge.

## Hardware Architecture

| Component | Function |
| :--- | :--- |
| **Jetson Orin Nano** | Main compute node (ROS 2). Handles memory-constrained TensorRT neural network inference, USB camera streaming, and high-level autonomous state machine logic. |
| **STM32 Microcontroller** | Low-level hardware execution. Parses UART string commands and maps kinematics to the motor drivers. |
| **3× DRV8833 Motor Drivers** | Provides 6-channel dual H-bridge power distribution for the drivetrain and collector mechanisms. |
| **4× Mecanum Wheels** | Omnidirectional locomotion setup allowing for complex vectoring and tank-style rotations. |
| **USB Camera Module** | Primary visual sensor for object detection and AprilTag coordinate extraction via OpenCV/V4L2. |
| **STL27L 360° LiDAR** | Provides 2D point cloud data for immediate obstacle avoidance overrides. |
| **LSM6DSOXTR IMU** | Provides rotational velocity data via I2C to ensure accurate orientation tracking. |

## Core Capabilities

* **Edge-Accelerated Vision:** Real-time object detection running a YOLO model compiled directly to an NVIDIA TensorRT engine (`.engine`) to conform to strict Unified Memory constraints.
* **Image-Based Visual Servoing (IBVS):** Dynamic PID-style navigation that calculates motor velocities based on pixel bounding box geometry and AprilTag pose estimation.
* **Mission-Critical State Machine:** A mathematically rigid 2.5-minute autonomous loop that handles radar-style sweeping, obstacle avoidance, ball collection, and automated return-to-home protocols.
* **Custom UART Serial Bridge:** A lightweight, non-blocking serial parser designed to translate ROS 2 decisions into raw physical execution without the overhead of heavy middleware.
