Autonomous Robotics Platform: Sensor Fusion & Motor Control
Unibots_UCL_TeamDylan A modular, ROS 2-powered robotics stack integrating multi-modal sensor fusion (LiDAR, Vision, Odometry) and robust motor control for autonomous navigation on the Jetson Orin Nano.

System Overview
This repository contains the full autonomous mobile robot (AMR) software and hardware integration stack. The architecture is designed for scalability, separating high-level perception and path-planning tasks from strict real-time low-level motor actuation.

Hardware Architecture
Main Compute (High-Level): NVIDIA Jetson Orin Nano (Running ROS 2, SLAM, Vision Processing)

Microcontroller (Low-Level): Arduino Uno R4 WiFi (Handling real-time serial parsing, PID control, encoder interrupts)

Perception Sensors:

STL27L 360° LiDAR (Obstacle detection, 2D/3D SLAM mapping)

Raspberry Pi Camera (Visual odometry, object recognition pipelines)

Actuation & Proprioception:

4× Geared Metal Motors with Quadrature Encoders

HW-627 Motor Driver

Software Stack
Middleware: ROS 2 (Humble/Iron)

Navigation: Nav2 stack integration for dynamic path planning and obstacle avoidance

Mapping: Real-time SLAM algorithms utilizing fused LiDAR and visual data

Control: Custom serial bridge between ROS 2 cmd_vel topics and Arduino-level PID motor control

Getting Started
Prerequisites
Ubuntu 22.04 / 24.04

ROS 2 installed and configured

Arduino IDE (for flashing low-level control scripts)

(Add your installation and build instructions here using colcon build)

Deep Dive: Understanding Your Architecture
To master this system, you need to understand the theoretical foundation of your design choices. You are utilizing a classic dual-tier architecture common in commercial robotics.

1. The Compute Hierarchy (Asynchronous vs. Real-Time)
The Jetson Orin Nano is an incredibly powerful AI edge computer, but it runs a standard Linux kernel (even with PREEMPT_RT, it's not perfect for microsecond precision). Operating systems like Linux manage thousands of threads. If you try to read high-speed quadrature encoders directly via the Jetson's GPIO pins, the OS might preempt your reading thread to handle a network packet, causing you to miss encoder ticks and ruining your odometry.
This is why your Arduino Uno R4 is critical. Microcontrollers lack an OS; they run a single loop with hardware-level interrupts. The Arduino perfectly counts encoder ticks and runs the PID (Proportional-Integral-Derivative) control loops to maintain exact wheel velocities. The Jetson just sends high-level commands over Serial (e.g., "move forward at 1.5 m/s") and the Arduino handles the physical reality of making that happen.

2. Sensor Fusion & The State Estimation Problem
You have an STL27L LiDAR, a Pi Cam, and Encoders. Why all three?

Encoders (Proprioception): Tell you how much the wheels have turned. They are highly accurate over short distances but suffer from cumulative error (drift) due to wheel slip.

LiDAR & Camera (Exteroception): Look at the outside world to correct that drift.
Sensor fusion mathematically combines these conflicting streams of data—usually via an Extended Kalman Filter (EKF)—to produce a single, highly confident estimate of the robot's pose (position and orientation).
