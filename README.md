# Multi-Wheel-Odometry-with-Kalman-Filtering

## Problem Statement:

Design and implement a software system for real-time odometry estimation for a 4-wheel differential drive robot.


## 🔹 Kinematics.py
- Converts raw encoder counts into:
  - Wheel displacements
  - Wheel linear velocities
  - Robot linear and angular velocity estimates
- Handles 4-wheel differential kinematics

## 🔹 KalmanFilter.py
- Implements the Extended Kalman Filter (EKF) for:
  - Pose: x, y, θ
  - Velocities: linear v, angular ω
- Fuses kinematic data with uncertainty modeling

## 🔹 OdometryEstimation.py
- Integrates kinematic outputs using the EKF
- Produces filtered:
  - Position estimates
  - Orientation
  - Velocity

## 🔹 main.py
- Entry point of the system
- Spawns multiprocessing encoder processes
- Stores filtered estimates to Estimated_Values.csv
- Shows real-time plots via matplotlib

## ⚙️ Features

### ✔️ Multi-process sensor pipeline
Uses Python’s multiprocessing module to simulate or stream encoder readings.

### ✔️ Kinematics-based forward model
Raw encoder counts → displacement → velocity → motion.

### ✔️ Extended Kalman Filter
Noise modeling and state estimation for higher accuracy.

### ✔️ Real-time Visualization
Displays robot trajectory and velocities live.

### ✔️ Automatic Logging
All estimates saved into Estimated_Values.csv.

## ▶️ How to Run

### 1. Install dependencies
