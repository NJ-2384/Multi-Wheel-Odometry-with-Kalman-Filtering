# Multi-Wheel-Odometry-with-Kalman-Filtering


## Problem Statement:

Design and implement a software system for real-time odometry estimation for a 4-wheel differential drive robot.

## Algorithm Flow

Raw Encoders (Encoder_data folder) → Main → Kinematics → OdometryEstimation → KalmanFilter → Estimated Output → Main


## 🔹 Kinematics.py
- Converts raw encoder counts into:
  - Wheel displacements
  - Wheel linear velocities
  - Robot linear and angular velocity estimates
- Handles 4-wheel differential kinematics


## 🔹 KalmanFilter.py
- Implements the Kalman Filter for:
  - Pose: x, y, θ
  - Velocities: linear v, angular ω
- Fuses kinematic data with uncertainty modeling


## 🔹 OdometryEstimation.py
- Integrates kinematic outputs using the Kalman Filter
- Produces filtered:
  - Position estimates
  - Orientation
  - Velocity

## 🔹 main.py
- Entry point of the system
- Spawns multiprocessing encoder processes
- Stores filtered estimates to Estimated_Values.csv
- Shows real-time plots via matplotlib


<img width="1920" height="1032" alt="Graph" src="https://github.com/user-attachments/assets/a01c9bdf-392d-4738-85c2-3793c1b496fc" />


## ⚙️ Features

### ✔️ Multi-process and Queue
Uses Python’s multiprocessing module and Queue Datastructure to handle Asynchronous encoder ingestion and Safe concurrent data handling

### ✔️ Kinematics-based forward model
Raw encoder counts → displacement → velocity.

### ✔️ Kalman Filter
Noise modeling and state estimation for higher accuracy.

### ✔️ Real-time Visualization
Displays robot trajectory and velocities live.

### ✔️ Automatic Logging
All estimates saved into Estimated_Values.csv.
