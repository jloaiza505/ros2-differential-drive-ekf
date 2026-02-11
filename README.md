# ros2-differential-drive-ekf
Differential drive robot simulation with encoder/IMU fusion and Extended Kalman Filter–based state estimation for drift reduction and pose accuracy analysis.

Problem Statement

Mobile robots operating in real environments suffer from sensor noise and wheel slip, leading to drift and inaccurate localization. This project addresses the challenge of estimating robot pose using noisy encoder and IMU data. The system implements a full kinematic model and compares raw odometry to filtered state estimates using an Extended Kalman Filter. The objective is to quantify drift reduction and demonstrate robust real-time state estimation in simulation.
