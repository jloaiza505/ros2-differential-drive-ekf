# ros2-differential-drive-ekf
Differential-drive robot simulation with noisy wheel odometry + noisy IMU yaw, fused by an EKF for improved state estimation.

## Overview
This project contains two runtime nodes:
- `robot_sim_pkg/robot_sim_node`: publishes simulated ground truth, noisy wheel odometry, noisy IMU yaw, and TF.
- `state_estimator_pkg/ekf_node`: fuses wheel odometry + IMU yaw into filtered odometry, publishes path traces, and logs RMSE/drift metrics.

RViz is configured to show:
- raw odometry in blue (`/wheel/odom`)
- filtered odometry in green (`/ekf/odom`)
- TF tree
- path traces for raw, filtered, and ground truth

## Workspace Build
```bash
cd /home/james/ros2_projects/ros2-differential-drive-ekf/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_sim_pkg state_estimator_pkg
source install/setup.bash
```

## One-Command Launch (Sim + EKF + RViz)
```bash
ros2 launch robot_sim_pkg phase1.launch.py
```

## Main Topics
- Ground truth odom: `/ground_truth/odom`
- Ground truth path: `/ground_truth/path`
- Wheel odom (raw): `/wheel/odom`
- Wheel path (raw): `/wheel/path`
- IMU: `/imu/data`
- EKF odom: `/ekf/odom`
- EKF path: `/ekf/path`

## Metrics Logging
`ekf_node` logs metrics every 5 seconds:
- Position RMSE for raw and filtered estimates
- Yaw RMSE for raw and filtered estimates
- RMSE reduction percentage
- Drift comparison (`raw` vs `filtered`) with drift reduction percentage

Example log:
```text
RMSE pos raw=... m filt=... m (reduction=...%),
RMSE yaw raw=... rad filt=... rad,
drift raw=... m filt=... m (...),
gt_travel=... m
```

Notes:
- `reduction > 0` means filtered is better.
- Drift reduction is reported as `n/a` when raw drift is near zero to avoid misleading percentages.

## Quick Checks
```bash
ros2 topic hz /wheel/odom
ros2 topic hz /ekf/odom
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo odom base_link
```
