# ros2-differential-drive-ekf
ROS 2 project that simulates a differential-drive robot, injects sensor noise, and estimates pose with an Extended Kalman Filter (EKF). The pipeline compares raw odometry against filtered output and reports quantitative error metrics.

## Why This Project
Wheel odometry drifts over time, especially with noise and slip-like perturbations. This project demonstrates:
- Kinematic simulation for a differential-drive platform
- Multi-sensor fusion (wheel odometry + IMU yaw) with EKF
- Quantitative evaluation (RMSE and drift reduction)
- Visualization and experiment reproducibility with ROS 2 launch + RViz

## Architecture
```mermaid
flowchart LR
    SIM[robot_sim_node] --> GT[/ground_truth/odom]
    SIM --> WO[/wheel/odom]
    SIM --> IMU[/imu/data]
    SIM --> TF[TF: odom -> base_link]
    WO --> EKF[ekf_node]
    IMU --> EKF
    GT --> EKF
    EKF --> EODOM[/ekf/odom]
    EKF --> WPATH[/wheel/path]
    EKF --> EPATH[/ekf/path]
    EKF --> GPATH[/ground_truth/path]
    EODOM --> RVIZ[RViz]
    WPATH --> RVIZ
    EPATH --> RVIZ
    GPATH --> RVIZ
    TF --> RVIZ
```

## Repo Layout
- `ros2_ws/src/robot_sim_pkg`: simulator, RViz config, launch file
- `ros2_ws/src/state_estimator_pkg`: EKF node and metrics collection
- `tools/plot_metrics.py`: utility to plot CSV experiment output

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- `numpy`
- Optional for plots: `matplotlib`

Install plotting dependency:
```bash
python3 -m pip install matplotlib
```

## Build
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_sim_pkg state_estimator_pkg
source install/setup.bash
```

## One-Command Run
```bash
ros2 launch robot_sim_pkg phase1.launch.py
```

RViz opens with:
- Raw odometry in blue (`/wheel/odom`)
- Filtered odometry in green (`/ekf/odom`)
- Ground-truth path in gray (`/ground_truth/path`)
- Raw and filtered path traces (`/wheel/path`, `/ekf/path`)
- TF tree (fixed frame: `odom`)

## Launch Parameters (Experiment Knobs)
You can tune sim and EKF at launch time:
```bash
ros2 launch robot_sim_pkg phase1.launch.py \
  left_wheel_noise_std:=0.18 right_wheel_noise_std:=0.18 \
  imu_yaw_noise_std:=0.08 imu_wz_noise_std:=0.04 \
  r_imu_yaw:=0.20 q_yaw:=0.02 \
  metrics_csv_path:=/tmp/ekf_metrics.csv random_seed:=7
```

Important parameters:
- Simulator:
  - `left_wheel_noise_std`, `right_wheel_noise_std`
  - `imu_yaw_noise_std`, `imu_wz_noise_std`
  - `random_seed` (for reproducible runs)
- EKF:
  - `q_pos`, `q_yaw`
  - `r_odom_pos`, `r_imu_yaw`
  - `use_wheel_pos_update`
  - `metrics_period_sec`
  - `metrics_csv_path`

## Metrics
`ekf_node` prints periodic metrics:
- Position RMSE (raw vs filtered)
- Yaw RMSE (raw vs filtered)
- RMSE reduction percentage
- Drift (raw vs filtered) and drift reduction percentage

If `metrics_csv_path` is set, metrics are also saved to CSV for post-processing.

## Plotting Results
```bash
python3 tools/plot_metrics.py /tmp/ekf_metrics.csv --out /tmp/ekf_metrics.png
```

## Suggested Portfolio Results Section
Include these artifacts in your project page:
1. RViz screenshot with all traces visible
2. Metrics plot (`raw` vs `filtered` RMSE/drift over time)
3. Table with averaged results over multiple seeds

Example table template:

| Seed | Raw Pos RMSE (m) | Filtered Pos RMSE (m) | Improvement (%) |
|---|---:|---:|---:|
| 1 | ... | ... | ... |
| 2 | ... | ... | ... |
| 3 | ... | ... | ... |
| Mean | ... | ... | ... |

## Quick Debug Commands
```bash
ros2 topic hz /wheel/odom
ros2 topic hz /ekf/odom
ros2 topic hz /imu/data
ros2 run tf2_ros tf2_echo odom base_link
```

## Limitations
- Without an absolute position sensor (e.g., GPS, landmarks), EKF position correction is limited.
- Current process/measurement models are intentionally simple for educational clarity.

## Next Improvements
- Add automated launch/integration tests
- Add richer process model (bias states, slip model)
- Add absolute position update source for stronger drift suppression
