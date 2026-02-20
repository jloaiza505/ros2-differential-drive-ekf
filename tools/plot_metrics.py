#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def load_metrics(csv_path):
    data = {
        'elapsed_sec': [],
        'samples': [],
        'rmse_raw_pos_m': [],
        'rmse_filt_pos_m': [],
        'rmse_reduction_pct': [],
        'rmse_raw_yaw_rad': [],
        'rmse_filt_yaw_rad': [],
        'raw_drift_m': [],
        'filt_drift_m': [],
        'drift_reduction_pct': [],
        'gt_travel_m': [],
    }

    def parse_float(value):
        if value is None:
            return math.nan
        value = value.strip()
        if not value:
            return math.nan
        return float(value)

    with csv_path.open('r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['elapsed_sec'].append(float(row['elapsed_sec']))
            data['samples'].append(int(row['samples']))
            data['rmse_raw_pos_m'].append(float(row['rmse_raw_pos_m']))
            data['rmse_filt_pos_m'].append(float(row['rmse_filt_pos_m']))
            data['rmse_reduction_pct'].append(float(row['rmse_reduction_pct']))
            data['rmse_raw_yaw_rad'].append(float(row['rmse_raw_yaw_rad']))
            data['rmse_filt_yaw_rad'].append(float(row['rmse_filt_yaw_rad']))
            data['raw_drift_m'].append(float(row['raw_drift_m']))
            data['filt_drift_m'].append(float(row['filt_drift_m']))
            data['drift_reduction_pct'].append(parse_float(row['drift_reduction_pct']))
            data['gt_travel_m'].append(float(row['gt_travel_m']))
    return data


def main():
    parser = argparse.ArgumentParser(description='Plot EKF metrics CSV.')
    parser.add_argument('csv_path', type=Path, help='Path to EKF CSV metrics file')
    parser.add_argument('--out', type=Path, default=Path('ekf_metrics.png'), help='Output image path')
    args = parser.parse_args()

    metrics = load_metrics(args.csv_path)
    t = metrics['elapsed_sec']
    if not t:
        raise RuntimeError('No rows found in CSV: {}'.format(args.csv_path))

    fig, axes = plt.subplots(4, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(t, metrics['rmse_raw_pos_m'], label='Raw Position RMSE', color='tab:blue')
    axes[0].plot(t, metrics['rmse_filt_pos_m'], label='Filtered Position RMSE', color='tab:green')
    axes[0].set_ylabel('Pos RMSE [m]')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, metrics['rmse_raw_yaw_rad'], label='Raw Yaw RMSE', color='tab:orange')
    axes[1].plot(t, metrics['rmse_filt_yaw_rad'], label='Filtered Yaw RMSE', color='tab:red')
    axes[1].set_ylabel('Yaw RMSE [rad]')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].plot(t, metrics['raw_drift_m'], label='Raw Drift', color='tab:blue')
    axes[2].plot(t, metrics['filt_drift_m'], label='Filtered Drift', color='tab:green')
    axes[2].set_ylabel('Drift [m]')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    axes[3].plot(t, metrics['rmse_reduction_pct'], label='RMSE Reduction %', color='tab:purple')
    axes[3].plot(t, metrics['drift_reduction_pct'], label='Drift Reduction %', color='tab:brown')
    axes[3].axhline(0.0, color='black', linewidth=1.0, linestyle='--', alpha=0.5)
    axes[3].set_ylabel('Reduction [%]')
    axes[3].grid(True, alpha=0.3)
    axes[3].legend()

    axes[3].set_xlabel('Elapsed Time [s]')

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print('Saved {}'.format(args.out))


if __name__ == '__main__':
    main()
