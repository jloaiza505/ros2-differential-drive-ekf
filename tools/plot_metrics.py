#!/usr/bin/env python3
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt


def load_metrics(csv_path):
    t = []
    rmse_raw = []
    rmse_filt = []
    drift_raw = []
    drift_filt = []
    with csv_path.open('r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t.append(float(row['elapsed_sec']))
            rmse_raw.append(float(row['rmse_raw_pos_m']))
            rmse_filt.append(float(row['rmse_filt_pos_m']))
            drift_raw.append(float(row['raw_drift_m']))
            drift_filt.append(float(row['filt_drift_m']))
    return t, rmse_raw, rmse_filt, drift_raw, drift_filt


def main():
    parser = argparse.ArgumentParser(description='Plot EKF metrics CSV.')
    parser.add_argument('csv_path', type=Path, help='Path to EKF CSV metrics file')
    parser.add_argument('--out', type=Path, default=Path('ekf_metrics.png'), help='Output image path')
    args = parser.parse_args()

    t, rmse_raw, rmse_filt, drift_raw, drift_filt = load_metrics(args.csv_path)
    if not t:
        raise RuntimeError('No rows found in CSV: {}'.format(args.csv_path))

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot(t, rmse_raw, label='Raw Position RMSE', color='tab:blue')
    axes[0].plot(t, rmse_filt, label='Filtered Position RMSE', color='tab:green')
    axes[0].set_ylabel('RMSE [m]')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, drift_raw, label='Raw Drift', color='tab:blue')
    axes[1].plot(t, drift_filt, label='Filtered Drift', color='tab:green')
    axes[1].set_ylabel('Drift [m]')
    axes[1].set_xlabel('Elapsed Time [s]')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=150)
    print('Saved {}'.format(args.out))


if __name__ == '__main__':
    main()
