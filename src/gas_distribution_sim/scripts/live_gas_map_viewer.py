#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import time
import math
import matplotlib.pyplot as plt


CSV_PATH = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_samples_3d.csv')


def load_samples(csv_path):
    samples = []

    if not os.path.exists(csv_path):
        return samples

    try:
        with open(csv_path, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                samples.append({
                    'x': float(row['x']),
                    'y': float(row['y']),
                    'z': float(row['z']),
                    'ppm': float(row['ppm']),
                })
    except Exception as e:
        print(f"CSV okuma hatası: {e}")

    return samples


def expand_limits(values, padding_ratio=0.08, min_padding=1.0):
    vmin = min(values)
    vmax = max(values)

    if math.isclose(vmin, vmax):
        return vmin - min_padding, vmax + min_padding

    span = vmax - vmin
    pad = max(span * padding_ratio, min_padding)
    return vmin - pad, vmax + pad


def main():
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 6))

    while plt.fignum_exists(fig.number):
        samples = load_samples(CSV_PATH)

        ax.clear()

        if samples:
            xs = [s['x'] for s in samples]
            ys = [s['y'] for s in samples]
            ppms = [s['ppm'] for s in samples]

            ax.plot(xs, ys, linewidth=1.5, alpha=0.85, label='Drone trajectory')
            sc = ax.scatter(xs, ys, c=ppms, s=18)

            ax.scatter(xs[0], ys[0], s=80, marker='o', label='Start')
            ax.scatter(xs[-1], ys[-1], s=100, marker='x', label='Current')

            x_min, x_max = expand_limits(xs, padding_ratio=0.10, min_padding=1.0)
            y_min, y_max = expand_limits(ys, padding_ratio=0.10, min_padding=1.0)

            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)

            cbar = fig.colorbar(sc, ax=ax)
            cbar.set_label('PPM')

            ax.set_title(f'Live Gas Map Viewer | samples={len(samples)}')
        else:
            ax.set_title('Live Gas Map Viewer | sample bekleniyor...')

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.grid(True, alpha=0.25)
        ax.legend(loc='upper right')
        plt.tight_layout()
        plt.pause(0.5)

    plt.ioff()


if __name__ == '__main__':
    main()
