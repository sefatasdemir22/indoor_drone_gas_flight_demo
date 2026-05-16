#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt


def load_samples_csv(csv_path):
    samples = []

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f'CSV bulunamadı: {csv_path}')

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            samples.append({
                'x': float(row['x']),
                'y': float(row['y']),
                'z': float(row['z']),
                'ppm': float(row['ppm']),
            })

    return samples


def build_grid_from_samples(samples, x_min, x_max, y_min, y_max, resolution):
    width = int(math.ceil((x_max - x_min) / resolution))
    height = int(math.ceil((y_max - y_min) / resolution))

    sum_grid = np.zeros((height, width), dtype=float)
    count_grid = np.zeros((height, width), dtype=int)

    for s in samples:
        x = s['x']
        y = s['y']
        ppm = s['ppm']

        if x < x_min or x >= x_max or y < y_min or y >= y_max:
            continue

        ix = int((x - x_min) / resolution)
        iy = int((y - y_min) / resolution)

        if 0 <= ix < width and 0 <= iy < height:
            sum_grid[iy, ix] += ppm
            count_grid[iy, ix] += 1

    avg_grid = np.full((height, width), np.nan, dtype=float)
    mask = count_grid > 0
    avg_grid[mask] = sum_grid[mask] / count_grid[mask]

    return avg_grid, count_grid


def expand_limits(values, padding_ratio=0.08, min_padding=1.0):
    vmin = min(values)
    vmax = max(values)

    if math.isclose(vmin, vmax):
        return vmin - min_padding, vmax + min_padding

    span = vmax - vmin
    pad = max(span * padding_ratio, min_padding)
    return vmin - pad, vmax + pad


def main():
    samples_csv = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_samples.csv')
    out_dir = os.path.expanduser('~/araswarm_ws/gas_map_logs')
    out_path = os.path.join(out_dir, 'gas_map_heatmap_pretty.png')

    os.makedirs(out_dir, exist_ok=True)

    # Mapper ile uyumlu grid parametreleri
    x_min, x_max = -5.0, 25.0
    y_min, y_max = -10.0, 10.0
    resolution = 0.5

    samples = load_samples_csv(samples_csv)

    if not samples:
        print('Sample CSV boş. Önce veri toplanmalı.')
        return

    heatmap, count_grid = build_grid_from_samples(
        samples=samples,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max,
        resolution=resolution
    )

    valid_values = heatmap[~np.isnan(heatmap)]
    if valid_values.size == 0:
        print('Grid içinde geçerli hücre yok.')
        return

    vmin = float(np.min(valid_values))
    vmax = float(np.max(valid_values))
    if math.isclose(vmin, vmax):
        vmax = vmin + 1e-6

    xs = [s['x'] for s in samples]
    ys = [s['y'] for s in samples]

    plot_x_min, plot_x_max = expand_limits(xs, padding_ratio=0.10, min_padding=1.0)
    plot_y_min, plot_y_max = expand_limits(ys, padding_ratio=0.10, min_padding=1.0)

    fig, ax = plt.subplots(figsize=(12, 7))

    im = ax.imshow(
        heatmap,
        origin='lower',
        interpolation='bilinear',
        aspect='auto',
        extent=[x_min, x_max, y_min, y_max],
        vmin=vmin,
        vmax=vmax
    )

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label('Average PPM')

    # Trajectory line
    ax.plot(xs, ys, linewidth=1.5, alpha=0.9, label='Drone trajectory')

    # Sample points
    ax.scatter(xs, ys, s=10, alpha=0.8)

    # Start / End markers
    ax.scatter(xs[0], ys[0], s=60, marker='o', label='Start')
    ax.scatter(xs[-1], ys[-1], s=80, marker='x', label='End')

    ax.set_title('Gas Concentration Heatmap')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Tüm world yerine örneklenen bölgeyi odakla
    ax.set_xlim(plot_x_min, plot_x_max)
    ax.set_ylim(plot_y_min, plot_y_max)

    ax.grid(True, alpha=0.2)
    ax.legend()
    plt.tight_layout()

    plt.savefig(out_path, dpi=220)
    print(f'Heatmap kaydedildi: {out_path}')
    print(f'Toplam sample sayısı: {len(samples)}')
    print(f'Dolu hücre sayısı: {np.count_nonzero(~np.isnan(heatmap))}')

    plt.show()


if __name__ == '__main__':
    main()
