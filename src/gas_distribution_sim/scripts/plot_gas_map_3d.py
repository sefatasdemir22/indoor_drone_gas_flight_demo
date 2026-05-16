#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import numpy as np
import matplotlib.pyplot as plt


def load_voxel_csv(csv_path):
    voxels = []

    if not os.path.exists(csv_path):
        raise FileNotFoundError(f'CSV bulunamadı: {csv_path}')

    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            voxels.append({
                'voxel_ix': int(row['voxel_ix']),
                'voxel_iy': int(row['voxel_iy']),
                'voxel_iz': int(row['voxel_iz']),
                'center_x': float(row['center_x']),
                'center_y': float(row['center_y']),
                'center_z': float(row['center_z']),
                'sample_count': int(row['sample_count']),
                'sum_ppm': float(row['sum_ppm']),
                'avg_ppm': float(row['avg_ppm']),
            })

    return voxels


def expand_limits(values, padding_ratio=0.10, min_padding=0.5):
    vmin = min(values)
    vmax = max(values)

    if math.isclose(vmin, vmax):
        return vmin - min_padding, vmax + min_padding

    span = vmax - vmin
    pad = max(span * padding_ratio, min_padding)
    return vmin - pad, vmax + pad


def main():
    voxel_csv = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_voxel.csv')
    out_dir = os.path.expanduser('~/araswarm_ws/gas_map_logs')
    out_path = os.path.join(out_dir, 'gas_map_3d_scatter.png')

    os.makedirs(out_dir, exist_ok=True)

    voxels = load_voxel_csv(voxel_csv)

    if not voxels:
        print('Voxel CSV boş. Önce 3D mapper veri toplamalı.')
        return

    xs = np.array([v['center_x'] for v in voxels], dtype=float)
    ys = np.array([v['center_y'] for v in voxels], dtype=float)
    zs = np.array([v['center_z'] for v in voxels], dtype=float)
    ppms = np.array([v['avg_ppm'] for v in voxels], dtype=float)
    counts = np.array([v['sample_count'] for v in voxels], dtype=float)

    # Nokta boyutları: sample_count'a göre ama kontrollü
    sizes = 30.0 + 20.0 * np.sqrt(counts)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')

    sc = ax.scatter(
        xs,
        ys,
        zs,
        c=ppms,
        s=sizes,
        alpha=0.85
    )

    cbar = plt.colorbar(sc, ax=ax, pad=0.12)
    cbar.set_label('Average PPM')

    ax.set_title('3D Gas Voxel Map')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    x_min, x_max = expand_limits(xs.tolist(), padding_ratio=0.10, min_padding=0.5)
    y_min, y_max = expand_limits(ys.tolist(), padding_ratio=0.10, min_padding=0.5)
    z_min, z_max = expand_limits(zs.tolist(), padding_ratio=0.10, min_padding=0.5)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)

    # Başlangıç ve bitiş gibi kaba görsel ipucu için
    # düşük yoğunluk / yüksek yoğunluk bölgelerini ayırt etmeyi kolaylaştırır
    hottest_idx = int(np.argmax(ppms))
    coldest_idx = int(np.argmin(ppms))

    ax.scatter(xs[hottest_idx], ys[hottest_idx], zs[hottest_idx], s=140, marker='^', label='Max PPM voxel')
    ax.scatter(xs[coldest_idx], ys[coldest_idx], zs[coldest_idx], s=100, marker='x', label='Min PPM voxel')

    ax.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=220)

    print(f'3D scatter kaydedildi: {out_path}')
    print(f'Toplam voxel sayısı: {len(voxels)}')
    print(f'Min avg_ppm: {ppms.min():.6f}')
    print(f'Max avg_ppm: {ppms.max():.6f}')

    plt.show()


if __name__ == '__main__':
    main()
