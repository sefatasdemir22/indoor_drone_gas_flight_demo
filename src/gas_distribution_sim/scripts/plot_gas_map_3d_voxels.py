#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import hashlib
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize


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


def load_samples_csv(csv_path):
    samples = []

    if not os.path.exists(csv_path):
        return samples

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


def expand_limits(values, padding_ratio=0.10, min_padding=0.5):
    vmin = min(values)
    vmax = max(values)

    if math.isclose(vmin, vmax):
        return vmin - min_padding, vmax + min_padding

    span = vmax - vmin
    pad = max(span * padding_ratio, min_padding)
    return vmin - pad, vmax + pad


def stable_seed(*parts):
    raw = "|".join(str(p) for p in parts).encode("utf-8")
    digest = hashlib.sha256(raw).hexdigest()[:16]
    return int(digest, 16) % (2**32 - 1)


def build_smoke_points(voxels, resolution=0.5):
    """
    Her voxel'i, içinde dağılmış yarı saydam çok sayıda nokta ile temsil eder.
    Bu sayede hacim / duman hissi oluşur.
    """
    ppms = np.array([v['avg_ppm'] for v in voxels], dtype=float)
    pmin = float(np.min(ppms))
    pmax = float(np.max(ppms))

    if math.isclose(pmin, pmax):
        norm_vals = np.ones_like(ppms)
    else:
        norm_vals = (ppms - pmin) / (pmax - pmin)

    smoke_x = []
    smoke_y = []
    smoke_z = []
    smoke_c = []
    smoke_s = []
    smoke_a = []

    for v, nv in zip(voxels, norm_vals):
        cx = v['center_x']
        cy = v['center_y']
        cz = v['center_z']
        ppm = v['avg_ppm']
        count = v['sample_count']

        # Daha yüksek ppm = daha yoğun bulut
        n_points = int(25 + 110 * nv + 4 * math.sqrt(max(count, 1)))
        spread = resolution * (0.55 + 0.25 * nv)

        rng = np.random.default_rng(
            stable_seed(v['voxel_ix'], v['voxel_iy'], v['voxel_iz'], round(ppm, 6))
        )

        # Küp içinde değil, merkeze daha yakın gaussian dağılım → daha bulut hissi
        px = rng.normal(loc=cx, scale=spread / 3.2, size=n_points)
        py = rng.normal(loc=cy, scale=spread / 3.2, size=n_points)
        pz = rng.normal(loc=cz, scale=spread / 3.2, size=n_points)

        # Çok taşanları kırp
        half = resolution * 0.55
        px = np.clip(px, cx - half, cx + half)
        py = np.clip(py, cy - half, cy + half)
        pz = np.clip(pz, cz - half, cz + half)

        size = 18.0 + 26.0 * nv
        alpha = 0.05 + 0.18 * nv

        smoke_x.extend(px.tolist())
        smoke_y.extend(py.tolist())
        smoke_z.extend(pz.tolist())
        smoke_c.extend([ppm] * n_points)
        smoke_s.extend([size] * n_points)
        smoke_a.extend([alpha] * n_points)

    return (
        np.array(smoke_x, dtype=float),
        np.array(smoke_y, dtype=float),
        np.array(smoke_z, dtype=float),
        np.array(smoke_c, dtype=float),
        np.array(smoke_s, dtype=float),
        np.array(smoke_a, dtype=float),
    )


def main():
    voxel_csv = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_voxel.csv')
    samples_csv = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_samples_3d.csv')
    out_dir = os.path.expanduser('~/araswarm_ws/gas_map_logs')
    out_path = os.path.join(out_dir, 'gas_map_3d_wow.png')

    os.makedirs(out_dir, exist_ok=True)

    voxels = load_voxel_csv(voxel_csv)
    if not voxels:
        print('Voxel CSV boş. Önce 3D mapper veri toplamalı.')
        return

    samples = load_samples_csv(samples_csv)

    xs = np.array([v['center_x'] for v in voxels], dtype=float)
    ys = np.array([v['center_y'] for v in voxels], dtype=float)
    zs = np.array([v['center_z'] for v in voxels], dtype=float)
    ppms = np.array([v['avg_ppm'] for v in voxels], dtype=float)

    resolution = 0.5
    sx, sy, sz, sc, ss, sa = build_smoke_points(voxels, resolution=resolution)

    norm = Normalize(vmin=float(np.min(ppms)), vmax=float(np.max(ppms)))
    cmap = cm.get_cmap('viridis')
    smoke_colors = cmap(norm(sc))
    smoke_colors[:, 3] = sa  # alpha'yı doğrudan renk matrisine işle

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Duman / hacim hissi
    ax.scatter(
        sx, sy, sz,
        c=smoke_colors,
        s=ss,
        depthshade=False
    )

    # Voxel merkezlerini hafifçe de işaretle
    ax.scatter(
        xs, ys, zs,
        c=ppms,
        s=22,
        alpha=0.35
    )

    # Drone trajectory
    if samples:
        tx = np.array([s['x'] for s in samples], dtype=float)
        ty = np.array([s['y'] for s in samples], dtype=float)
        tz = np.array([s['z'] for s in samples], dtype=float)

        ax.plot(
            tx, ty, tz,
            linewidth=2.5,
            alpha=0.95,
            label='Drone trajectory'
        )

        # Zemine izdüşüm gölge
        floor_z = max(0.0, float(np.min(zs)) - 0.15)
        ax.plot(
            tx, ty, np.full_like(tz, floor_z),
            linewidth=1.4,
            alpha=0.35,
            linestyle='--',
            label='Trajectory shadow'
        )

        ax.scatter([tx[0]], [ty[0]], [tz[0]], s=120, marker='o', label='Start')
        ax.scatter([tx[-1]], [ty[-1]], [tz[-1]], s=140, marker='x', label='End')

    hottest_idx = int(np.argmax(ppms))
    ax.scatter(
        [xs[hottest_idx]],
        [ys[hottest_idx]],
        [zs[hottest_idx]],
        s=220,
        marker='*',
        label='Max PPM zone'
    )

    mappable = cm.ScalarMappable(norm=norm, cmap=cmap)
    mappable.set_array([])
    cbar = plt.colorbar(mappable, ax=ax, pad=0.08, shrink=0.82)
    cbar.set_label('Average PPM')

    ax.set_title('3D Gas Distribution Map')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    all_x = xs.tolist()
    all_y = ys.tolist()
    all_z = zs.tolist()

    if samples:
        all_x += tx.tolist()
        all_y += ty.tolist()
        all_z += tz.tolist()

    x_min, x_max = expand_limits(all_x, padding_ratio=0.12, min_padding=0.8)
    y_min, y_max = expand_limits(all_y, padding_ratio=0.12, min_padding=0.8)
    z_min, z_max = expand_limits(all_z, padding_ratio=0.15, min_padding=0.6)

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(max(0.0, z_min), z_max)

    # Daha dramatik açı
    ax.view_init(elev=22, azim=-64)

    # Eksen oranlarını veri dağılımına göre ayarla
    try:
        ax.set_box_aspect((
            max(x_max - x_min, 1.0),
            max(y_max - y_min, 1.0),
            max(z_max - z_min, 1.0)
        ))
    except Exception:
        pass

    ax.legend(loc='upper right')
    plt.tight_layout()
    plt.savefig(out_path, dpi=240)

    print(f'WOW görsel kaydedildi: {out_path}')
    print(f'Toplam voxel sayısı: {len(voxels)}')
    print(f'Toplam sample sayısı: {len(samples)}')
    print(f'Min avg_ppm: {ppms.min():.6f}')
    print(f'Max avg_ppm: {ppms.max():.6f}')

    plt.show()


if __name__ == '__main__':
    main()
