#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import sys

import yaml


VOXEL_CSV = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_voxel.csv')
SAMPLES_CSV = os.path.expanduser('~/araswarm_ws/gas_map_logs/gas_map_samples_3d.csv')
SUMMARY_TXT = os.path.expanduser('~/araswarm_ws/gas_map_logs/experiment_summary.txt')
SCENARIO_YAML = os.path.expanduser(
    '~/araswarm_ws/src/araswarm_simulation/src/gas_distribution_sim/config/gas_scenarios.yaml'
)


def require_file(path, label):
    if not os.path.exists(path):
        raise FileNotFoundError(f'{label} bulunamadı: {path}')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'{label} normal dosya değil: {path}')


def read_samples(csv_path):
    samples = []
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            samples.append({
                'ros_time_sec': float(row['ros_time_sec']),
                'x': float(row['x']),
                'y': float(row['y']),
                'z': float(row['z']),
                'ppm': float(row['ppm']),
            })
    return samples


def read_voxels(csv_path):
    voxels = []
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


def infer_active_scenario(yaml_path):
    if not os.path.exists(yaml_path):
        return None

    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return None

    scenarios = data.get('scenarios', {}) or {}
    if len(scenarios) == 1:
        return next(iter(scenarios.keys()))

    return None


def build_summary(samples, voxels, active_scenario):
    total_samples = len(samples)
    filled_voxels = len(voxels)

    if total_samples == 0:
        raise ValueError('Sample CSV boş. Önce demo uçuşundan veri toplanmalı.')
    if filled_voxels == 0:
        raise ValueError('Voxel CSV boş. Önce 3D mapper veri üretmeli.')

    sample_ppms = [s['ppm'] for s in samples]
    sample_avg_ppm = sum(sample_ppms) / total_samples
    max_sample = max(samples, key=lambda s: s['ppm'])

    voxel_avg_ppms = [v['avg_ppm'] for v in voxels]
    voxel_avg_ppm = sum(voxel_avg_ppms) / filled_voxels
    max_voxel = max(voxels, key=lambda v: v['avg_ppm'])
    warn_mixed_logs = abs(max_sample['ppm'] - max_voxel['avg_ppm']) > 1.0
    path_length_3d = compute_path_length_3d(samples)
    mission_duration_sec = samples[-1]['ros_time_sec'] - samples[0]['ros_time_sec']
    sample_frequency_hz = total_samples / mission_duration_sec if mission_duration_sec > 0.0 else 0.0
    coordinate_distance = distance_3d(
        max_sample['x'],
        max_sample['y'],
        max_sample['z'],
        max_voxel['center_x'],
        max_voxel['center_y'],
        max_voxel['center_z']
    )

    lines = [
        'Deney Özeti',
        '==========',
        '',
        'Sample Bazlı İstatistikler',
        '--------------------------',
        f'Sample toplamı            : {total_samples}',
        f'Sample ortalama ppm       : {sample_avg_ppm:.3f}',
        f'Sample maksimum ppm       : {max_sample["ppm"]:.3f}',
        (
            'Sample maksimum koordinatı: '
            f'x={max_sample["x"]:.3f}, '
            f'y={max_sample["y"]:.3f}, '
            f'z={max_sample["z"]:.3f}'
        ),
        '',
        'Voxel Bazlı İstatistikler',
        '-------------------------',
        f'Dolu voxel sayısı         : {filled_voxels}',
        f'Voxel ortalama avg_ppm    : {voxel_avg_ppm:.3f}',
        f'Voxel maksimum avg_ppm    : {max_voxel["avg_ppm"]:.3f}',
        (
            'Voxel maksimum koordinatı: '
            f'x={max_voxel["center_x"]:.3f}, '
            f'y={max_voxel["center_y"]:.3f}, '
            f'z={max_voxel["center_z"]:.3f}'
        ),
        (
            'En yoğun voxel index     : '
            f'ix={max_voxel["voxel_ix"]}, '
            f'iy={max_voxel["voxel_iy"]}, '
            f'iz={max_voxel["voxel_iz"]}'
        ),
        '',
        'Görev ve Kapsama Özeti',
        '----------------------',
        f'Yaklaşık 3B yol uzunluğu : {path_length_3d:.3f} m',
        f'Yaklaşık görev süresi    : {mission_duration_sec:.3f} sn',
        f'Ortalama örnekleme frekansı: {sample_frequency_hz:.3f} Hz',
        (
            'Haritalama kapsaması     : '
            f'{filled_voxels} dolu voxel, {total_samples} toplam sample ile temsil edildi.'
        ),
    ]

    if warn_mixed_logs:
        lines.extend([
            '',
            'UYARI: Sample ve voxel istatistikleri arasında fark var. Eski loglar karışmış olabilir.'
        ])

    lines.extend([
        '',
        'Yoğunluk Tutarlılık Yorumu',
        '--------------------------',
        f'Sample-voxel maksimum koordinat uzaklığı: {coordinate_distance:.3f} m'
    ])

    if coordinate_distance <= 1.0:
        lines.append('Gaz yoğunluğu ölçümleri voxel haritası ile tutarlı görünmektedir.')
    else:
        lines.append('Sample ve voxel yoğunluk bölgeleri arasında fark vardır; loglar veya grid çözünürlüğü kontrol edilmelidir.')

    if active_scenario:
        lines.append(f'Aktif senaryo            : {active_scenario}')
    else:
        lines.append('Aktif senaryo            : bulunamadı')

    return lines


def distance_3d(x1, y1, z1, x2, y2, z2):
    dx = x1 - x2
    dy = y1 - y2
    dz = z1 - z2
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def compute_path_length_3d(samples):
    total = 0.0
    for previous, current in zip(samples, samples[1:]):
        total += distance_3d(
            previous['x'],
            previous['y'],
            previous['z'],
            current['x'],
            current['y'],
            current['z']
        )
    return total


def write_summary(lines, output_path):
    output_dir = os.path.dirname(output_path)
    os.makedirs(output_dir, exist_ok=True)

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))
        f.write('\n')


def main():
    try:
        require_file(VOXEL_CSV, 'Voxel CSV')
        require_file(SAMPLES_CSV, 'Samples CSV')

        samples = read_samples(SAMPLES_CSV)
        voxels = read_voxels(VOXEL_CSV)
        active_scenario = infer_active_scenario(SCENARIO_YAML)
        lines = build_summary(samples, voxels, active_scenario)
        write_summary(lines, SUMMARY_TXT)

    except Exception as e:
        print(f'HATA: Deney özeti üretilemedi: {e}')
        return 1

    for line in lines:
        print(line)

    print(f'\nÖzet dosyaya kaydedildi: {SUMMARY_TXT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
