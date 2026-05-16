#!/bin/bash

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="$REPO_ROOT/src/gas_distribution_sim/scripts"

GAS_SENSOR_SCRIPT="$SCRIPTS_DIR/gas_sensor_node.py"
GAS_MAPPER_SCRIPT="$SCRIPTS_DIR/gas_mapper_3d_node.py"
SAFE_SCAN_SCRIPT="$SCRIPTS_DIR/safe_scan_flight.py"
EXPERIMENT_SUMMARY_SCRIPT="$SCRIPTS_DIR/experiment_summary.py"

VOXEL_CSV="$HOME/araswarm_ws/gas_map_logs/gas_map_voxel.csv"
SAMPLES_CSV="$HOME/araswarm_ws/gas_map_logs/gas_map_samples_3d.csv"
SUMMARY_TXT="$HOME/araswarm_ws/gas_map_logs/experiment_summary.txt"

ROS_SETUP="source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; source ~/araswarm_ws/install/setup.bash"

log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

check_file() {
    local path="$1"
    local label="$2"

    if [[ -s "$path" ]]; then
        log "OK: $label oluşturuldu: $path"
        return 0
    fi

    if [[ -e "$path" ]]; then
        log "UYARI: $label var ama boş görünüyor: $path"
        return 1
    fi

    log "HATA: $label bulunamadı: $path"
    return 1
}

open_tab() {
    local title="$1"
    local command="$2"

    gnome-terminal --tab --title="$title" -- bash -lc "$command; exec bash"
}

log "Eski PX4, Gazebo, MicroXRCEAgent ve proje Python süreçleri temizleniyor..."
killall -9 px4 gzserver gzclient MicroXRCEAgent 2>/dev/null || true
pkill -9 -f "safe_scan_flight.py" 2>/dev/null || true
pkill -9 -f "gas_sensor_node.py" 2>/dev/null || true
pkill -9 -f "gas_mapper_3d_node.py" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 2

log "Eski deney çıktıları temizleniyor..."
rm -f "$VOXEL_CSV" "$SAMPLES_CSV" "$SUMMARY_TXT"

if ! command -v gnome-terminal >/dev/null 2>&1; then
    log "HATA: gnome-terminal bulunamadı. Demo tabları açılamıyor."
    exit 1
fi

for script in "$REPO_ROOT/baslat.sh" "$GAS_SENSOR_SCRIPT" "$GAS_MAPPER_SCRIPT" "$SAFE_SCAN_SCRIPT"; do
    if [[ ! -f "$script" ]]; then
        log "HATA: Gerekli script bulunamadı: $script"
        exit 1
    fi
done

log "ROS ortamı hazırlanıyor..."
eval "$ROS_SETUP"

log "Ana simülasyon başlatılıyor: bash baslat.sh"
cd "$REPO_ROOT" || exit 1
bash "$REPO_ROOT/baslat.sh"

log "PX4 ve Gazebo'nun stabil başlaması bekleniyor..."
sleep 4

log "Gaz sensörü yeni gnome-terminal tabında başlatılıyor..."
open_tab "Gas Sensor" "cd '$REPO_ROOT'; $ROS_SETUP; python3 '$GAS_SENSOR_SCRIPT'"

sleep 1

log "3B gaz mapper yeni gnome-terminal tabında başlatılıyor..."
open_tab "Gas Mapper 3D" "cd '$REPO_ROOT'; $ROS_SETUP; python3 '$GAS_MAPPER_SCRIPT'"

log "Mapper'ın abonelikleri ve CSV dosyalarını hazırlaması bekleniyor..."
sleep 1

log "Güvenli tarama uçuşu başlatılıyor..."
cd "$REPO_ROOT" || exit 1
python3 "$SAFE_SCAN_SCRIPT"
SAFE_SCAN_EXIT_CODE=$?

log "Uçuş komutu tamamlandı. Mapper'ın son CSV yazımı için kısa bekleme..."
sleep 8

log "Demo çıktı dosyaları kontrol ediliyor..."
check_file "$VOXEL_CSV" "gas_map_voxel.csv"
VOXEL_STATUS=$?

check_file "$SAMPLES_CSV" "gas_map_samples_3d.csv"
SAMPLES_STATUS=$?

if [[ $SAFE_SCAN_EXIT_CODE -ne 0 ]]; then
    log "HATA: safe_scan_flight.py hata kodu ile bitti: $SAFE_SCAN_EXIT_CODE"
    exit "$SAFE_SCAN_EXIT_CODE"
fi

if [[ $VOXEL_STATUS -ne 0 || $SAMPLES_STATUS -ne 0 ]]; then
    log "HATA: Demo tamamlandı ancak beklenen CSV çıktılarında eksik/boş dosya var."
    exit 1
fi

if [[ -f "$EXPERIMENT_SUMMARY_SCRIPT" ]]; then
    log "Deney özeti üretiliyor..."
    python3 "$EXPERIMENT_SUMMARY_SCRIPT"
else
    log "UYARI: experiment_summary.py bulunamadı, deney özeti atlanıyor."
fi

log "Demo başarıyla tamamlandı."
