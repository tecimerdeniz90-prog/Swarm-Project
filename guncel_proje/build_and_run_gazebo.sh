#!/bin/bash
set -e

echo "=== ROS 2 Workspace Derleme ve Gazebo Başlatma ==="

# 1. ROS 2 Ortamını Kaynaklandır (Source)
source /opt/ros/humble/setup.bash

# 2. Workspace Klasörüne Git
cd /mnt/c/Users/tecim/.gemini/antigravity-ide/scratch/multi-uav-path-planning/ros2_ws

# Colcon Kurulum Kontrolü
if ! command -v colcon &> /dev/null; then
    echo "colcon derleme aracı bulunamadı, kuruluyor..."
    sudo apt update && sudo apt install -y python3-colcon-common-extensions
fi

# 3. Colcon ile Derleme Yap
echo "Workspace derleniyor..."
colcon build --symlink-install

# 4. Derlenen Paketi Kaynaklandır (Source)
source install/setup.bash

# 5. Simülasyonu Başlat
echo "Gazebo ve Çoklu İHA Simülasyonu başlatılıyor..."
ros2 launch multi_uav_sim sim.launch.py
