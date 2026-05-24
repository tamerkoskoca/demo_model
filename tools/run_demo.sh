#!/bin/bash
# BURKUT Demo - Tüm sistemi başlatır

echo "Sim başlatılıyor..."
cd ~/burkut-sim
./tools/run_sim.sh &

echo "Sim açılması bekleniyor (15 saniye)..."
sleep 15

echo "Perception node'ları başlatılıyor..."
source ~/burkut-sim/install/setup.bash
ros2 run burkut_perception lidar_perception &
ros2 run burkut_perception yolo_perception &

echo "Dashboard açılıyor..."
sleep 2
ros2 run burkut_perception perception_dashboard
