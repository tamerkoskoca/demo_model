#!/bin/bash
source ~/burkut-sim/install/setup.bash

ros2 run burkut_perception lidar_perception &
ros2 run burkut_perception yolo_perception &
ros2 run burkut_perception perception_dashboard
