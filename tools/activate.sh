#!/usr/bin/env bash
# Usage: source tools/activate.sh
set -euo pipefail

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "[activate] HATA: Bu script 'source' ile çalıştırılmalı." >&2
  echo "[activate] Kullanım: source tools/activate.sh" >&2
  exit 1
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$REPO_ROOT"

# ROS 2 Humble underlay
set +u
source /opt/ros/humble/setup.bash
set -u

# Burkut-sim workspace overlay
if [[ -f "$WS/install/setup.bash" ]]; then
  set +u
  source "$WS/install/setup.bash"
  set -u
else
  echo "[activate] WARN: Workspace henüz derlenmemiş."
  echo "[activate] Çözüm: cd '$WS' && colcon build"
fi

export BURKUT_SIM_ACTIVE=1
export BURKUT_SIM_ROOT="$REPO_ROOT"
export PX4_DIR="$HOME/PX4-Autopilot"
if [[ ! -d "$PX4_DIR" ]]; then
  echo "[activate] WARN: PX4_DIR bulunamadı: $PX4_DIR"
fi
export PATH="$PATH:$PX4_DIR/Micro-XRCE-DDS-Agent/build"
export GZ_SIM_RESOURCE_PATH="$REPO_ROOT/src/burkut_worlds/worlds:$REPO_ROOT/src/burkut_worlds/models:$REPO_ROOT/src/burkut_description/models:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
# NVIDIA PRIME: Gazebo'nun NVIDIA GPU'yu kullanması için
export __NV_PRIME_RENDER_OFFLOAD=1
export __NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
echo "[activate] OK: ROS_DISTRO=$ROS_DISTRO | ROOT=$REPO_ROOT"
