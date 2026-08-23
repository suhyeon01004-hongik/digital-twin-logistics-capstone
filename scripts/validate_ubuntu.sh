#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_root}"

source /opt/ros/humble/setup.bash
source scripts/milemate_env.sh

python3 scripts/validate_workspace.py
python3 scripts/validate_protocols.py
python3 scripts/validate_matlab_static.py
python3 scripts/validate_runtime_assets.py
python3 scripts/validate_evidence.py
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v

rosdep install --from-paths ros2_ws/src --ignore-src -r -y
colcon --log-base ros2_ws/log build \
  --base-paths ros2_ws/src \
  --build-base ros2_ws/build \
  --install-base ros2_ws/install \
  --symlink-install
