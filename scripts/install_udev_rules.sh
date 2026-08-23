#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rules_source="$script_dir/99-milemate-sensors.rules"
rules_target="/etc/udev/rules.d/99-milemate-sensors.rules"

if [[ ! -f "$rules_source" ]]; then
  echo "udev rule not found: $rules_source" >&2
  exit 1
fi

sudo usermod -aG dialout "$USER"
sudo usermod -aG video "$USER"
sudo install -m 0644 "$rules_source" "$rules_target"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed $rules_target"
echo "Reconnect devices and sign out/in once for group changes to take effect."
