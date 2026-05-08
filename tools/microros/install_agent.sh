#!/usr/bin/env bash
# Install the micro-ROS Agent matching our Jazzy-aligned client (libmicroros)
# from the official ROS 2 Jazzy apt repo. Ubuntu 24.04 host required.
#
# Idempotent: skips if already installed.
set -euo pipefail

if dpkg -s ros-jazzy-micro-ros-agent >/dev/null 2>&1; then
    echo "ros-jazzy-micro-ros-agent already installed."
    exit 0
fi

. /etc/os-release
if [[ "$VERSION_CODENAME" != "noble" ]]; then
    echo "ERROR: ROS 2 Jazzy targets Ubuntu 24.04 (noble); host is $VERSION_CODENAME." >&2
    exit 1
fi

if ! grep -q "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list.d/*.list 2>/dev/null; then
    echo "Adding ROS 2 apt repo..."
    sudo apt-get update
    sudo apt-get install -y curl gnupg lsb-release software-properties-common
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
fi

sudo apt-get update
sudo apt-get install -y ros-jazzy-micro-ros-agent

echo
echo "Done. Source the agent's setup before running it:"
echo "    source /opt/ros/jazzy/setup.bash"
