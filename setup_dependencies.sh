#!/bin/bash
# Simple setup script for QuadSimToReality on Ubuntu 24.04
# Installs all necessary dependencies for the ROS2 branch
set -e  # Exit on error

echo "===== QuadSimToReality Dependencies Setup ====="

# Check if running with sudo
if [ "$EUID" -ne 0 ]; then
    echo "Please run this script with sudo"
    exit 1
fi

# Update package lists
echo "[+] Updating package lists..."
apt-get update

# Install basic dev dependencies
echo "[+] Installing basic dev dependencies..."
apt-get install -y build-essential cmake git python3-pip python3-dev curl gnupg2 lsb-release

# Install ROS2 Jazzy
echo "[+] Setting up ROS2 Jazzy..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
apt-get update
echo "[+] Installing ROS2 Jazzy..."
apt-get install -y ros-jazzy-desktop ros-jazzy-rmw-cyclonedds-cpp python3-colcon-common-extensions

# Install additional ROS2 packages for visualization
echo "[+] Installing ROS2 visualization packages..."
apt-get install -y ros-jazzy-cv-bridge ros-jazzy-rviz2 ros-jazzy-vision-msgs ros-jazzy-image-transport

# Install ZeroMQ (for compatibility with older code)
echo "[+] Installing ZeroMQ..."
apt-get install -y libzmq3-dev

# Install Python packages for simulation and visualization
echo "[+] Installing Python packages..."
pip3 install gymnasium numpy stable-baselines3 opencv-python zmq

# Add ROS2 setup to user's bashrc if not already there
USER_HOME=$(eval echo ~${SUDO_USER})
if ! grep -q "source /opt/ros/jazzy/setup.bash" $USER_HOME/.bashrc; then
    echo "[+] Adding ROS2 setup to bashrc..."
    echo "" >> $USER_HOME/.bashrc
    echo "# ROS2 Jazzy" >> $USER_HOME/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> $USER_HOME/.bashrc
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $USER_HOME/.bashrc
fi

echo "===== Dependencies installation complete! ====="
echo "Please log out and log back in, or run 'source ~/.bashrc'"
echo "to update your environment variables."
echo ""
echo "IMPORTANT: You must source the ROS2 setup script before running any ROS2 code:"
echo "    source /opt/ros/jazzy/setup.bash"
