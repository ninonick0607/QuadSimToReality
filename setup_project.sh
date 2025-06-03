#!/bin/bash
set -e

echo "[1/4] Updating package lists..."
sudo apt update

echo "[2/4] Installing system dependencies..."
#    Note: python3 and pip usually come with the ROS image, but ensure venv is installed
sudo apt install -y \
  python3-venv \
  python3-pip \
  build-essential \
  libbullet-dev \
  libopencv-dev \
  cmake \
  ninja-build

echo "[3/4] Creating and activating virtual environment..."
#    Change ~/myproject_venv to wherever you like (inside Distrobox home)
python3 -m venv ~/myproject_venv
source ~/myproject_venv/bin/activate

echo "[4/4] Installing Python packages (Gymnasium, SB3, OpenCV, ZMQ)..."
pip install --upgrade pip setuptools wheel
pip install \
  gymnasium \
  "stable-baselines3[extra]" \
  numpy \
  opencv-python \
  pyzmq

echo "✅ Inside Distrobox: Python venv set up at ~/myproject_venv"
echo "👉 To re‐activate later (inside this container), run: source ~/myproject_venv/bin/activate"

