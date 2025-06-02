#!/bin/bash
set -e

echo "[1/4] Updating and upgrading system..."
sudo apt update && sudo apt upgrade -y

echo "[2/4] Installing system packages..."
sudo apt install -y \
  curl gnupg lsb-release build-essential git wget \
  python3-pip python3-dev python3.12-venv \
  libbullet-dev libopencv-dev cmake ninja-build

echo "[3/4] Creating and activating virtual environment..."
python3 -m venv ~/myproject_venv
source ~/myproject_venv/bin/activate

echo "[4/4] Installing Python dependencies (Gymnasium, SB3, OpenCV, ZMQ)..."
pip install --upgrade pip setuptools wheel
pip install \
  gymnasium \
  stable-baselines3[extra] \
  numpy \
  opencv-python \
  pyzmq

echo "✅ All packages installed in virtual environment at ~/myproject_venv"
echo "🧠 To activate later, run: source ~/myproject_venv/bin/activate"

