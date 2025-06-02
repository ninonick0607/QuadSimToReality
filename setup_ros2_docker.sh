#!/bin/bash
set -e

CONTAINER_NAME=ros2dev
WORKSPACE_DIR=~/ros2_ws
DOCKER_ALIAS_SCRIPT=~/.ros2dev_docker_start.sh

echo "[1/6] Installing Docker..."
sudo apt update
sudo apt install -y docker.io

echo "[2/6] Adding current user to 'docker' group..."
if groups $USER | grep -q '\bdocker\b'; then
  echo "→ User already in docker group."
else
  sudo usermod -aG docker $USER
  echo "→ Added '$USER' to docker group. You must log out and back in before continuing."
fi

echo "[3/6] Creating workspace at $WORKSPACE_DIR..."
mkdir -p "$WORKSPACE_DIR/src"

echo "[4/6] Writing Docker launch script to $DOCKER_ALIAS_SCRIPT..."
cat <<EOF > "$DOCKER_ALIAS_SCRIPT"
#!/bin/bash
xhost +local:docker > /dev/null

docker run -it --rm \\
  --net=host \\
  --privileged \\
  -e DISPLAY=\$DISPLAY \\
  -e XDG_RUNTIME_DIR=\$XDG_RUNTIME_DIR \\
  -e QT_X11_NO_MITSHM=1 \\
  -v /tmp/.X11-unix:/tmp/.X11-unix \\
  -v $WORKSPACE_DIR:/root/ros2_ws \\
  osrf/ros:humble-desktop bash
EOF

chmod +x "$DOCKER_ALIAS_SCRIPT"

echo "[5/6] Adding 'ros2dev' alias to ~/.bashrc if missing..."
if ! grep -q "alias ros2dev=" ~/.bashrc; then
  echo "alias ros2dev='$DOCKER_ALIAS_SCRIPT'" >> ~/.bashrc
  echo "→ Added alias: 'ros2dev'"
fi

echo "[6/6] Pulling ROS image — may fail if Docker group not yet active..."
if groups $USER | grep -q '\bdocker\b'; then
  docker pull osrf/ros:humble-desktop
else
  echo "⚠️  Skipping 'docker pull' — log out and back in, then run 'ros2dev' to pull and launch ROS 2 container."
fi

echo ""
echo "✅ Setup complete."
echo "➡️  Please log out and log back in to activate Docker group permissions."
echo "➡️  Then run: ros2dev"
