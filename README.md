# QuadSimToReality

A high-fidelity quadcopter simulation environment that bridges the gap between simulation and reality using Unreal Engine 5.

![Quadcopter Simulation](https://example.com/quad_simulation_screenshot.png)

## Overview

QuadSimToReality is a project that aims to create a realistic quadcopter simulation environment that can be used for:

- Reinforcement learning algorithm development and testing
- Drone flight controller development
- Testing control algorithms before deploying to real hardware
- Visualizing and analyzing drone behavior in complex environments

The project leverages Unreal Engine's physics system to create a physically accurate drone model that responds realistically to control inputs. It includes a complete PID control implementation, visualizations, and integration with external control systems through both ZeroMQ and ROS2.

## System Requirements

### Software Requirements
- **Operating System**: Ubuntu 22.04 LTS
- **Unreal Engine**: Version 5.5
- **Python**: Version 3.8+
- **Git LFS**: For handling large files in the repository
- **Compiler**: GCC 11+ (comes with Ubuntu 22.04)

### Hardware Requirements
- **CPU**: 4+ cores recommended
- **RAM**: 16+ GB recommended
- **GPU**: NVIDIA GTX 1070 or equivalent AMD GPU (with 4+ GB VRAM)
- **Storage**: At least 20 GB of free space

## Project Structure and Source Files

### Core Components

#### C++ Components

- `Source/QuadSimToReality/Private/Core/`
  - `DroneGlobalState.cpp` - Manages global state for the drone across the simulation
  - `DroneJSONConfig.cpp` - Loads and parses drone configuration from JSON files
  - `DroneMathUtils.cpp` - Utility math functions for drone control algorithms
  - `ThrusterComponent.cpp` - Implements the thruster physics for the drone propellers

- `Source/QuadSimToReality/Private/Controllers/`
  - `QuadDroneController.cpp` - Main drone controller implementing PID and flight modes
  - `ROS2Controller.cpp` - ROS2-based communication for external control

- `Source/QuadSimToReality/Private/Pawns/`
  - `QuadPawn.cpp` - The main quadcopter pawn class that brings together all components

- `Source/QuadSimToReality/Private/UI/`
  - `ImGuiUtil.cpp` - Implements the ImGui-based debug UI for drone parameter tuning

#### Python Components

- `quadsimenv.py` - Python environment for connecting to the simulation using gymnasium
- `setup_dependencies.sh` - Script to set up all required dependencies
- `generate_and_run.sh` - Script to generate Unreal project files and run the simulation


## Drone Configuration

The drone parameters can be customized by editing the `Config/DroneConfig.json` file. This includes parameters such as:

```json
{
    "flight_parameters": {
      "max_velocity": 450.0,
      "max_angle": 35.0,
      "max_pid_output": 700.0,
      "altitude_threshold": 0.6,
      "min_altitude_local": 500.0,
      "acceptable_distance": 200
    },
    "navigation": {
      "base_height": 1000.0,
      "spiral_radius": 1500.0
    },
    "controller": {
      "altitude_rate": 400.0,
      "yaw_rate": 90.0,
      "min_velocity_for_yaw": 10.0
    }
}
```

You can adjust these parameters to fine-tune the drone's behavior without needing to recompile the project. 

## Setup and Installation

### Prerequisites

Before starting, make sure you have:
1. Ubuntu 22.04 LTS installed
2. Git and Git LFS installed
3. Administrator (sudo) privileges on your system

### Step 1: Install Git and Git LFS

```bash
sudo apt update
sudo apt install git
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt install git-lfs
git lfs install
```

### Step 2: Clone the Repository

```bash
git clone https://github.com/yourusername/QuadSimToReality.git
cd QuadSimToReality
git lfs pull
```

### Step 3: Install Unreal Engine 5.5

1. Create an Epic Games account if you don't have one: [Epic Games](https://www.epicgames.com/)
2. Visit the [Unreal Engine for Linux](https://www.unrealengine.com/en-US/linux) page and download the prepackaged version of Unreal Engine 5.5
3. Once the download is complete, extract the archive to your home folder:

```bash
# Navigate to the download location
cd ~/Downloads

# Extract the archive to your home folder
# Note: The exact filename may differ based on the version
tar -xvzf UnrealEngine-5.5.0-linux.tar.gz -C ~/
```

4. Make the engine executable:

```bash
# Navigate to the engine directory
cd ~/UnrealEngine-5.5.0

# Make the engine binary executable if needed
chmod +x Engine/Binaries/Linux/UnrealEditor
```

This approach is much faster than building from source and is the recommended method for most users.

4. Build Unreal Engine:

```bash
./Setup.sh
./GenerateProjectFiles.sh
make
```

This process will take some time. For more detailed instructions, refer to the [official Unreal Engine documentation](https://docs.unrealengine.com/5.5/en-US/building-unreal-engine-from-source/).

### Step 4: Install Dependencies

Make the setup script executable and run it:

```bash
chmod +x setup_dependencies.sh
sudo ./setup_dependencies.sh
```

This script will install:
- ROS2 Humble
- ZeroMQ
- Python dependencies
- Other required libraries

### Step 5: Generate Project Files and Run

Make the run script executable and run it:

```bash
chmod +x generate_and_run.sh
./generate_and_run.sh
```

By default, this will launch the Unreal Editor with the FlagMap loaded. To run in standalone game mode or specify a different map, use the options described in the script help:

```bash
./generate_and_run.sh --help
```

### Step 6: Setting Up an IDE

#### Option 1: JetBrains Rider (Recommended)
1. Install JetBrains Rider:
   ```bash
   sudo snap install rider --classic
   ```
2. Open the `.code-workspace` file in the repository root
3. Configure Rider to use the Unreal Engine toolchain

#### Option 2: Visual Studio Code
1. Install VS Code:
   ```bash
   sudo snap install code --classic
   ```
2. Install the C/C++ extension and the Unreal Engine extension
3. Open the `.code-workspace` file in the repository root

#### Option 3: Other IDEs
You can use any IDE that supports C++ development. For setup instructions, refer to the IDE's documentation for Unreal Engine integration.

## Running Reinforcement Learning

To run the reinforcement learning environment:

1. Make sure you have all the Python dependencies installed:
```bash
pip install gymnasium stable-baselines3 zmq opencv-python
```

2. Launch the Unreal Engine simulation in standalone mode:
```bash
./generate_and_run.sh --no-editor
```

3. In a separate terminal, run the training script:
```bash
python quadsimenv.py
```

## Troubleshooting

### Common Issues

1. **Error: "Failed to compile Unreal Engine"**
   - Make sure you have all the required build dependencies installed
   - Check Unreal Engine's documentation for specific requirements

2. **Error: "Could not find Unreal Engine"**
   - Edit the `generate_and_run.sh` script to point to your Unreal Engine installation directory

3. **Error: "ZMQ connection failed"**
   - Ensure ZeroMQ is properly installed
   - Check for any firewall settings blocking the ports

4. **Error: "ROS2 nodes not found"**
   - Make sure ROS2 Humble is installed and sourced in your terminal
   - Check that all ROS2 dependencies are installed

### Getting Help

If you encounter issues not covered in this README, please:
1. Check the GitHub Issues tab for similar problems
2. Create a new issue with detailed information about your problem

## License
**WIP**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

