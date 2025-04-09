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
- **Operating System**: Windows 11
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



