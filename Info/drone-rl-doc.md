# Reinforcement Learning for Quadcopter Altitude Control

## Overview

This document explains a reinforcement learning (RL) system designed to control a quadcopter drone's altitude in a simulated environment. The system uses Proximal Policy Optimization (PPO) to train an agent that learns to maintain a specified target altitude.

## System Architecture

The system consists of three main components:

1. **Unreal Engine Simulator**: A physics-based drone simulator built in Unreal Engine
2. **ZMQ Communication Layer**: Middleware for exchanging data between Python and Unreal Engine
3. **RL Agent**: A PPO-based agent implemented with Stable Baselines 3

### Communication Flow
```
┌─────────────────┐        ┌─────────────────┐        ┌─────────────────┐
│                 │        │                 │        │                 │
│  Unreal Engine  │◄─────► │  ZMQ Interface  │◄─────► │   RL Agent      │
│    Simulator    │        │                 │        │   (Python)      │
│                 │        │                 │        │                 │
└─────────────────┘        └─────────────────┘        └─────────────────┘
```

## Environment Details

### `QuadSimEnv` Class
This class implements the Gymnasium environment interface, allowing it to work with Stable Baselines 3 RL algorithms.

### Action Space

```python
self.action_space = gym.spaces.Box(
    low=np.array([-1]),  # Just Z control
    high=np.array([1]),
    dtype=np.float32
)
```

The action space is a continuous 1-dimensional space (a single value between -1 and 1):
- **Z-axis control only**: The agent controls only the vertical velocity of the drone
- **Value range**: -1.0 to 1.0, where:
  - Positive values: Move upward
  - Negative values: Move downward
  - Magnitude: Controls the intensity of movement

### Observation Space

```python
self.observation_space = gym.spaces.Box(
    low=np.array([-1, -1]), 
    high=np.array([1, 1]), 
    dtype=np.float32
)
```

The observation space consists of two normalized values:
1. **Normalized Z-velocity** (`z_vel`): The drone's current vertical velocity divided by 100.0
   - This scaling makes the velocity values more manageable for neural networks
2. **Normalized Z-distance to goal** (`z_to_goal`): The vertical distance to the target altitude divided by 2000.0
   - Positive values: Need to move upward to reach target
   - Negative values: Need to move downward to reach target

### State Management

The environment maintains several state variables:
- `state`: Dictionary containing:
  - `image`: Camera feed from the drone (128x128 RGB)
  - `velocity`: 3D velocity vector [vx, vy, vz]
  - `position`: 3D position vector [x, y, z]
- `goal_state`: Target 3D position
- `prev_goal_state`: Previous target position
- `prev_action`: For action smoothing
- `prev_velocity`: Previous velocity for velocity change calculations
- `steps`: Step counter for episode termination

### ZMQ Communication

The environment uses ZeroMQ (ZMQ) for bidirectional communication with the Unreal Engine simulator:

1. **Image Socket (`image_socket`)**: Receives camera feed from the drone
   - Port: 5557
   - Subscribe to all messages ('')

2. **Command Socket (`command_socket`)**: Sends velocity commands and reset signals
   - Port: 5556
   - Commands:
     - "VELOCITY": Followed by a 3D float array [vx, vy, vz]
     - "RESET": Resets the drone position

3. **Control Socket (`control_socket`)**: Receives drone state information
   - Port: 5558
   - Format: "VELOCITY:x,y,z;POSITION:x,y,z;GOAL:x,y,z"

## Reward Structure

The reward function is designed to teach the drone to reach and maintain a target altitude of 1000cm. It has three components:

### 1. Distance Penalty
```python
distance_penalty = -z_distance / 100.0
reward += distance_penalty
```
- **Purpose**: Encourages the drone to minimize distance to the target altitude
- **Scaling**: Dividing by 100.0 keeps the penalty manageable (e.g., 1000cm distance = -10.0 reward)
- **Linear Scaling**: The penalty increases linearly with distance

### 2. Stability Bonus/Penalty
```python
if z_distance < 50.0:  # When close to target
    stability_factor = -abs(z_velocity) * 0.1
    reward += stability_factor
```
- **Purpose**: Encourages the drone to stabilize (minimize velocity) when near the target
- **Activation**: Only applies when within 50cm of target altitude
- **Effect**: Penalizes high velocities when close to target, preventing oscillation

### 3. Target Achievement Bonus
```python
if z_distance < 20.0:
    target_bonus = 10.0
    reward += target_bonus
```
- **Purpose**: Provides a large positive reward for reaching and maintaining the target
- **Threshold**: 20cm from target altitude
- **Magnitude**: +10.0 (significant compared to other reward components)

### Reward Balance
The reward structure creates a gradient that guides the drone to:
1. First move toward the target (minimize distance)
2. Then slow down as it approaches (stability)
3. Finally maintain position at the target (achievement bonus)

## Control Flow

### Action Processing
```python
self.prev_action = 0.7 * self.prev_action + 0.3 * action[0]
full_action = np.array([0.0, 0.0, self.prev_action]) * 250.0
```
- **Action Smoothing**: Uses exponential weighted average of current and previous actions (70%/30% split)
- **Scaling**: Multiplies by 250.0 to convert normalized actions to physical velocity values
- **3D Vector**: Creates a 3D velocity vector with only Z component (x and y set to 0)

### Simulation Steps
```python
for i in range(64):
    self.send_velocity_command(full_action)
```
- **Simulation Frequency**: Sends the command 64 times to ensure stability
- This emulates a control loop running at a higher frequency than the agent

### Episode Termination
```python
done = self.steps >= 512
```
- **Episode Length**: 512 steps per episode
- This gives the agent sufficient time to reach and stabilize at the target

## Inference (Deployment) Code

The main block loads a trained model and runs it:
```python
best_model_path = "./RL_training/checkpoints/quad_model_120000_steps.zip"
env = QuadSimEnv()
model = PPO.load(best_model_path)

# Run the model
obs, _ = env.reset()
while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    if done or truncated:
        obs, _ = env.reset()
```

- **Model Loading**: Loads a previously trained PPO model (120,000 steps)
- **Deterministic Policy**: Uses deterministic actions for deployment
- **Continuous Operation**: Keeps running the agent, resetting when episodes end

## Technical Implementation Details

### Action Smoothing
The action smoothing technique (70% previous, 30% current) serves several purposes:
1. Reduces jerky movements that could destabilize the drone
2. Provides momentum to the control signal
3. Helps account for the difference between simulation and control frequencies

### Normalization Factors
- Velocity divided by 100.0: Makes typical drone velocities (±50-100 cm/s) map to normalized range (±0.5-1.0)
- Distance divided by 2000.0: Makes reasonable distances (0-2000cm) map to normalized range (0-1.0)

### State Handling
- The environment catches and handles communication errors by resetting
- It extracts position, velocity and goal information from formatted state messages
