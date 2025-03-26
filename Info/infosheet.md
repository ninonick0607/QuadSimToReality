# QuadSim RL Training Fact Sheet
## Observation Space
```
pythonCopyself.observation_space = gym.spaces.Box(
    low=np.array([-1, -1]), 
    high=np.array([1, 1]), 
    dtype=np.float32
)
```
- 2D observation space: [normalized_z_velocity, normalized_z_distance_to_goal]
- Range (-1 to 1) for each dimension
- Z-velocity is normalized by dividing by 150.0
- Z-distance to goal is normalized by dividing by 100,000.0

## Action Space
```
pythonCopyself.action_space = gym.spaces.Box(
    low=np.array([-1]),
    high=np.array([1]),
    dtype=np.float32
)
```

- 1D action space: only controls Z-axis velocity
- Range (-1 to 1)
- Multiplied by 250.0 to get actual velocity command
- X and Y velocities are forced to 0

## Goal State

- Changed every 512 steps (one episode)
- Set by C++ ZMQController
- Range: Z between 500-1500 units, X and Y fixed at 0
- Purpose: Gives drone a target height to reach

## Reward Structure
- pythonCopyreward = 0
- reward += z_velocity          # Encourages upward movement
- reward += current_z / 1000.0  # Rewards being high up
- reward -= z_distance / 1000.0 # Penalizes being far from goal
- reward -= horizontal_velocity * 0.1  # Small penalty for horizontal movement

## Training Parameters

Episodes: 512 steps each
Total Steps: 512,000 (1000 episodes)
Learning Rate: 0.003
Policy: MLP (Multi-Layer Perceptron)
Algorithm: PPO (Proximal Policy Optimization)

### Save Frequencies

Checkpoints: Every 5000 steps
Evaluation: Every 5000 steps
Best Model: Saved when performance improves

### Command System

Commands per step: 64
Command Type: Velocity control
Command Format: [0, 0, Z] where Z is scaled action * 250.0

### Training Objective
The drone is being trained to:

Achieve and maintain target heights
Minimize horizontal movement
Maintain stable upward control
Efficiently reach new goal heights when they change

Would you like me to expand on any of these sections?