import gymnasium as gym
import numpy as np
import zmq
import time
import cv2  
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
import os

# Disable some TensorFlow optimizations (if needed)
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'
import tensorboard
import tensorflow


class QuadSimEnv(gym.Env):
    """
    An RL environment for controlling a quadrotor simulation.
    This version uses ZeroMQ to send velocity commands and receive state
    and image data. A target_drone_id (string) can be passed so that the
    environment only processes data for that drone.
    """
    def __init__(self, target_drone_id: str = None):
        super(QuadSimEnv, self).__init__()  

        # Only control the Z-axis
        self.action_space = gym.spaces.Box(
            low=np.array([-1]),  
            high=np.array([1]),
            dtype=np.float32
        )

        # Observations: normalized z velocity and z distance to goal
        self.observation_space = gym.spaces.Box(
            low=np.array([-1, -1]), 
            high=np.array([1, 1]), 
            dtype=np.float32
        )

        # Internal state dictionaries
        self.state = {
            'image': np.zeros((128, 128, 3), dtype=np.uint8),
            'velocity': np.zeros(3, dtype=np.float32),
            'position': np.zeros(3, dtype=np.float32)
        }
        self.goal_state = np.zeros(3, dtype=np.float32)
        self.prev_goal_state = np.zeros(3, dtype=np.float32)
        self.prev_action = 0.0
        self.prev_velocity = 0.0

        # ZeroMQ context and sockets
        self.context = zmq.Context()

        # Socket for receiving images (assumes topic is drone id)
        self.image_socket = self.context.socket(zmq.SUB)
        self.image_socket.connect("tcp://localhost:5557")
        # Subscribe to all topics if no target is given; else subscribe to target id
        if target_drone_id:
            self.image_socket.setsockopt_string(zmq.SUBSCRIBE, target_drone_id)
        else:
            self.image_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        # Publisher socket for sending velocity commands or reset command
        self.command_socket = self.context.socket(zmq.PUB)
        self.command_socket.bind("tcp://*:5556")  

        # Socket for receiving state/control data
        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.connect("tcp://localhost:5558")
        # Same subscription logic as above:
        if target_drone_id:
            self.control_socket.setsockopt_string(zmq.SUBSCRIBE, target_drone_id)
        else:
            self.control_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        # Optional: Store the target id for filtering messages in handle_data
        self.target_drone_id = target_drone_id

        self.steps = 0
        time.sleep(0.1)

    def get_observation(self):
        # Normalize velocity and distance values
        z_vel = self.state['velocity'][2] / 150.0  
        z_to_goal = (self.goal_state[2] - self.state['position'][2]) / 5000.0  
        return np.array([z_vel, z_to_goal])

    def reset(self, seed=None):
        self.send_reset_command()
        self.handle_data()  # Get initial state
        self.steps = 0
        return self.get_observation(), {}
    
    def calculate_z_distance(self, pos1, pos2):
        return abs(pos1[2] - pos2[2])  # Only Z difference matters

    def step(self, action):
        # Smooth the action over time
        self.prev_action = 0.7 * self.prev_action + 0.3 * action[0]
        full_action = np.array([0.0, 0.0, self.prev_action]) * 250.0

        # Send repeated velocity commands to simulate continuous control
        for i in range(64):
            self.send_velocity_command(full_action)

        self.handle_data()

        # Compute rewards based on current position, velocity, and goal
        current_z = self.state['position'][2]  
        z_velocity = self.state['velocity'][2]  
        z_distance = self.calculate_z_distance(self.state['position'], self.goal_state)
        
        reward = 0.0
        
        # Reward for approaching the goal (weighted)
        distance_reward = -np.tanh(z_distance / 1000.0) 
        reward += distance_reward * 2.0 

        # Penalty for being below a safe height
        min_safe_height = 100.0  
        if current_z < min_safe_height:
            height_penalty = -2.0 * (min_safe_height - current_z) / min_safe_height
            reward += height_penalty
        
        # Bonus/penalty based on correct directional movement
        if z_distance > 0:
            correct_direction = np.sign(z_velocity) == np.sign(self.goal_state[2] - current_z)
            reward += 1.0 if correct_direction else -1.0
        
        # Penalize large velocities near the target
        near_target = z_distance < 100.0  
        if near_target:
            hover_penalty = -abs(z_velocity) * 2.0  
            reward += hover_penalty
        else:
            optimal_velocity = np.clip(z_distance * 0.2, -500, 500)
            velocity_efficiency = -abs(z_velocity - optimal_velocity) * 0.3  
            reward += velocity_efficiency
                
        self.prev_velocity = z_velocity

        # Small penalty for energy usage
        energy_penalty = -abs(z_velocity) * 0.01
        reward += energy_penalty

        # Update steps and print status periodically
        self.steps += 1
        if self.steps % 100 == 0:
            print(f"\nStep {self.steps}:")
            print(f"Position: {self.state['position']}")
            print(f"Velocity: {self.state['velocity']}")
            print(f"Goal: {self.goal_state}")
            print(f"Reward: {reward}")
            print(f"Action: {action[0]}")
            print(f"Z Distance to Goal: {z_distance}")

        done = self.steps >= 512

        if done:
            observation, _ = self.reset(seed=None)
        else: 
            observation = self.get_observation()

        return observation, reward, done, False, {}

    def send_velocity_command(self, velocity):
        # Use a topic that optionally includes the target drone id
        command_topic = self.target_drone_id if self.target_drone_id else "VELOCITY"
        message = np.array(velocity, dtype=np.float32).tobytes()
        print(f"[Python] Sending velocity command: {velocity} (Topic: {command_topic})")
        self.command_socket.send_multipart([command_topic.encode(), message])
 
    def send_reset_command(self):
        command_topic = self.target_drone_id if self.target_drone_id else "RESET"
        self.prev_goal_state = self.goal_state.copy()
        self.command_socket.send_string(command_topic)
        time.sleep(0.1)

    def handle_data(self):
        """
        Attempts to receive a state message from the control socket.
        Expects a string formatted as:
            "<drone_id>;VELOCITY:val1,val2,val3;POSITION:val1,val2,val3;GOAL:val1,val2,val3"
        If a target_drone_id is set, only process messages that start with that id.
        """
        try:
            if self.control_socket.poll(100, zmq.POLLIN):
                unified_data = self.control_socket.recv_string()
                # Split off the topic (drone id) from the data.
                parts = unified_data.split(";", 1)
                if len(parts) != 2:
                    raise ValueError("Invalid data format; missing delimiter")
                topic, data_string = parts
                # If target is set and does not match, ignore this message.
                if self.target_drone_id and topic != self.target_drone_id:
                    return

                data_parts = data_string.split(";")
                if len(data_parts) != 3:
                    raise ValueError("Invalid data format for state")
                    
                parsed_data = {}
                for part in data_parts:
                    key, values = part.split(":")
                    parsed_data[key] = list(map(float, values.split(",")))
                
                # Ensure each expected key has 3 values
                for k in ["VELOCITY", "POSITION", "GOAL"]:
                    if len(parsed_data.get(k, [])) != 3:
                        raise ValueError(f"Invalid {k} data")
                        
                self.state.update({
                    'velocity': np.array(parsed_data["VELOCITY"]),
                    'position': np.array(parsed_data["POSITION"])
                })
                self.goal_state = np.array(parsed_data["GOAL"])
                
        except Exception as e:
            print(f"Data handling error: {str(e)}")
            self.reset()
        
    def get_data(self):
        """
        Attempts to receive an image from the image socket.
        Returns the image (as a NumPy array) if successful, otherwise None.
        """
        try:
            [topic, message] = self.image_socket.recv_multipart(flags=zmq.NOBLOCK)
            # If target_drone_id is set, verify the topic.
            if self.target_drone_id and topic.decode() != self.target_drone_id:
                return None
            image_data = np.frombuffer(message, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return image if image is not None else None
        except (zmq.Again, Exception):
            return None
   

if __name__ == "__main__":
    # Directories for logging, saving checkpoints, and best models
    rl_dir = "./RL_training"
    checkpoints_dir = os.path.join(rl_dir, "checkpoints")
    best_model_dir = os.path.join(rl_dir, "best_model")
    logs_dir = os.path.join(rl_dir, "logs")

    os.makedirs(checkpoints_dir, exist_ok=True)
    os.makedirs(best_model_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)
    

    target_drone_id = "drone_1"  # or None to not filter
    
    # Create and wrap the environment with Monitor for logging.
    env = Monitor(QuadSimEnv(target_drone_id=target_drone_id), logs_dir)
    
    # Evaluation callback: saves the best model when performance improves.
    eval_callback = EvalCallback(
        env,
        best_model_save_path=best_model_dir,
        log_path=logs_dir,
        eval_freq=5000,
        deterministic=True,
        render=False
    )

    # Checkpoint callback: periodically saves the model.
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=checkpoints_dir,
        name_prefix="quad_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
        verbose=1
    )

    # Create the PPO model with a simple MLP policy.
    model = PPO(
        "MlpPolicy", 
        env,
        policy_kwargs=dict(net_arch=[64, 64]),
        learning_rate=3e-4,
        n_steps=2048,
        gamma=0.99,
        verbose=1,
        tensorboard_log=logs_dir
    )

    # Start training for a total of 512*1000 timesteps.
    model.learn(
        total_timesteps=512 * 1000,
        callback=[checkpoint_callback, eval_callback]
    )
