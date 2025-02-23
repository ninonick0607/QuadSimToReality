import gymnasium as gym
import numpy as np
import zmq
import time
import cv2  
import threading
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
import os

os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'

import tensorboard
import tensorflow
class QuadSimEnv(gym.Env):
    def __init__(self):
        super(QuadSimEnv, self).__init__()  

        # Modify action space to only control Z
        self.action_space = gym.spaces.Box(
            low=np.array([-1]),  # Just Z control
            high=np.array([1]),
            dtype=np.float32
        )

        # Simplified observation space for Z only (z_velocity, z_distance_to_goal)
        self.observation_space = gym.spaces.Box(
            low=np.array([-1, -1]), 
            high=np.array([1, 1]), 
            dtype=np.float32
        )

        self.state = {
            'image': np.zeros((128, 128, 3), dtype=np.uint8),
            'velocity': np.zeros(3, dtype=np.float32),
            'position': np.zeros(3, dtype=np.float32)
        }

        self.goal_state = np.zeros(3, dtype=np.float32)
        self.prev_goal_state = np.zeros(3, dtype=np.float32)
        self.prev_action = 0.0  # Initialize prev_action
        self.prev_velocity = 0.0  # Also initialize prev_velocity while we're at it
        self.context = zmq.Context()

        # Subscriber socket for receiving images
        self.image_socket = self.context.socket(zmq.SUB)
        self.image_socket.connect("tcp://localhost:5557")
        self.image_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        # Publisher socket for sending velocity commands / reset command
        self.command_socket = self.context.socket(zmq.PUB)
        self.command_socket.bind("tcp://*:5556")  

        # Subscriber socket for receiving state
        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.connect("tcp://localhost:5558")
        self.control_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        self.steps = 0
        time.sleep(0.1)

    def get_observation(self):
        z_vel = self.state['velocity'][2] / 150.0  # Normalize velocity
        z_to_goal = (self.goal_state[2] - self.state['position'][2]) / 5000.0  # Normalize distance
        return np.array([z_vel, z_to_goal])

    def reset(self, seed=None):
        self.send_reset_command()
        self.handle_data()
        self.steps = 0
        return self.get_observation(), {}
    
    def calculate_z_distance(self, pos1, pos2):
        return abs(pos1[2] - pos2[2])  # Only care about Z distance
    

    def step(self, action):

        self.prev_action = 0.7 * self.prev_action + 0.3 * action[0]
        full_action = np.array([0.0, 0.0, self.prev_action]) * 250.0  # Reduced from 250
        
        for i in range(64):
            self.send_velocity_command(full_action)

        self.handle_data()

        # Calculate reward components
        current_z = self.state['position'][2]  # In centimeters
        z_velocity = self.state['velocity'][2]  # In cm/s
        z_distance = self.calculate_z_distance(self.state['position'], self.goal_state)
        
        reward = 0.0
        
        # Goal-reaching reward - INCREASED WEIGHT
        distance_reward = -np.tanh(z_distance / 1000.0) 
        reward += distance_reward * 2.0 

        # Height maintenance reward - INCREASED PENALTY
        min_safe_height = 100.0  # 1 meter in cm
        if current_z < min_safe_height:
            height_penalty = -2.0 * (min_safe_height - current_z) / min_safe_height  # Increased from -2.0
            reward += height_penalty
        
        # Efficient movement reward - KEEP AS IS since it's binary
        if z_distance > 0:
            correct_direction = np.sign(z_velocity) == np.sign(self.goal_state[2] - current_z)
            reward += 1.0 if correct_direction else -1.0
        
        # Velocity control rewards - INCREASED PENALTIES
        near_target = z_distance < 100.0  # 1 meter in cm
        if near_target:
            hover_penalty = -abs(z_velocity) * 2.0  # Increased from 0.5
            reward += hover_penalty
        else:
            optimal_velocity = np.clip(z_distance * 0.2, -500, 500)
            velocity_efficiency = -abs(z_velocity - optimal_velocity) * 0.3  # Increased from 0.1
            reward += velocity_efficiency
                
        self.prev_velocity = z_velocity

        # Energy efficiency - INCREASED PENALTY
        energy_penalty = -abs(z_velocity) * 0.01
        reward += energy_penalty
   
        done = self.steps >= 512
        info = {}
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
        info = {}
        self.steps += 1
        
        if done:
            observation, _ = self.reset(seed=None)
        else: 
            observation = self.get_observation()

        return observation, reward, done, False, info

    def send_velocity_command(self, velocity):
        command_topic = "VELOCITY"
        message = np.array(velocity, dtype=np.float32).tobytes()
        print(f"[Python] Sending velocity command: {velocity}")

        self.command_socket.send_multipart([command_topic.encode(), message])
 
    def send_reset_command(self):
        command_topic = "RESET"
        self.prev_goal_state = self.goal_state.copy()
        self.command_socket.send_string(command_topic)
        time.sleep(0.1)

    def handle_data(self):
        try:
            # Add timeout to prevent blocking
            if self.control_socket.poll(100, zmq.POLLIN):
                unified_data = self.control_socket.recv_string()
                data_parts = unified_data.split(";")
                
                # Add validation
                if len(data_parts) != 3:
                    raise ValueError("Invalid data format")
                    
                parsed_data = {}
                for part in data_parts:
                    key, values = part.split(":")
                    parsed_data[key] = list(map(float, values.split(",")))
                
                # Validate vector lengths
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
            # Implement recovery logic
            self.reset()
        
    def get_data(self):
        try:
            [topic, message] = self.image_socket.recv_multipart(flags=zmq.NOBLOCK)
            drone_id = topic.decode()
            image_data = np.frombuffer(message, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return image if image is not None else None
        except (zmq.Again, Exception):
            return None
     
def display_captured_images():
    # Create an instance of your environment
    env = QuadSimEnv()
    
    # Optionally, initialize the environment if needed
    # obs, _ = env.reset()

    # Create a window for displaying images
    cv2.namedWindow("Drone Capture", cv2.WINDOW_NORMAL)

    frame_count = 0
    start_time = time.time()
    fps = 0

    while True:
        # Retrieve the latest captured image
        image = env.get_data()

        # Only proceed if we have a valid image
        if image is not None:
            frame_count += 1
            current_time = time.time()
            elapsed = current_time - start_time

            # Update FPS every second
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start_time = current_time

            # Overlay FPS info on the image
            cv2.putText(image, f"FPS: {fps:.2f}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # You can overlay more information here if needed

            # Display the image in the window
            cv2.imshow("Drone Capture", image)

        # Check if the user pressed 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up the window
    cv2.destroyAllWindows()
  
if __name__ == "__main__":

    # display_thread = threading.Thread(target=display_captured_images, daemon=True)
    # display_thread.start()


    rl_dir = "./RL_training"
    checkpoints_dir = os.path.join(rl_dir, "checkpoints")
    best_model_dir = os.path.join(rl_dir, "best_model")
    logs_dir = os.path.join(rl_dir, "logs")
    
    os.makedirs(checkpoints_dir, exist_ok=True)
    os.makedirs(best_model_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)
    

    # Create environment
    env = Monitor(QuadSimEnv(), logs_dir)
    
    # Evaluation callback - saves when performance improves
    eval_callback = EvalCallback(
        env,
        best_model_save_path=best_model_dir,
        log_path=logs_dir,
        eval_freq=5000,  # Evaluate every 5000 steps
        deterministic=True,
        render=False
    )

    # Checkpoint callback - saves periodically
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,  # Save every 5000 steps
        save_path=checkpoints_dir,
        name_prefix="quad_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
        verbose=1
    )

    model = PPO(
        "MlpPolicy", 
        env,
        policy_kwargs=dict(net_arch=[64,64]),  # Simpler network
        learning_rate=3e-4,  # Standard RL baseline rate
        n_steps=2048,        # Longer rollout
        gamma=0.99,
        verbose=1,
        tensorboard_log=logs_dir
    )

    model.learn(
        total_timesteps=512 * 1000,
        callback=[checkpoint_callback, eval_callback]
    )
   

'''
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=0.003,
        n_steps=512, 
        tensorboard_log=logs_dir
    )



if __name__ == "__main__":
    rl_dir = "./RL_training"
    checkpoint_dir = os.path.join(rl_dir,"checkpoints")
    best_model_dir = os.path.join(rl_dir, "best_model")
    
    # Create environment
    env = QuadSimEnv()
    
    # Load the best model
    best_model_path = os.path.join(best_model_dir, "best_model")
    checkpoint_model_path = os.path.join(checkpoint_dir,"quad_model_170000_steps")
    if os.path.exists(checkpoint_model_path + ".zip"):
        print(f"Loading best model from {checkpoint_model_path}")
        model = PPO.load(checkpoint_model_path, env=env)
        
        # Run the model
        obs, _ = env.reset()
        while True:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, truncated, info = env.step(action)
            if done:
                obs, _ = env.reset()
    else:
        print(f"No model found at {checkpoint_model_path}")
   
'''     
