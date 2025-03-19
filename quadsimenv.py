import gymnasium as gym
import numpy as np
import zmq
import time
import cv2  
import glob 
import os
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, BaseCallback
from stable_baselines3.common.monitor import Monitor

import tensorboard
# --- QuadSimEnv definition ---
class QuadSimEnv(gym.Env):
    def __init__(self):
        super(QuadSimEnv, self).__init__()  

        # Modify action space to only control Z
        self.action_space = gym.spaces.Box(
            low=np.array([-1]),  # Just Z control
            high=np.array([1]),
            dtype=np.float32
        )

        # Simplified observation space for dZ only (z_velocity, z_distance_to_goal)
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
        self.prev_action = 0.0  
        self.prev_velocity = 0.0  
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
        self.control_socket.setsockopt(zmq.CONFLATE, 1)  # Keep only the latest message
        self.control_socket.connect("tcp://localhost:5558")
        self.control_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        self.steps = 0
        time.sleep(0.1)

    def get_observation(self):
        # MODIFIED: Changed normalization scaling to be more sensitive around the operating range
        z_vel = self.state['velocity'][2] / 100.0  # More sensitive velocity normalization
        z_to_goal = (self.goal_state[2] - self.state['position'][2]) / 2000.0  # More focused distance scaling
        return np.array([z_vel, z_to_goal])

    def reset(self, seed=None):
        self.send_reset_command()
        self.handle_data()
        self.steps = 0
        return self.get_observation(), {}
    
    def calculate_z_distance(self, pos1, pos2):
        return abs(pos1[2] - pos2[2])
    
    def step(self, action):
        # Apply action and update environment (keep your existing code here)
        # self.prev_action = 0.7 * self.prev_action + 0.3 * action[0]
        self.prev_action = action[0]
        full_action = np.array([0.0, 0.0, self.prev_action]) * 250.0
        
        self.send_velocity_command(full_action)
        time.sleep(0.2) # Action frequency is ~5 Hz

        self.handle_data()

        current_z = self.state['position'][2]
        print(f"Current Z: {current_z:.2f} cm")
        target_z = 1000.0
        z_distance = abs(current_z - target_z)
        z_velocity = self.state['velocity'][2]
        print(f"Velocity: {z_velocity:.2f} cm/s")
        
        # REBALANCED reward with positive components
        reward = 0.0
        
        # Base reward - slightly negative just for existing (encourages efficiency)
        reward -= 0.1
        
        # Distance component - normalized to be zero at target
        reward += 5.0 * (1.0 - min(1.0, z_distance/500.0))  # +5 at target, 0 when 500+ cm away
        
        # Progress reward - positive for moving toward goal
        if hasattr(self, 'prev_z_distance'):
            progress = self.prev_z_distance - z_distance
            reward += progress * 2.0  # Substantial reward for making progress
        
        # Target achievement bonus
        if z_distance < 30.0:
            reward += 10.0  # Big positive reward for reaching target
        
        # Efficiency penalty - applies only to excessive velocity
        optimal_velocity = min(80.0, max(5.0, z_distance * 0.1))  # Between 5-80 cm/s based on distance
        inefficiency = abs(abs(z_velocity) - optimal_velocity)
        reward -= min(1.0, inefficiency/50.0)  # Small penalty for inefficient velocity
        
        # Termination conditions
        self.prev_z_distance = z_distance
        done = self.steps >= 128
        info = {'height': current_z, 'target': target_z, 'distance': z_distance}
        self.steps += 1
        
        observation = self.get_observation()

        return observation, reward, done, False, info

    def send_velocity_command(self, velocity):
        command_topic = "VELOCITY"
        message = np.array(velocity, dtype=np.float32).tobytes()
        self.command_socket.send_multipart([command_topic.encode(), message])
 
    def send_reset_command(self):
        command_topic = "RESET"
        self.prev_goal_state = self.goal_state.copy()
        print("Sending reset command")
        self.command_socket.send_string(command_topic)
        time.sleep(0.1)

    def handle_data(self):
        try:
            if self.control_socket.poll(100, zmq.POLLIN):
                unified_data = self.control_socket.recv_string()
                data_parts = unified_data.split(";")
                if len(data_parts) != 3:
                    raise ValueError("Invalid data format")
                    
                parsed_data = {}
                for part in data_parts:
                    key, values = part.split(":")
                    parsed_data[key] = list(map(float, values.split(",")))
                
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
        try:
            [topic, message] = self.image_socket.recv_multipart(flags=zmq.NOBLOCK)
            image_data = np.frombuffer(message, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is not None:
                print("Image received! Shape:", image.shape)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                print("Image decode failed.")
            return image if image is not None else None
        except (zmq.Again, Exception) as e:
            return None

# Lower architecture to 8x8, 16x16 or even 32x32
# Increase learning rate a little bit
# Simplify reward functions


# New code for loading the best model and running it
if __name__ == "__main__":
    #best_model_path = "./RL_training/checkpoints/quad_model_420000_steps.zip"
    best_model_path = "./RL_training/best_model/best_model.zip"

    # Create the environment
    env = QuadSimEnv()

    # Load the model
    model = PPO.load(best_model_path, device="cpu")

    # Run the model
    obs, _ = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)
        if done or truncated:
            print("Resetting environment")
            obs, _ = env.reset()



'''
if __name__ == "__main__":
    # Hard-coded paths
    checkpoints_dir = "./RL_training/checkpoints"
    best_model_dir = "./RL_training/best_model"
    logs_dir = "./RL_training/logs"

    # Create the environment
    env = Monitor(QuadSimEnv(), logs_dir)
    
    # Set up evaluation and checkpoint callbacks
    eval_callback = EvalCallback(
        env,
        best_model_save_path=best_model_dir,
        log_path=logs_dir,
        eval_freq=5000,
        deterministic=True,
        render=False
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=checkpoints_dir,
        name_prefix="quad_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
        verbose=1
    )
    
    # Check for existing model checkpoints to continue training
    latest_model = None
    
    # First check best model
    best_model_path = best_model_dir + "/best_model.zip"
    if os.path.exists(best_model_path):
        latest_model = best_model_path
        print(f"Found best model: {latest_model}")
    
    # If no best model, check checkpoints
    if not latest_model:
        checkpoint_files = glob.glob(checkpoints_dir + "/*.zip")
        if checkpoint_files:
            # Sort by modification time (most recent first)
            checkpoint_files.sort(key=os.path.getmtime, reverse=True)
            latest_model = checkpoint_files[0]
            print(f"Found checkpoint: {latest_model}")
    
    # Create or load the model
    if latest_model:
        print(f"Continuing training from: {latest_model}")
        model = PPO.load(
            latest_model, 
            env=env,
            tensorboard_log=logs_dir
        )
        # Update learning rate and other hyperparameters if needed
        model.learning_rate = 2e-4
    ### TODO: Lower network architecture, 32x32
    else:
        print("Starting new training run")
        model = PPO(
            "MlpPolicy", 
            env,
            policy_kwargs=dict(net_arch=[128, 128]),
            learning_rate=2e-4,
            n_steps=2048,
            batch_size=64,
            gamma=0.99,
            verbose=1,
            tensorboard_log=logs_dir
        )
    
    try:
        # Begin training with the modified environment and callbacks
        model.learn(
            total_timesteps=512 * 1000,
            callback=[checkpoint_callback, eval_callback]
        )
    except KeyboardInterrupt:
        print("Training interrupted by user.")

'''


'''
class ImageDisplayCallback(BaseCallback):
    def __init__(self, update_freq=10, verbose=0):
        super(ImageDisplayCallback, self).__init__(verbose)
        self.update_freq = update_freq
        self.fig, self.ax = plt.subplots()
        self.img_disp = None
        plt.ion()
        plt.show()
        self.counter = 0

    def _on_step(self) -> bool:
        self.counter += 1
        if self.counter % self.update_freq == 0:
            try:
                env = self.training_env.envs[0].unwrapped
                image = env.get_data()
                # If no new image is available, use the last displayed image
                if image is None and self.img_disp is not None:
                    image = self.img_disp.get_array()
                elif image is None:
                    image = np.zeros((128, 128, 3), dtype=np.uint8)

                if self.img_disp is None:
                    self.img_disp = self.ax.imshow(image)
                else:
                    self.img_disp.set_data(image)
                self.fig.canvas.draw_idle()
                self.fig.canvas.flush_events()
            except Exception as e:
                print("Error updating image:", e)
        return True
'''
