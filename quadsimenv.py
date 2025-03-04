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
        # MODIFIED: Slightly reduced action smoothing for more responsive control
        self.prev_action = 0.8 * self.prev_action + 0.2 * action[0]
        full_action = np.array([0.0, 0.0, self.prev_action]) * 250.0
        
        for i in range(64):
            self.send_velocity_command(full_action)

        self.handle_data()

        # Update image state by receiving image from ZMQ
        image = self.get_data()
        if image is not None:
            self.state['image'] = image

        current_z = self.state['position'][2]
        z_velocity = self.state['velocity'][2]
        z_distance = self.calculate_z_distance(self.state['position'], self.goal_state)
        
        # MODIFIED: Improved reward function focused on altitude control
        reward = 0.0
        
        # Base distance reward - stronger penalty for being far from target
        distance_reward = -np.tanh(z_distance / 500.0) * 3.0  # Increased sensitivity and weight
        reward += distance_reward
        
        # Target height is expected to be around 1000 based on ZMQController
        target_height = self.goal_state[2]  # Using the received goal height
        
        # ADDED: Proximity bonus - gradually increases as drone gets closer to target height
        proximity_factor = np.exp(-z_distance / 200.0)  # Exponential reward that peaks at target
        reward += proximity_factor * 2.0
        
        # Safety height penalty
        min_safe_height = 100.0  
        if current_z < min_safe_height:
            height_penalty = -3.0 * (min_safe_height - current_z) / min_safe_height
            reward += height_penalty
        
        # Direction reward - encourage moving in the right direction
        if z_distance > 50:  # Only apply when not very close to target
            correct_direction = np.sign(z_velocity) == np.sign(self.goal_state[2] - current_z)
            reward += 1.5 if correct_direction else -1.5
        
        # MODIFIED: Refined hover behavior near goal
        if z_distance < 50.0:
            # Strong hover reward when close to target - encourage stability
            hover_reward = 2.0 - abs(z_velocity) * 0.05
            reward += hover_reward
        else:
            # Velocity efficiency - optimal speed is proportional to distance
            optimal_velocity = np.clip(np.sign(self.goal_state[2] - current_z) * 
                                      min(z_distance * 0.3, 300), -300, 300)
            velocity_efficiency = -abs(z_velocity - optimal_velocity) * 0.004
            reward += velocity_efficiency
                
        self.prev_velocity = z_velocity
        
        # Small energy penalty to discourage excessive movements
        energy_penalty = -abs(z_velocity) * 0.002
        reward += energy_penalty
        
        # ADDED: Completion bonus when very close to target
        if z_distance < 30.0:
            reward += 3.0 * (1.0 - z_distance/30.0)  # Up to +3 reward for perfect positioning

        done = self.steps >= 512
        info = {
            'z_distance': z_distance,
            'current_z': current_z,
            'target_z': self.goal_state[2]
        }
        self.steps += 1
        
        if done:
            observation, _ = self.reset(seed=None)
        else: 
            observation = self.get_observation()

        return observation, reward, done, False, info

    def send_velocity_command(self, velocity):
        command_topic = "VELOCITY"
        message = np.array(velocity, dtype=np.float32).tobytes()
        self.command_socket.send_multipart([command_topic.encode(), message])
 
    def send_reset_command(self):
        command_topic = "RESET"
        self.prev_goal_state = self.goal_state.copy()
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
                # We're keeping the goal as provided by ZMQController
                
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



# New code for loading the best model and running it
if __name__ == "__main__":
    best_model_path = "./RL_training/checkpoints/quad_model_510000_steps.zip"
    # Create the environment
    env = QuadSimEnv()

    # Load the model
    model = PPO.load(best_model_path)

    # Run the model
    obs, _ = env.reset()
    while True:
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)
        if done or truncated:
            obs, _ = env.reset()