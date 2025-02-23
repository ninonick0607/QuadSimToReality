import gymnasium as gym
import numpy as np
import zmq
import time
import cv2  
import os
import matplotlib.pyplot as plt
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
        z_vel = self.state['velocity'][2] / 150.0  # Normalize velocity
        z_to_goal = (self.goal_state[2] - self.state['position'][2]) / 5000.0  # Normalize distance
        return np.array([z_vel, z_to_goal])

    def reset(self, seed=None):
        self.send_reset_command()
        self.handle_data()
        self.steps = 0
        return self.get_observation(), {}
    
    def calculate_z_distance(self, pos1, pos2):
        return abs(pos1[2] - pos2[2])
    
    def step(self, action):
        self.prev_action = 0.7 * self.prev_action + 0.3 * action[0]
        full_action = np.array([0.0, 0.0, self.prev_action]) * 250.0
        
        for i in range(64):
            self.send_velocity_command(full_action)

        self.handle_data()

        # Update image state by receiving image from ZMQ
        image = self.get_data()
        if image is not None:
            self.state['image'] = image
        # You could optionally log here to verify an image was received:
        else:
            print("No image received in this step.")

        current_z = self.state['position'][2]
        z_velocity = self.state['velocity'][2]
        z_distance = self.calculate_z_distance(self.state['position'], self.goal_state)
        

        current_z = self.state['position'][2]
        z_velocity = self.state['velocity'][2]
        z_distance = self.calculate_z_distance(self.state['position'], self.goal_state)
        
        reward = 0.0
        distance_reward = -np.tanh(z_distance / 1000.0) 
        reward += distance_reward * 2.0 

        min_safe_height = 100.0  
        if current_z < min_safe_height:
            height_penalty = -2.0 * (min_safe_height - current_z) / min_safe_height
            reward += height_penalty
        
        if z_distance > 0:
            correct_direction = np.sign(z_velocity) == np.sign(self.goal_state[2] - current_z)
            reward += 1.0 if correct_direction else -1.0
        
        if z_distance < 100.0:
            hover_penalty = -abs(z_velocity) * 2.0
            reward += hover_penalty
        else:
            optimal_velocity = np.clip(z_distance * 0.2, -500, 500)
            velocity_efficiency = -abs(z_velocity - optimal_velocity) * 0.3
            reward += velocity_efficiency
                
        self.prev_velocity = z_velocity
        energy_penalty = -abs(z_velocity) * 0.01
        reward += energy_penalty

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
        #print(f"[Python] Sending velocity command: {velocity}")
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
            print("No image received or error:", e)
            return None


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


# --- Main training code ---
if __name__ == "__main__":
    rl_dir = "./RL_training"
    checkpoints_dir = os.path.join(rl_dir, "checkpoints")
    best_model_dir = os.path.join(rl_dir, "best_model")
    logs_dir = os.path.join(rl_dir, "logs")
    
    os.makedirs(checkpoints_dir, exist_ok=True)
    os.makedirs(best_model_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)
    
    # Create the environment
    env = Monitor(QuadSimEnv(), logs_dir)
    
    # Set up evaluation and checkpoint callbacks
    eval_callback = EvalCallback(
        env,
        best_model_save_path=best_model_dir,
        log_path=logs_dir,
        eval_freq=5000,  # Evaluate every 5000 steps
        deterministic=True,
        render=False
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=5000,  # Save every 5000 steps
        save_path=checkpoints_dir,
        name_prefix="quad_model",
        save_replay_buffer=True,
        save_vecnormalize=True,
        verbose=1
    )
    
    # Create the image display callback (no threading needed now)
    image_display_callback = ImageDisplayCallback(update_freq=10)

    # Initialize the PPO model
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
    
    try:
        # Begin training; the image display callback updates the plot on every 'update_freq' steps
        model.learn(
            total_timesteps=512 * 1000,
            callback=[checkpoint_callback, eval_callback, image_display_callback]
        )
    except KeyboardInterrupt:
        print("Training interrupted by user.")
