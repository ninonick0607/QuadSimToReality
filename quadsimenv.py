import gymnasium as gym
import numpy as np
import zmq
import time
import cv2  
from stable_baselines3 import PPO
import struct

class QuadSimEnv(gym.Env):
    def __init__(self):
        super(QuadSimEnv, self).__init__()

        # Actions remain 3-dimensional (although you might primarily focus on z later).
        self.action_space = gym.spaces.Box(
            low=np.array([-1, -1, -1]), 
            high=np.array([1, 1, 1]),
            dtype=np.float32
        )

        # Observation is now only 2-dimensional:
        # [normalized vertical velocity, normalized altitude error]
        self.observation_space = gym.spaces.Box(
            low=np.array([-1, -1]),
            high=np.array([1, 1]),
            dtype=np.float32
        )

        # Initial state for image, velocity, and position (3D)
        self.state = {
            'image': np.zeros((128, 128, 3), dtype=np.uint8),
            'velocity': np.zeros(3, dtype=np.float32),
            'position': np.zeros(3, dtype=np.float32)
        }

        # Goal state: we care mostly about the z component (altitude)
        self.goal_state = np.zeros(3, dtype=np.float32)
        self.prev_goal_state = np.zeros(3, dtype=np.float32)
        self.context = zmq.Context()

        # Subscriber for images.
        self.image_socket = self.context.socket(zmq.SUB)
        self.image_socket.connect("tcp://localhost:5557")
        self.image_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        # Publisher for velocity/reset commands.
        self.command_socket = self.context.socket(zmq.PUB)
        self.command_socket.bind("tcp://*:5556")

        # Subscriber for state/control data.
        self.control_socket = self.context.socket(zmq.SUB)
        self.control_socket.connect("tcp://localhost:5558")
        self.control_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        self.steps = 0
        time.sleep(0.1)

    def reset(self, seed=None):
        self.send_reset_command()
        self.handle_data()
        self.steps = 0

        # Compute the altitude error (goal z - current z)
        relative_z = self.goal_state[2] - self.state['position'][2]
        # Normalize the z-velocity and altitude difference.
        observation = np.array([
            self.state['velocity'][2] / 150.0,   # normalized vertical velocity
            relative_z / 100_000.0               # normalized altitude error
        ], dtype=np.float32)
        return observation, {}

    def calculate_distance(self, pos1, pos2):
        return np.sqrt(np.sum((np.array(pos1) - np.array(pos2)) ** 2))
    
    def step(self, action):
        # Scale the action.
        action = np.array(action) * 150.0

        # Send the velocity command repeatedly to ensure it is processed.
        for i in range(64):
            self.send_velocity_command(action)

        self.handle_data()
        image = self.get_data()
        if image is not None:
            self.state['image'] = image

        # Compute the altitude error.
        relative_z = self.goal_state[2] - self.state['position'][2]
        # Reward is the vertical velocity. This is positive if ascending,
        # negative if descending. (You can modify this if you want to penalize altitude error.)
        reward = self.state['velocity'][2]
        done = self.steps >= 512
        info = {}
        self.steps += 1

        if done:
            observation, _ = self.reset(seed=None)
        else:
            observation = np.array([
                self.state['velocity'][2] / 150.0,
                relative_z / 100_000.0
            ], dtype=np.float32)

        return observation, reward, done, False, info

    def send_velocity_command(self, velocity):
        drone_id = "drone1"
        message = np.array(velocity, dtype=np.float32).tobytes()
        self.command_socket.send_multipart([drone_id.encode(), message])

    def send_reset_command(self):
        command_topic = "RESET"
        self.prev_goal_state = self.goal_state.copy()
        self.command_socket.send_string(command_topic)
        time.sleep(0.1)

    def handle_data(self):
        unified_data = ""
        try:
            while True:
                try:
                    unified_data = self.control_socket.recv_string(flags=zmq.NOBLOCK)
                    break  
                except zmq.Again:
                    break

            if unified_data:
                data_parts = unified_data.split(";")
                parsed_data = {key: list(map(float, value.split(","))) 
                               for key, value in (part.split(":") for part in data_parts)}
                self.state['velocity'] = parsed_data.get("VELOCITY", [0.0, 0.0, 0.0])
                self.state['position'] = parsed_data.get("POSITION", [0.0, 0.0, 0.0])
                self.goal_state = parsed_data.get("GOAL", [0.0, 0.0, 0.0])
        except Exception as e:
            print(f"Error parsing unified data: {e}")

    def get_data(self):
        try:
            [topic, message] = self.image_socket.recv_multipart(flags=zmq.NOBLOCK)
            # We decode the image.
            image_data = np.frombuffer(message, dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return image if image is not None else None
        except zmq.Again:
            return None
        except Exception as e:
            return None

if __name__ == "__main__":
    env = QuadSimEnv()
    model = PPO("MlpPolicy", env, verbose=1, learning_rate=0.003, n_steps=512)
    model.learn(total_timesteps=512 * 1000)
    model.save("ppo_quad_sim")
