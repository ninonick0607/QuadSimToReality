import gymnasium as gym
import numpy as np
import zmq
import time
import matplotlib.pyplot as plt
import glob 
import os
import struct
import cv2                
from collections import OrderedDict
from typing import Callable
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback, BaseCallback
from stable_baselines3.common.  monitor import Monitor

#import tensorboard
# --- QuadSimEnv definition ---
class QuadSimEnv(gym.Env):
    def __init__(self, action_frequency: int=10, reward_fn: Callable[[float], float]=None):
        super(QuadSimEnv, self).__init__()  

        self.action_space = gym.spaces.Box(
            low=-1,  
            high=1,
            shape=(1,), # Yaw Rate command
            dtype=np.float32
        )

        self.observation_space = gym.spaces.Dict({
            "observation": gym.spaces.Box(
                low=-np.inf,
                high=np.inf,
                shape=(6,),
                dtype=np.float32
            ),
            "pixels": gym.spaces.Box(
                low=0,
                high=255,
                shape=(128, 128, 3),
                dtype=np.uint8
            )
        })

        self.state = {
            'velocity': np.zeros(3, dtype=np.float32),
            'position': np.zeros(3, dtype=np.float32),
            'attitude': np.zeros(3, dtype=np.float32)
        }

        self.goal_state = np.zeros(3, dtype=np.float32)
        self.prev_goal_state = np.zeros(3, dtype=np.float32)
        self.prev_action = 0.0  
        self.prev_velocity = 0.0
        self.image = np.zeros((128, 128, 3), dtype=np.uint8)
        self.action_frequency = action_frequency
        self.context = zmq.Context()

        if reward_fn is not None: self.reward_fn = reward_fn
        else:
            self.reward_fn = lambda angle: np.maximum(0, np.maximum(0.5 - (angle - 5) / 170, 1 - angle / 10))


        # Subscriber socket for receiving images
        self.image_socket = self.context.socket(zmq.SUB)
        self.image_socket.setsockopt(zmq.CONFLATE, 1)
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

        # Publisher socket for sending obstacle commands
        self.obstacle_socket = self.context.socket(zmq.PUB)
        self.obstacle_socket.bind("tcp://*:5559")
        
        self.collision_state = False

        self.collision_socket = self.context.socket(zmq.SUB)
        self.collision_socket.setsockopt(zmq.CONFLATE, 1)
        self.collision_socket.connect("tcp://localhost:5560") # Connect to the new CollisionPort
        self.collision_socket.setsockopt_string(zmq.SUBSCRIBE, '')
        # --- Matplotlib Setup ---
        self.fig, self.ax = plt.subplots()
        self.im_display = self.ax.imshow(self.image) # Initial display object
        plt.ion() # Turn on interactive mode
        plt.show(block=False) # Show the plot without blocking
        # --- End Matplotlib Setup ---
        self.steps = 0
        time.sleep(0.1)

    def get_observation(self):
        """
        Returns a vector with the state information relevant to training:
        (0-2) Quadrotor velocity (vx, vy, vz)
        (3) Distance to goal (scalar)
        (4-5) Angle to goal (relative to drone body, cosine and sine)
        """
        
        vel = self.state['velocity']
        distance_to_goal = np.linalg.norm(self.state['position'][:-1] - self.goal_state[:-1])
        global_angle_to_goal = np.arctan2(self.goal_state[1] - self.state['position'][1], self.goal_state[0] - self.state['position'][0])
        local_angle_to_goal = np.rad2deg(global_angle_to_goal) - self.state['attitude'][2]
        local_angle_to_goal = np.deg2rad(local_angle_to_goal)

        return np.array([*vel, distance_to_goal, np.cos(local_angle_to_goal), np.sin(local_angle_to_goal)], dtype=np.float32)

    def reset(self, seed=None):
        # self.send_reset_command()
        self.send_obstacle_command(1, True)
        time.sleep(0.1)  # Wait for the reset to take effect
        self.handle_data()
        self.handle_collision_data() 
        self.steps = 0
        obs = self.get_observation()
        # self.image = self.retrieve_image()
        complete_obs = OrderedDict([
            ('pixels', self.image),
            ('observation', obs)
        ])
        return complete_obs, {}
    
    
    def step(self, action):
        # Apply action and update environment
        # full_action = np.array([*action, 0.0]) * 250.0
        full_action = np.array([0, 0, action[0], 0])
        
        self.send_velocity_command(full_action)

        time.sleep(1 / self.action_frequency) # Action frequency is ~10 Hz

        self.handle_data()
        self.handle_collision_data() 
        
        observation = self.get_observation()
        # if self.steps % 5 == 0:
        #     self.image = self.retrieve_image()
        complete_obs = OrderedDict([
            ('pixels', self.image),
            ('observation', observation)
        ])

        # reward = 1 - (observation[6] / 13000) # Reward based on distance to goal (normalized to ~[0, 1])
        # reward += 1 - np.abs((observation[2] - 250) / 250) # Reward based on altitude (reward 1 is 250cm, reward 0 = 0cm or 500cm)
        local_angle = np.abs(np.rad2deg(np.arctan2(observation[5], observation[4])))
        # Reward based on angle to goal
        reward = self.reward_fn(local_angle)

        # Termination conditions
        done = False
        if self.steps >= 256: done = True; print("Max steps reached")
        if self.collision_state:
            done = True; print("Collision detected")
            reward -= 1.0 # Optional penalty
        # if observation[2] > 500: done = True; print("Quadrotor too high")
        # if observation[2] < 5: done = True; print("Quadrotor too low")
        
        self.steps += 1
        return complete_obs, reward, done, False, {}

    def send_velocity_command(self, velocity):
        # Should be a 1D numpy array with 4 elements: [vx, vy, vz, yaw_rate]
        command_topic = "VELOCITY"
        message = np.array(velocity, dtype=np.float32).tobytes()
        self.command_socket.send_multipart([command_topic.encode(), message])
        
    def send_reset_command(self):
        command_topic = "RESET"
        self.prev_goal_state = self.goal_state.copy()
        print("Sending reset command")
        self.command_socket.send_string(command_topic)
        time.sleep(0.1)

    def send_obstacle_command(self, obstacleNum, bObstacleRand):
        # print("Obstacles called")
        obstacle_topic = "CREATE_OBSTACLE"
        float_data = struct.pack('f', float(obstacleNum))
        bool_data = struct.pack('?', bool(bObstacleRand))
        
        self.obstacle_socket.send_multipart([
            obstacle_topic.encode(), 
            float_data,
            bool_data
        ])
        
    def handle_data(self):
            print("Checking for state data...") # DEBUG PRINT
            try:
                # Use poll with a short timeout instead of NOBLOCK for initial check
                if self.control_socket.poll(10): # Poll for 10 milliseconds
                    unified_data = self.control_socket.recv_string()
                    print(f"--- RAW STATE DATA RECEIVED: {unified_data}") # DEBUG PRINT
                    data_parts = unified_data.split(";")
                    parsed_data = {}
                    for part in data_parts:
                        key, values_str = part.split(":")
                        values = values_str.split(",")
                        if len(values) != 3:
                            print(f"!!! Invalid data for key {key}: {values_str}") # DEBUG PRINT
                            # Optional: return or raise error
                            return # Exit processing if format is wrong
                        parsed_data[key] = list(map(float, values))

                    self.state.update({
                        'velocity': np.array(parsed_data["VELOCITY"]),
                        'position': np.array(parsed_data["POSITION"]),
                        'attitude': np.array(parsed_data["ATTITUDE"])
                    })
                    self.goal_state = np.array(parsed_data["GOAL"])
                    print("--- State data successfully parsed.") # DEBUG PRINT
                # else: # Optional print if you want to see polls with no data
                #     print("--- No state data available in poll.") # DEBUG PRINT

            except Exception as e:
                print(f"!!! State data handling EXCEPTION: {str(e)}") # DEBUG PRINT

    def handle_collision_data(self):
        print("Checking for collision data...") # DEBUG PRINT
        try:
            # Use non-blocking receive here is fine
            collision_msg = self.collision_socket.recv_string(flags=zmq.NOBLOCK)
            print(f"--- RAW COLLISION DATA RECEIVED: {collision_msg}") # DEBUG PRINT
            self.collision_state = (collision_msg == "1")
            print(f"--- Collision state set to: {self.collision_state}") # DEBUG PRINT
        except zmq.Again:
            # This is expected when no new message is available
            print("--- No new collision data (zmq.Again).") # DEBUG PRINT
            pass # Keep the last state
        except Exception as e:
            print(f"!!! Collision handling EXCEPTION: {str(e)}") # DEBUG PRINT
            self.collision_state = False
# Inside QuadSimEnv class:
    def retrieve_image(self):
        """Receives and decodes image data from the ZMQ socket."""
        try:
            # Use recv() assuming the whole message is the image bytes
            message = self.image_socket.recv(flags=zmq.NOBLOCK)
            image_data = np.frombuffer(message, dtype=np.uint8)
            # Decode assuming it's a standard format like JPEG or PNG
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is not None:
                # print("Image received! Shape:", image.shape) # Optional debug
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # Convert BGR to RGB for Matplotlib
                # Ensure the image has the expected shape (optional resize/crop)
                if image.shape[0:2] != (128, 128):
                     # Example: Resize if it's not the correct size
                     image = cv2.resize(image, (128, 128), interpolation=cv2.INTER_AREA)
                self.image = image # Update the class's image attribute
                return image
            else:
                # Failed decoding, keep the old image
                print("Warning: Failed to decode image")
                return self.image
        except zmq.Again:
            # No new message, return the last known image
            return self.image
        except Exception as e:
            print(f"Error receiving/processing image: {str(e)}")
            # Return the last known image on other errors
            return self.image
        
    def close(self):
        # Terminate the ZeroMQ context
        if hasattr(self, 'context') and self.context:
            self.context.destroy()
            self.context = None
        super().close()


if __name__ == "__main__":
    best_model_path = "./RL_training/best_model/best_model.zip"
    env = QuadSimEnv()
    time.sleep(1.0)
    env.send_obstacle_command(100, True) # Optionally send command
    # time.sleep(1.0)
    # env.handle_data() # Initial fetch if needed

    try:
        step_count = 0
        while True:
            env.handle_data()
            env.handle_collision_data()

            # --- Retrieve and Display Image ---
            current_image = env.retrieve_image() # Gets latest or last known image
            env.im_display.set_data(current_image) # Update plot data
            plt.pause(0.05) # Allow plot to redraw (adjust pause as needed)
            # --- End Image Display ---

            # You might add dummy actions or other logic here for testing
            # action = env.action_space.sample() # Example
            # obs, reward, done, _, info = env.step(action) # Example
            # if done:
            #    env.reset()

            time.sleep(0.1) # Control loop speed
            step_count += 1
            if step_count > 500: # Limit test duration
                 break

    except KeyboardInterrupt:
        print("\nLoop interrupted by user.")
    finally:
        print("Closing environment.")
        env.close()
