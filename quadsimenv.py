import gymnasium as gym
import numpy as np
import zmq
import time
import cv2  

class QuadSimEnv(gym.Env):
    def __init__(self):
        super(QuadSimEnv, self).__init__()

        self.action_space = gym.spaces.Box(
            low=np.array([-1, -1, -1]), 
            high=np.array([1, 1, 1]),
            dtype=np.float32
        )

        self.observation_space = gym.spaces.Dict({
            'image': gym.spaces.Box(
                low=0, high=255, shape=(128, 128, 3), dtype=np.uint8
            ),
            'velocity': gym.spaces.Box(
                low=np.array([-1, -1, -1]), high=np.array([1, 1, 1]), dtype=np.float32
            )
        })

        self.state = {
            'image': np.zeros((128, 128, 3), dtype=np.uint8),
            'velocity': np.zeros(3, dtype=np.float32)
        }

        self.context = zmq.Context()

        # Subscriber socket for receiving images
        self.image_socket = self.context.socket(zmq.SUB)
        self.image_socket.connect("tcp://localhost:5555")
        self.image_socket.setsockopt_string(zmq.SUBSCRIBE, '')

        # Publisher socket for sending velocity commands
        self.command_socket = self.context.socket(zmq.PUB)
        self.command_socket.bind("tcp://*:5556")  

        time.sleep(0.1)

    def send_velocity_command(self, velocity):
        drone_id = "drone1"  
        message = np.array(velocity, dtype=np.float32).tobytes()
        print(f"Sending velocity command: {velocity}")
        self.command_socket.send_multipart([drone_id.encode(), message])

if __name__ == "__main__":
    env = QuadSimEnv()

    # Command to move up at 250 m/s in the Z direction
    velocity_command = [0, 0, 250]  
    env.send_velocity_command(velocity_command)
    print("Velocity command sent.")
