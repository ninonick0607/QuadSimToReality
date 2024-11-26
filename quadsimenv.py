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

    def reset(self):
        self.state = {
            'image': np.zeros((128, 128, 3), dtype=np.uint8),
            'velocity': np.zeros(3, dtype=np.float32)
        }
        return self.state

    def step(self, action):
        

        self.send_velocity_command(action)

        image = self.get_image()
        if image is not None:
            self.state['image'] = image
        else:
            pass

        self.state['velocity'] = action

        reward = 0  
        done = False  
        info = {}

        return self.state, reward, done, info

    def send_velocity_command(self, velocity):

        drone_id = "drone1"  
        message = np.array(velocity, dtype=np.float32).tobytes()


        print(velocity)
        self.command_socket.send_multipart([drone_id.encode(), message])

    def get_image(self):
        try:

            [topic, message] = self.image_socket.recv_multipart(flags=zmq.NOBLOCK)
            drone_id = topic.decode()
            image_data = np.frombuffer(message, dtype=np.uint8)

            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            if image is not None:

                return image
            else:
                print("Failed to decode image")
                return None
        except zmq.Again:
            # No message was available
            return None
        except Exception as e:
            print(f"Error receiving image: {e}")
            return None

if __name__ == "__main__":
    env = QuadSimEnv()
    obs = env.reset()

    while True:
        action = env.action_space.sample()  
        action = [0, 0, 250]
        obs, reward, done, info = env.step(action)

        time.sleep(0.1) 
