import gymnasium as gym
import numpy as np
import zmq
import time
import cv2
import sys
from PyQt5.QtWidgets import QPushButton
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QSlider, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt

class QuadSimEnv(gym.Env):
    def __init__(self):
        super(QuadSimEnv, self).__init__()

        # Updated action space to have min 0 and max 250 for all axes
        self.action_space = gym.spaces.Box(
            low=np.array([0, 0, 0]),
            high=np.array([250, 250, 250]),
            dtype=np.float32
        )

        # Updated observation space accordingly
        self.observation_space = gym.spaces.Dict({
            'image': gym.spaces.Box(
                low=0, high=255, shape=(128, 128, 3), dtype=np.uint8
            ),
            'velocity': gym.spaces.Box(
                low=np.array([0, 0, 0]), high=np.array([250, 250, 250]), dtype=np.float32
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

        self.state['velocity'] = action

        reward = 0
        done = False
        info = {}

        return self.state, reward, done, info

    def send_velocity_command(self, velocity):
        drone_id = "drone1"
        message = np.array(velocity, dtype=np.float32).tobytes()

        print(f"Sending velocity command: {velocity}")
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

class ActionSlider(QWidget):
    def __init__(self, update_action_callback):
        super().__init__()
        self.update_action_callback = update_action_callback
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Action Slider Control')
        self.setGeometry(100, 100, 400, 250)

        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []
        ranges = [(-250, 250), (-250, 250), (-250, 250)]
        initial_values = [0, 0, 0]
        labels = ['X', 'Y', 'Z']

        for i in range(3):
            h_layout = QHBoxLayout()

            label = QLabel(f"{labels[i]}: {initial_values[i]:.2f}")
            self.labels.append(label)
            h_layout.addWidget(label)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(ranges[i][0])
            slider.setMaximum(ranges[i][1])
            slider.setValue(initial_values[i])
            slider.valueChanged.connect(self.slider_changed)
            self.sliders.append(slider)
            h_layout.addWidget(slider)

            reset_button = QPushButton(f"Reset {labels[i]}")
            reset_button.clicked.connect(lambda checked, idx=i: self.reset_single_slider(idx))
            h_layout.addWidget(reset_button)

            layout.addLayout(h_layout)

        reset_all_button = QPushButton("Reset All")
        reset_all_button.clicked.connect(self.reset_all_sliders)
        layout.addWidget(reset_all_button)

        self.setLayout(layout)

    def slider_changed(self):
        action = []
        for i in range(3):
            value = self.sliders[i].value()
            self.labels[i].setText(f"{['X', 'Y', 'Z'][i]}: {value:.2f}")
            action.append(value)
        self.update_action_callback(action)

    def reset_single_slider(self, index):
        self.sliders[index].setValue(0)
        self.labels[index].setText(f"{['X', 'Y', 'Z'][index]}: 0.00")
        self.slider_changed()

    def reset_all_sliders(self):
        for i in range(3):
            self.reset_single_slider(i)

if __name__ == "__main__":
    env = QuadSimEnv()
    obs = env.reset()

    # Initialize QApplication
    app = QApplication(sys.argv)

    # Variable to hold the current action
    current_action = [0, 0, 0]

    # Function to update the action based on slider values
    def update_action(action_values):
        global current_action
        current_action = action_values

    # Create and show the slider widget
    slider_widget = ActionSlider(update_action)
    slider_widget.show()

    # Run the main loop in a QTimer to keep the GUI responsive
    from PyQt5.QtCore import QTimer

    def main_loop():
        action = current_action
        obs, reward, done, info = env.step(action)
        # If you want to display images, you can add code here
        # to show self.state['image'] using OpenCV or PyQt

    timer = QTimer()
    timer.timeout.connect(main_loop)
    timer.start(100)  # Adjust the timer interval as needed (in milliseconds)

    sys.exit(app.exec_())
