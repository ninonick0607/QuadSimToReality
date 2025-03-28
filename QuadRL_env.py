#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
from example_interfaces.msg import Float64
import gymnasium as gym
from gymnasium import spaces
import threading
import time
import sys
import matplotlib.pyplot as plt

class QuadSimEnv(Node, gym.Env):
    def __init__(self):
        # Initialize both Node and gym.Env
        Node.__init__(self, 'quad_sim_env')
        gym.Env.__init__(self)
        
        # Set up QoS profiles
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=20,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscribers
        self.position_sub = self.create_subscription(
            Point,
            '/drone/position',
            self.position_callback,
            reliable_qos
        )
        
        self.goal_sub = self.create_subscription(
            Point,
            '/goal/position',
            self.goal_callback,
            reliable_qos
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            image_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            reliable_qos
        )
        
        self.reset_pub = self.create_publisher(
            String,
            '/reset',
            reliable_qos
        )

        self.obstacle_pub = self.create_publisher(
            Float64,
            '/obstacles',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                depth=1,
                durability=QoSDurabilityPolicy.VOLATILE
            )
        )


        # Action and observation spaces
        self.action_space = spaces.Box(
            low=np.array([-1.0]),
            high=np.array([1.0]),
            dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )

        # State variables
        self.current_position = np.zeros(3)
        self.current_velocity = np.zeros(3)
        self.goal_position = np.zeros(3)
        self.latest_image = np.zeros((128, 128, 3), dtype=np.uint8)
        self.prev_z = 0.0
        self.last_update_time = self.get_clock().now()
        self.prev_action = 0.0
        self.steps = 0
        self.lock = threading.Lock()

        # Start ROS spinning in a separate thread
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

    def position_callback(self, msg):
        with self.lock:
            now = self.get_clock().now()
            dt = (now - self.last_update_time).nanoseconds / 1e9
            
            # Calculate velocity from position differences
            new_position = np.array([msg.x, msg.y, msg.z])
            if dt > 0:
                self.current_velocity = (new_position - self.current_position) / dt
            self.current_position = new_position
            self.last_update_time = now

    def goal_callback(self, msg):
        with self.lock:
            self.goal_position = np.array([msg.x, msg.y, msg.z])

    def image_callback(self, msg):
        try:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img_data = img_data.reshape((msg.height, msg.width, 3))
            if msg.encoding == 'bgr8':
                self.latest_image = img_data[:, :, ::-1]  # Convert BGR to RGB
            else:
                self.latest_image = img_data
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def send_obstacle_command(self, obstacleNum):
        msg = Float64()
        msg.data = float(obstacleNum)
        
        # Send once with confirmation
        self.obstacle_pub.publish(msg)
        self.get_logger().info(f"Sent obstacle count: {obstacleNum}")
        
        # Verify publication
        print(f"\n--- PUBLISHED OBSTACLE COUNT: {obstacleNum} ---\n")
        print("Verify with: ros2 topic echo /obstacles")

    def get_observation(self):
        with self.lock:
            z_vel = self.current_velocity[2] / 100.0  # Normalize velocity
            z_dist = (self.goal_position[2] - self.current_position[2]) / 2000.0  # Normalize distance
            return np.array([z_vel, z_dist], dtype=np.float32)

    def reset(self, seed=None, options=None):
        self.get_logger().info("Resetting environment")
        reset_msg = String()
        reset_msg.data = "reset"
        self.reset_pub.publish(reset_msg)
        
        # Reset internal state
        with self.lock:
            self.current_position = np.zeros(3)
            self.current_velocity = np.zeros(3)
            self.prev_action = 0.0
            self.steps = 0
            
        # Allow time for reset to propagate
        time.sleep(1.0)
        return self.get_observation(), {}

    def step(self, action):
        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.z = float(action[0] * 250.0)  # Scale action
        self.cmd_vel_pub.publish(cmd_vel)

        # Allow time for state update
        time.sleep(0.01)
        
        # Get current state
        observation = self.get_observation()
        self.steps += 1

        # Calculate reward
        with self.lock:
            z_dist = abs(self.goal_position[2] - self.current_position[2])
            z_vel = self.current_velocity[2]
            
        reward = -0.1  # Time penalty
        reward += 5.0 * (1.0 - min(1.0, z_dist/500.0))  # Distance component
        
        # Progress reward
        if hasattr(self, 'prev_z_dist'):
            progress = self.prev_z_dist - z_dist
            reward += progress * 2.0
        self.prev_z_dist = z_dist
        
        # Target bonus
        if z_dist < 30.0:
            reward += 10.0
            
        # Efficiency penalty
        optimal_velocity = min(80.0, max(5.0, z_dist * 0.1))
        inefficiency = abs(abs(z_vel) - optimal_velocity)
        reward -= min(1.0, inefficiency/50.0)

        # Termination condition
        done = self.steps >= 512
        info = {
            'distance': z_dist,
            'velocity': z_vel,
            'position': self.current_position[2]
        }

        return observation, reward, done, False, info

# --------------------------
# Main function for testing
# --------------------------
def main(args=None):
    rclpy.init(args=args)
    env = QuadSimEnv()
    time.sleep(2)

    # Get obstacle count from command line if provided
    obstacle_count = 150
    if len(sys.argv) > 1:
        try:
            obstacle_count = int(sys.argv[1])
        except ValueError:
            print(f"Invalid obstacle count: {sys.argv[1]}, using default: 150")

    # Send obstacle command
    env.send_obstacle_command(obstacle_count)

    # Set up matplotlib figure for displaying camera feed and drone data
    plt.ion()
    fig, (ax_img, ax_data) = plt.subplots(1, 2, figsize=(12, 6))
    
    # Initialize image display
    img_display = ax_img.imshow(env.latest_image)
    ax_img.set_title("Camera Feed")
    ax_img.axis('off')
    
    # Initialize text display for current position and goal
    data_text = ax_data.text(0.5, 0.5, '', ha='center', va='center', fontsize=12)
    ax_data.axis('off')
    ax_data.set_title("Drone Data")
    
    # ---------------------------
    # Commented out training part:
    # 
    # from stable_baselines3 import PPO
    # model = PPO.load("best_model.zip")
    # obs, _ = env.reset()
    # while True:
    #     action, _states = model.predict(obs, deterministic=True)
    #     obs, reward, done, _, info = env.step(action)
    #     if done:
    #         obs, _ = env.reset()
    # ---------------------------
    
    # Instead, we simply run a loop to update our display based on incoming data.
    try:
        while rclpy.ok():
            # Update the image display with the latest image
            img_display.set_data(env.latest_image)
            
            # Get current position and goal position safely
            with env.lock:
                pos = env.current_position.copy()
                goal = env.goal_position.copy()
            
            # Update the text display
            data_text.set_text(f"Current Position:\nX: {pos[0]:.2f}, Y: {pos[1]:.2f}, Z: {pos[2]:.2f}\n\n"
                               f"Goal Position:\nX: {goal[0]:.2f}, Y: {goal[1]:.2f}, Z: {goal[2]:.2f}")
            
            fig.canvas.draw_idle()
            plt.pause(0.1)
            time.sleep(0.05)
    except KeyboardInterrupt:
        env.get_logger().info("Shutting down test display...")
    finally:
        env.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
