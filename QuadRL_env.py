#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from example_interfaces.msg import Empty
from example_interfaces.msg import Float64
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec 
import numpy as np 

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import threading
import time
import sys
import os
import glob 
from scipy.spatial.transform import Rotation as R 

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor

# Constants based on analysis and quadsimenv.py / main.py (MLP_no_obstacle_3)
MAX_FORWARD_SPEED_CMD = 100.0 # cm/s (Adjust based on desired max speed) - Using 100 as placeholder
MAX_YAW_RATE_CMD = np.pi / 2 # rad/s (Adjust based on desired max yaw rate) - Using pi/2 as placeholder
OBS_NORM_SCALE = np.array([600.0, 600.0, 600.0, 2200.0, 1.0, 1.0], dtype=np.float32)
OBS_NORM_LOC = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32) # Loc is 0 for MLP_no_obstacle_3
REWARD_DIST_SCALE_CM = 2200.0 # From main.py reward_fn_angle_penalty
REWARD_ANGLE_SCALE_DEG = 1800.0 # From main.py reward_fn_angle_penalty (Note: main.py uses 1800, implies angle range?)



class QuadSimEnv(Node, gym.Env):

    def __init__(self):
        Node.__init__(self, 'quad_controller_env_rl')
        gym.Env.__init__(self)
        self.get_logger().info("Initializing QuadSim ROS2 Gym Environment...")
        self.lock = threading.Lock() 

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

        self.state = {
            'velocity':np.zeros(3,dtype=np.float64),
            'position':np.zeros(3,dtype=Float64),
            'attitude':np.zeros(3,dtype=Float64)
        }

        # --- Subscribers ---
        self.odometry_sub = self.create_subscription( Odometry, '/odom', self.odometry_callback, reliable_qos )
        self.goal_sub = self.create_subscription( Point, '/goal/position', self.goal_callback, reliable_qos )
        self.image_sub = self.create_subscription( Image, '/camera/image', self.image_callback, image_qos )

        self.collision_state = False
        self.collision_sub = self.create_subscription(
            Float64,
            '/drone/collision',
            self.collision_callback,
            reliable_qos
        )
        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.reset_pub = self.create_publisher(Empty, '/reset', reliable_qos) # Change String to Empty
        self.obstacle_pub = self.create_publisher( Float64, '/obstacles', QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1, durability=QoSDurabilityPolicy.VOLATILE))
        self.hover_pub = self.create_publisher(Float64,'/hover/height',reliable_qos)
        # --- Action/Observation Spaces ---
        self.action_space = spaces.Box( low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32 )
        self.observation_space = spaces.Box( low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32 )


        # --- State Variables ---
        self.goal_position_m = np.zeros(3, dtype=np.float32)
        self.goal_position_cm = np.zeros(3, dtype=np.float32)
        self.latest_image = np.zeros((128, 128, 3), dtype=np.uint8) # Default black image
        self.last_state_update_time = self.get_clock().now()
        self.obstacle_count = 0.0
        self.distance_to_goal_cm = 0.0
        self.local_angle_rad = 0.0
        self.local_angle_deg = 0.0
        self.steps = 0
        self.image_received_flag = False # Flag to check if we got at least one image

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()
        self.get_logger().info("ROS2 spinning started in background thread.")
        time.sleep(1.0)

    def odometry_callback(self, msg):
        with self.lock:
            pos_m = msg.pose.pose.position
            self.state['position'] = np.array([pos_m.x, pos_m.y, pos_m.z], dtype=np.float32) * 100.0

            quat = msg.pose.pose.orientation
            try:
                r = R.from_quat([quat.x, quat.y, quat.z, quat.w])
                euler_rad = r.as_euler('xyz', degrees=False)
                self.state['attitude'] = euler_rad.astype(np.float32)
            except Exception as e:
                 self.get_logger().error(f"Quaternion to Euler conversion failed: {e}")

            vel_mps = msg.twist.twist.linear
            self.state['velocity'] = np.array([vel_mps.x, vel_mps.y, vel_mps.z], dtype=np.float32) * 100.0

            try:
                self.last_state_update_time = rclpy.time.Time.from_msg(msg.header.stamp)
            except Exception as e:
                self.get_logger().warn(f"Could not get timestamp from Odometry header: {e}. Using current time.")
                self.last_state_update_time = self.get_clock().now()

            # Calculate metrics immediately after state update if goal is known
            if np.any(self.goal_position_cm): # Check if goal has been set
                self._calculate_goal_metrics()

    def goal_callback(self, msg):
        with self.lock:
            self.goal_position_cm = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
            self._calculate_goal_metrics()
    def collision_callback(self, msg):
        with self.lock:
            self.collision_state = msg.data
    def image_callback(self, msg):
        # --- Updated Image Callback (Mirroring DroneMonitor logic) ---
        try:
            expected_size = 128 * 128 * 3
            # Check 1: Basic size check (like DroneMonitor)
            if len(msg.data) != expected_size:
                self.get_logger().error(f"Invalid image data size: got {len(msg.data)} bytes, expected {expected_size}")
                # Optional: Check height/width too if available and consistent
                # if msg.height != 128 or msg.width != 128:
                #     self.get_logger().error(f"Mismatched dimensions: H={msg.height}, W={msg.width}")
                return # Skip processing this message

            # Extract image data using the simpler reshape (like DroneMonitor)
            # Assumes data is tightly packed BGR8
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(128, 128, 3)

            # Check encoding format - expecting bgr8 primarily
            if msg.encoding == "bgr8":
                rgb_array = img_array[..., ::-1]  # BGR to RGB conversion
            # Handle rgb8 directly if received
            elif msg.encoding == "rgb8":
                rgb_array = img_array
            else:
                # Log warning but still try BGR conversion as fallback
                self.get_logger().warn(f"Unexpected encoding: {msg.encoding}, treating as BGR")
                rgb_array = img_array[..., ::-1]  # BGR to RGB conversion

            # Update the shared image variable safely
            with self.lock:
                self.latest_image = rgb_array
                self.image_received_flag = True # Set flag once we get the first valid image

            # Optional: Warn if mostly black (like DroneMonitor) - can be noisy
            # if np.mean(rgb_array) < 5:
            #     self.get_logger().warn("Received mostly black image!")

        except Exception as e:
            self.get_logger().error(f"Image processing error in QuadSimEnv: {str(e)}")
        # --- End Updated Image Callback ---

    # --- Helper Methods ---
    def send_obstacle_command(self, obstacleNum):
        msg = Float64()
        msg.data = float(obstacleNum)
        self.get_logger().info(f"Attempting to publish obstacle count: {obstacleNum}")
        self.obstacle_pub.publish(msg)
        time.sleep(0.1)
        self.get_logger().info(f"Obstacle count {obstacleNum} published.")

    def send_hover_height(self,altitude):
        msg = Float64()
        msg.data = float(altitude)
        self.get_logger().info(f"Attempting to publish height: {altitude}")
        self.hover_pub.publish(msg)
        time.sleep(0.1)

    def _calculate_goal_metrics(self):
        current_pos_cm = self.state['position']
        current_yaw_rad = self.state['attitude'][2] 

        # Horizontal distance (X-Y plane)
        delta_pos_cm = self.goal_position_cm[:2] - current_pos_cm[:2]
        self.distance_to_goal_cm = np.linalg.norm(delta_pos_cm)

        # Global angle from drone's +X axis to goal (in radians)
        global_angle_to_goal_rad = np.arctan2(delta_pos_cm[1], delta_pos_cm[0])

        # Local angle relative to drone's heading (in radians)
        self.local_angle_rad = global_angle_to_goal_rad - current_yaw_rad

        # Wrap angle to [-pi, pi]
        self.local_angle_rad = (self.local_angle_rad + np.pi) % (2 * np.pi) - np.pi

        # Also store degrees for reward calculation and info
        self.local_angle_deg = np.rad2deg(self.local_angle_rad)

    # --- Gym Methods ---
    def get_observation(self):
        with self.lock:
            vel_cmps = self.state['velocity']
            dist_cm = self.distance_to_goal_cm
            angle_rad = self.local_angle_rad

            raw_obs = np.array([
                vel_cmps[0], vel_cmps[1], vel_cmps[2],
                dist_cm,
                np.cos(angle_rad),
                np.sin(angle_rad)
            ], dtype=np.float32)
            normalized_obs = (raw_obs - OBS_NORM_LOC) / OBS_NORM_SCALE
            normalized_obs = np.clip(normalized_obs, -5.0, 5.0) 

        return normalized_obs

    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.get_logger().info("Resetting environment...")
        reset_msg = Empty()     
        self.reset_pub.publish(reset_msg)

        with self.lock:
            self.state['velocity'] = np.zeros(3, dtype=np.float32)
            self.state['position'] = np.zeros(3, dtype=np.float32)
            self.state['attitude'] = np.zeros(3, dtype=np.float32)
            self.steps = 0
            self.last_state_update_time = self.get_clock().now()

            if np.any(self.goal_position_cm):
                 self._calculate_goal_metrics()
                 self.get_logger().info(f"Recalculated goal metrics after reset using last known goal. New Dist: {self.distance_to_goal_cm:.1f} cm, New Angle: {self.local_angle_deg:.1f} deg")
            else:
                 self.distance_to_goal_cm = 0.0
                 self.local_angle_rad = 0.0
                 self.local_angle_deg = 0.0
                 self.get_logger().info("No goal position known during reset. Metrics remain zero.")

        time.sleep(0.5)
        self.send_obstacle_command(self.obstacle_count)

        self.get_logger().info("Environment reset complete.")
        observation = self.get_observation()
        info = self._get_info()
        return observation, info
    
    def step(self, action):
        """Executes one step in the environment."""
        # --- Action Smoothing ---
        forward_vel_cmd = action[0] * MAX_FORWARD_SPEED_CMD # cm/s
        yaw_rate_cmd = action[1] * MAX_YAW_RATE_CMD      # rad/s

        cmd_vel = Twist()
        cmd_vel.linear.x = float(forward_vel_cmd / 100.0)
        cmd_vel.linear.y = 0.0 # No sideways command
        cmd_vel.linear.z = 0.0 # No vertical command (assuming horizontal navigation focus)
        cmd_vel.angular.z = float(yaw_rate_cmd) # Already in rad/s
        self.cmd_vel_pub.publish(cmd_vel)

        time.sleep(0.02) # Allow time for state update

        # State update happens in odometry_callback, which also calls _calculate_goal_metrics
        # We retrieve the latest calculated values here
        with self.lock:
            current_dist_cm = self.distance_to_goal_cm
            current_angle_deg = self.local_angle_deg # Using degrees for reward calculation
            # We need absolute angle for the penalty function
            current_abs_angle_deg = abs(current_angle_deg)

        observation = self.get_observation()
        self.steps += 1
        
        # --- Calculate Reward (Using reward_fn_angle_penalty logic) ---
        reward = 0.0
        reward -= current_dist_cm / REWARD_DIST_SCALE_CM
        reward -= current_abs_angle_deg / REWARD_ANGLE_SCALE_DEG

        terminated = False
        truncated = False

        if current_dist_cm < 50.0: # Goal reached threshold (e.g., 50 cm)
            terminated = True
            reward += 10.0 
            self.get_logger().info(f"Goal reached! Distance: {current_dist_cm:.1f} cm")
        elif self.steps >= 512: # Max steps per episode
            truncated = True
            self.get_logger().info(f"Episode truncated after {self.steps} steps.")
        # Add other termination conditions if needed (e.g., collision, out of bounds)

        info = self._get_info() # Get auxiliary info

        return observation, reward, terminated, truncated, info

    def _get_info(self):
        with self.lock:
            info = {
                'raw_position_cm': self.state['position'].tolist(),
                'raw_velocity_cmps': self.state['velocity'].tolist(),
                'raw_attitude_rad': self.state['attitude'].tolist(),
                'goal_position_cm': self.goal_position_cm.tolist(),
                'distance_to_goal_cm': self.distance_to_goal_cm,
                'local_angle_deg': self.local_angle_deg,
                'local_angle_rad': self.local_angle_rad,
                'steps': self.steps
            }
        return info
    
    def close(self):
        self.get_logger().info("Closing environment and shutting down ROS node.")
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        time.sleep(0.1)

        self.destroy_node()



def main_test(args=None):
    rclpy.init(args=args)
    print("--- Starting ROS2 Communication Test ---")
    env_test_node = QuadSimEnv()
    print("QuadSimEnv node initialized. Subscribers are listening, Publishers are ready.")
    print("Waiting for connections...")
    time.sleep(3.0)
    # --- Matplotlib Setup ---
    plt.ion()
    fig = plt.figure(figsize=(12, 7)) # Adjusted figure size maybe
    gs = gridspec.GridSpec(1, 3, width_ratios=[2, 2, 1], figure=fig) 
    ax_img = fig.add_subplot(gs[0, 0:2]) # Span columns 0 and 1
    img_display = ax_img.imshow(env_test_node.latest_image) # Show initial image
    ax_img.set_title("Camera Feed")
    ax_img.axis('off')

    # Axes for Status Table (takes last column)
    ax_status = fig.add_subplot(gs[0, 2]) # Column 2
    ax_status.set_title("Drone State")
    ax_status.axis('off') # Hide axes lines and ticks for the table area

    fig.suptitle('Drone State Monitor', fontsize=16)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust layout slightly
    plt.show(block=False)
    status_table = None

    try:
        print("\n--- Testing Python Publishers ---")

        env_test_node.obstacle_count = 50
        test_obstacle_count = env_test_node.obstacle_count
        print(f"Sending /obstacles command with count: {test_obstacle_count}...")
        env_test_node.send_obstacle_command(test_obstacle_count)
        env_test_node.send_hover_height(650)
        time.sleep(0.5) 


        print("Sending /cmd_vel commands (Forward Vel + Yaw Rate)...")
        test_actions = [[50.0, 0.5], [-50.0, -0.5], [0.0, 1.0], [0.0, -1.0], [25.0, 0.0]]
        for action in test_actions:
            forward_vel_cmd = action[0] 
            yaw_rate_cmd = action[1] 

            cmd_vel = Twist()
            cmd_vel.linear.x = float(forward_vel_cmd / 100.0)
            cmd_vel.linear.y = 0.0 
            cmd_vel.linear.z = 0.0 
            cmd_vel.angular.z = float(yaw_rate_cmd)

            env_test_node.cmd_vel_pub.publish(cmd_vel)
            time.sleep(1.5)

        print("Calling env_test_node.reset() method...")
        obs, info = env_test_node.reset()
        print("env_test_node.reset() finished.")

        print("  Sending Zero Velocity command...")
        env_test_node.cmd_vel_pub.publish(Twist())
        time.sleep(1.0)
        print("\n--- Monitoring Python Subscribers (via internal state) ---")
        print("Press Ctrl+C to stop the test.")

        while rclpy.ok():
            with env_test_node.lock:
                pos = env_test_node.state['position'].copy()
                vel = env_test_node.state['velocity'].copy()
                att = env_test_node.state['attitude'].copy()
                img = env_test_node.latest_image.copy()
                img_received = env_test_node.image_received_flag
                dist_to_goal = env_test_node.distance_to_goal_cm
                angle_to_goal = env_test_node.local_angle_deg
                collision_state = env_test_node.collision_state

            img_display.set_data(img)
            ax_img.set_title(f"Camera Feed (Received: {'Yes' if img_received else 'No'})")

            att_r_str = f"{np.rad2deg(att[0]):>6.1f}"
            att_p_str = f"{np.rad2deg(att[1]):>6.1f}"
            att_y_str = f"{np.rad2deg(att[2]):>6.1f}"
            pos_x_str = f"{(pos[0] / 100.0):>7.3f}"
            pos_y_str = f"{(pos[1] / 100.0):>7.3f}"
            pos_z_str = f"{(pos[2] / 100.0):>7.3f}"
            vel_x_str = f"{(vel[0] / 100.0):>7.3f}"
            vel_y_str = f"{(vel[1] / 100.0):>7.3f}"
            vel_z_str = f"{(vel[2] / 100.0):>7.3f}"
            dist_str = f"{dist_to_goal/100:>7.1f}"
            angle_str = f"{angle_to_goal:>7.1f}"
            collision_text = "Collision!!" if collision_state else "No Collision.."

            cell_text = [
                ["Att (deg):", ""],
                ["  Roll (R)", att_r_str],
                ["  Pitch (P)", att_p_str],
                ["  Yaw (Y)", att_y_str],
                ["", ""],
                ["Pos (m):", ""],
                ["  X", pos_x_str],
                ["  Y", pos_y_str],
                ["  Z", pos_z_str],
                ["", ""],
                ["Vel (m/s):", ""],
                ["  Vx", vel_x_str],
                ["  Vy", vel_y_str],
                ["  Vz", vel_z_str],
                ["", ""],
                ["Goal Metrics:", ""],
                ["  Distance (cm)", dist_str],
                ["  Local Angle (deg)", angle_str],
                ["Collision:", collision_text]
            ]
            col_labels = ["Parameter", "Value"]

            if status_table:
                status_table.remove()

            ax_status.axis('off')
            status_table = ax_status.table(cellText=cell_text,
                                        colLabels=col_labels,
                                        loc='center',
                                        cellLoc='left',
                                        colWidths=[0.45, 0.45])
            status_table.auto_set_font_size(False)
            status_table.set_fontsize(10)
            status_table.scale(1, 1.3)

            # Set collision cell text color
            for key, cell in status_table.get_celld().items():
                # The collision row is the last row, column index 1
                if key[0] == len(cell_text)-1 and key[1] == 1:
                    cell.get_text().set_color("red" if collision_state else "black")


            fig.canvas.draw_idle()
            plt.pause(0.05)

    except KeyboardInterrupt:
        print("\nCtrl+C received. Shutting down test.")
    except Exception as e:
        print(f"\nAn error occurred in main_test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Closing environment node and plot...")
        plt.close(fig) # Close the plot window
        if env_test_node: # Check if node was created
            env_test_node.close()
        if rclpy.ok():
            try:
                rclpy.shutdown()
                print("rclpy shutdown complete.")
            except Exception as e: print(f"Error during rclpy shutdown: {e}")
        print("Visualization Test Finished.")

# ----------------------------------------------
# Main function for SB3 Training/Loading/Running
# ----------------------------------------------
def main(args=None, run_mode="train"):
    rclpy.init(args=args)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_log_dir = os.path.join(script_dir, "RL_training_ROS2_MLP3")
    checkpoints_dir = os.path.join(base_log_dir, "checkpoints")
    best_model_dir = os.path.join(base_log_dir, "best_model")
    logs_dir = os.path.join(base_log_dir, "logs") 

    # --- Environment Setup (Keep this) ---
    os.makedirs(logs_dir, exist_ok=True) 
    env_unwrapped = QuadSimEnv()
    env = Monitor(env_unwrapped, logs_dir) 
    print("ROS2 Gym environment created and wrapped with Monitor.")

    # --- Model Loading ---
    latest_model = None
    best_model_path = os.path.join(best_model_dir, "best_model.zip") # Change

    if latest_model is None: # Only run auto-detect if specific model wasn't forced/found
        if os.path.exists(best_model_path):
            latest_model = best_model_path
            print(f"Found best model: {latest_model}")
        else:
            checkpoint_files = glob.glob(os.path.join(checkpoints_dir, "quad_ppo_ros2_mlp3_*.zip"))
            if checkpoint_files:
                checkpoint_files.sort(key=os.path.getmtime, reverse=True)
                latest_model = checkpoint_files[0]
                print(f"Found latest checkpoint: {latest_model}")

    if not latest_model:
        print("Error: No model found to load for inference.")
        env.close()
        if rclpy.ok():
            rclpy.shutdown()
        return # Exit if no model loaded

    print(f"Loading model for inference from: {latest_model}")
    model = PPO.load(latest_model, env=env)
    print("Model loaded successfully.")


    # --- EXECUTION MODE ---
    if run_mode == "train":
        print("RUN MODE: Training")
        print("Executing original training block...")
        # --- Hyperparameters matching RL.py PPO settings ---
        learning_rate = 3e-4
        n_steps = 512 # From RL.py collector frames_per_batch
        batch_size = 32 # From RL.py minibatch split
        n_epochs = 10 # From RL.py inner loop range(10)
        gamma = 0.99
        gae_lambda = 0.95
        clip_range = 0.2 # Default PPO clip range
        ent_coef = 0.0 # Often 0 or small positive value
        vf_coef = 0.5 # Default PPO value function coefficient
        max_grad_norm = 0.5 # From RL.py clip_grad_norm_
        policy_kwargs = dict(net_arch=[dict(pi=[128, 128], vf=[128, 128])])
        # Reset loaded model's learning rate etc if needed for further training
        model.learning_rate = learning_rate
        model.n_steps = n_steps
        model.batch_size = batch_size
        model.n_epochs = n_epochs

        total_timesteps = 1_000_000
        print(f"Starting training for {total_timesteps} timesteps...")
        try:
            # Setup Callbacks if training
            checkpoint_callback = CheckpointCallback(...) # Define as before
            eval_callback = EvalCallback(...) # Define as before
            model.learn(
                total_timesteps=total_timesteps,
                callback=[checkpoint_callback, eval_callback],
                log_interval=1,
                reset_num_timesteps=False # Continue timestep count if loaded
            )
            final_model_path = os.path.join(base_log_dir, "final_model_ros2_mlp3.zip")
            model.save(final_model_path)
            print(f"Training finished. Final model saved to: {final_model_path}")
        except KeyboardInterrupt:
            interrupt_model_path = os.path.join(base_log_dir, "interrupted_model_ros2_mlp3.zip")
            model.save(interrupt_model_path)
            print(f"\nTraining interrupted by user. Model saved to: {interrupt_model_path}")


    elif run_mode == "inference":
        # --- Inference Loop ---
        print("RUN MODE: Inference")
        num_episodes = 10 # How many times to run the policy
        max_steps_per_episode = 512 # Match truncation limit

        for episode in range(num_episodes):
            obs, info = env.reset()
            print(f"\n--- Starting Inference Episode {episode + 1}/{num_episodes} ---")
            terminated = False
            truncated = False
            step_count = 0
            total_reward = 0

            while not terminated and not truncated and step_count < max_steps_per_episode:
                # Use deterministic=True for consistent actions during inference
                action, _states = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                step_count += 1

                # Optional: Add a small delay to make it watchable
                time.sleep(0.05)
                # Optional: Print step info
                # print(f"  Step: {step_count}, Action: {action}, Reward: {reward:.3f}")


            print(f"--- Episode {episode + 1} Finished ---")
            print(f"  Reason: {'Goal Reached' if terminated else 'Max Steps/Truncated'}")
            print(f"  Steps: {step_count}")
            print(f"  Total Reward: {total_reward:.3f}")
            print(f"  Final Distance: {info.get('distance_to_goal_cm', 'N/A'):.1f} cm")
            print(f"  Final Angle: {info.get('local_angle_deg', 'N/A'):.1f} deg")

        print("\nInference complete.")
         # --- End Inference Loop ---

    else:
         print(f"Error: Unknown run_mode '{run_mode}'")


    # --- Cleanup (Common to both modes) ---
    print("Cleaning up resources...")
    env.close()
    if rclpy.ok():
        rclpy.shutdown()
    print("ROS2 shutdown complete. Exiting.")

if __name__ == '__main__':
    # main(run_mode="train")
    # main(run_mode="inference") 
    main_test() 

