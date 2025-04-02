#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from example_interfaces.msg import Float64

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import threading
import time
import sys
import os # Added for path operations
import glob # Added for finding model files

# Stable Baselines 3 imports
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback # Removed BaseCallback as it's not used now
from stable_baselines3.common.monitor import Monitor

# --- QuadSimEnv definition ---
class QuadSimEnv(Node, gym.Env):
    """
    Gymnasium environment integrated with ROS2 for drone control in Unreal Engine.
    Receives state via ROS topics and sends commands.
    Integrates Stable Baselines 3 for RL training.
    """
    def __init__(self):
        # Initialize both Node and gym.Env
        Node.__init__(self, 'quad_sim_env_rl') # Renamed node slightly to avoid conflicts if old main runs
        gym.Env.__init__(self)
        self.get_logger().info("Initializing QuadSim ROS2 Gym Environment...")

        # --- ROS2 Setup ---
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # Keep BEST_EFFORT for high-frequency image data
            depth=10, # Reduced depth slightly, adjust if needed
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscribers
        self.position_sub = self.create_subscription(
            Point, '/drone/position', self.position_callback, reliable_qos
        )
        self.goal_sub = self.create_subscription(
            Point, '/goal/position', self.goal_callback, reliable_qos
        )
        # Image sub is kept but not used in observation/reward for now
        self.image_sub = self.create_subscription(
            Image, '/camera/image', self.image_callback, image_qos
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', reliable_qos)
        self.reset_pub = self.create_publisher(String, '/reset', reliable_qos)
        self.obstacle_pub = self.create_publisher(
            Float64, '/obstacles',
            QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        )
        # --- End ROS2 Setup ---

        # --- Gym Environment Setup ---
        # Action space: only controlling Z velocity [-1, 1]
        self.action_space = spaces.Box(
            low=np.array([-1.0]),
            high=np.array([1.0]),
            dtype=np.float32
        )

        # Observation space: [z_velocity, z_distance_to_goal] - Normalized
        self.observation_space = spaces.Box(
            low=np.array([-1.0, -1.0]),
            high=np.array([1.0, 1.0]),
            dtype=np.float32
        )
        # --- End Gym Environment Setup ---

        # --- State Variables ---
        self.current_position = np.zeros(3, dtype=np.float32)
        self.current_velocity = np.zeros(3, dtype=np.float32)
        self.goal_position = np.zeros(3, dtype=np.float32) # Goal received from topic
        self.latest_image = np.zeros((128, 128, 3), dtype=np.uint8) # Default image
        self.last_update_time = self.get_clock().now()
        self.target_z = 1000.0 # Assuming a fixed target Z for now, like in quadsimenv.py reward. Can be overridden by goal_pos[2] if needed.
        self.prev_z_distance = self.calculate_z_distance() # Initialize prev distance

        # RL specific state
        self.prev_action = 0.0 # For action smoothing
        self.steps = 0
        self.lock = threading.Lock() # Protect access to shared state variables
        # --- End State Variables ---

        # Start ROS spinning in a separate thread
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()
        self.get_logger().info("ROS2 spinning started in background thread.")
        time.sleep(1.0) # Allow time for initial connections

    # --- ROS Callback Methods ---
    def position_callback(self, msg):
        with self.lock:
            now = self.get_clock().now()
            dt = (now - self.last_update_time).nanoseconds / 1e9
            new_position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)

            if dt > 0.001: # Avoid division by zero or tiny dt
                self.current_velocity = (new_position - self.current_position) / dt
            else:
                # Keep previous velocity if dt is too small
                pass # self.current_velocity remains unchanged

            self.current_position = new_position
            self.last_update_time = now

    def goal_callback(self, msg):
        with self.lock:
            # Update goal position AND the target_z used for reward calculation
            self.goal_position = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
            self.target_z = self.goal_position[2] # Use the Z from the goal topic
            # self.get_logger().info(f"Goal updated: {self.goal_position}, Target Z: {self.target_z}")

    def image_callback(self, msg):
        # Minimal processing for now, as image isn't used in observation
        try:
            # Check if image shape might change later, currently unused.
            pass # Basic check done in original code is fine.
            # Potentially update self.latest_image if needed for other purposes
            # Consider adding locking if latest_image is accessed elsewhere critically
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    # --- Helper Methods ---
    def send_obstacle_command(self, obstacleNum):
        msg = Float64()
        msg.data = float(obstacleNum)
        self.get_logger().info(f"Attempting to publish obstacle count: {obstacleNum}")
        self.obstacle_pub.publish(msg)
        # Adding a small delay to increase likelihood of message being sent before shutdown/next step
        time.sleep(0.1)
        self.get_logger().info(f"Obstacle count {obstacleNum} published.")

    def calculate_z_distance(self):
        # Calculates distance based on current state
        with self.lock:
            # Use self.target_z which is updated by goal_callback
            dist = abs(self.current_position[2] - self.target_z)
        return dist

    # --- Gym Methods ---
    def get_observation(self):
        """Calculates the observation based on current state."""
        with self.lock:
            # Normalize velocity: More sensitive around typical operating range
            z_vel = np.clip(self.current_velocity[2] / 100.0, -1.0, 1.0)

            # Normalize distance to goal: Scaled based on expected range
            # Using self.target_z which comes from the /goal/position topic
            z_dist_to_goal = (self.target_z - self.current_position[2]) / 2000.0 # Example scaling factor
            z_dist_to_goal = np.clip(z_dist_to_goal, -1.0, 1.0)

        return np.array([z_vel, z_dist_to_goal], dtype=np.float32)

    def reset(self, seed=None, options=None):
        """Resets the environment to a starting state."""
        super().reset(seed=seed) # Necessary for Gymnasium compatibility
        self.get_logger().info("Resetting environment...")

        # Send ROS reset command
        reset_msg = String()
        reset_msg.data = "reset"
        self.reset_pub.publish(reset_msg)

        # Reset internal RL state variables
        with self.lock:
            # Re-read position/velocity after reset command has potentially taken effect
            # Note: Actual drone position reset happens in Unreal/ROS,
            # we just wait briefly and reset internal tracking vars.
            self.current_position = np.zeros(3, dtype=np.float32) # Assume reset goes to 0,0,0
            self.current_velocity = np.zeros(3, dtype=np.float32)
            self.prev_action = 0.0
            self.steps = 0
            self.last_update_time = self.get_clock().now() # Reset timer

        # Allow time for reset command to propagate and state to potentially update
        time.sleep(0.5) # Adjusted wait time, might need tuning

        with self.lock:
            # Recalculate initial distance after reset
            self.prev_z_distance = self.calculate_z_distance()

        self.get_logger().info("Environment reset complete.")
        observation = self.get_observation()
        info = self._get_info() # Get initial info dict

        return observation, info

    def step(self, action):
        """Executes one step in the environment."""
        # --- Action Smoothing ---
        # Apply smoothing similar to quadsimenv.py for smoother control
        # Adjust factors (0.7, 0.3) if needed
        current_action_z = action[0]
        self.prev_action = 0.7 * self.prev_action + 0.3 * current_action_z
        smoothed_z_velocity_cmd = self.prev_action * 130.0 # Scale action to velocity cmd
        # --- End Action Smoothing ---

        # Publish velocity command via ROS
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0 # Only controlling Z for now
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = float(smoothed_z_velocity_cmd)
        cmd_vel.angular.z = 0.0 # No yaw control
        self.cmd_vel_pub.publish(cmd_vel)

        # Allow some time for the command to be processed and state to update via callbacks
        # This replaces the handle_data() call and fixed loops from ZeroMQ version
        time.sleep(0.02) # Small delay, adjust as needed based on system performance

        # Observation is calculated based on the *new* state received via callbacks
        observation = self.get_observation()
        self.steps += 1

        # --- Calculate Reward (adopted from quadsimenv.py structure) ---
        with self.lock:
            current_z = self.current_position[2]
            z_velocity = self.current_velocity[2]
            # target_z is updated by goal_callback

        z_distance = abs(current_z - self.target_z)

        reward = 0.0
        # Base time penalty (encourages efficiency)
        reward -= 0.1

        # Distance component (positive reward for being closer)
        # +5 at target (dist=0), 0 when 500+ cm away. Clamp distance term.
        dist_reward_component = 5.0 * (1.0 - min(1.0, z_distance / 500.0))
        reward += dist_reward_component

        # Progress reward (positive for getting closer than previous step)
        progress = self.prev_z_distance - z_distance # Positive if distance decreased
        reward += progress * 2.0 # Substantial reward for making progress

        # Target achievement bonus (big reward for being very close)
        if z_distance < 30.0: # Threshold from quadsimenv.py
            reward += 10.0

        # Efficiency penalty (penalize excessive velocity)
        # Calculate a reasonable target velocity based on distance
        optimal_velocity = min(80.0, max(5.0, z_distance * 0.1)) # Target 5-80 cm/s
        inefficiency = abs(abs(z_velocity) - optimal_velocity)
        # Small penalty, max -1.0 if inefficiency is 50cm/s or more
        reward -= min(1.0, inefficiency / 50.0)
        # --- End Calculate Reward ---


        # Update previous distance for next step's progress calculation
        self.prev_z_distance = z_distance

        # --- Termination Condition ---
        # Terminate after a fixed number of steps
        terminated = self.steps >= 512 # Max steps per episode
        truncated = False # Using terminated for step limit based on Gym v26

        # --- Info Dictionary ---
        info = self._get_info()

        if terminated:
             self.get_logger().info(f"Episode terminated after {self.steps} steps.")

        return observation, reward, terminated, truncated, info

    def _get_info(self):
        """Returns dictionary with auxiliary information."""
        with self.lock:
            info = {
                'z_position': self.current_position[2],
                'z_velocity': self.current_velocity[2],
                'z_target': self.target_z,
                'z_distance': self.calculate_z_distance(),
                'goal_position': self.goal_position.tolist(), # Use current goal
                'current_position': self.current_position.tolist(),
                'steps': self.steps
            }
        return info

    def close(self):
        """Clean up resources."""
        self.get_logger().info("Closing environment and shutting down ROS node.")
        # Clean up ROS node resources
        self.destroy_node()
        # Note: rclpy.shutdown() should be called externally after environment is closed


# ----------------------------------------------
# Main function for SB3 Training/Loading/Running
# (Adopted from quadsimenv.py)
# ----------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    # --- Configuration ---
    # Define directories for logs, checkpoints, and the best model
    script_dir = os.path.dirname(os.path.abspath(__file__)) # Get dir of this script
    base_log_dir = os.path.join(script_dir, "RL_training_ROS2") # Base directory
    checkpoints_dir = os.path.join(base_log_dir, "checkpoints")
    best_model_dir = os.path.join(base_log_dir, "best_model")
    logs_dir = os.path.join(base_log_dir, "logs") # For TensorBoard

    # Create directories if they don't exist
    os.makedirs(checkpoints_dir, exist_ok=True)
    os.makedirs(best_model_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)

    # --- Environment Setup ---
    # Create the custom ROS2 Gym environment
    env_unwrapped = QuadSimEnv()

    # Send obstacle command (optional, based on original QuadRL_env main)
    obstacle_count = 150 # Default value
    if len(sys.argv) > 1:
        try:
            obstacle_count = int(sys.argv[1])
            print(f"Using obstacle count from command line: {obstacle_count}")
        except ValueError:
            print(f"Invalid obstacle count: {sys.argv[1]}, using default: {obstacle_count}")
    env_unwrapped.send_obstacle_command(obstacle_count)
    time.sleep(0.5) # Give time for command to be processed

    # Wrap the environment with Monitor for SB3 logging (rewards, episode lengths)
    env = Monitor(env_unwrapped, logs_dir)
    print("ROS2 Gym environment created and wrapped with Monitor.")

    # --- Callbacks Setup ---
    # Checkpoint callback: Saves the model periodically
    checkpoint_callback = CheckpointCallback(
        save_freq=10000, # Save every 10k steps (adjust as needed)
        save_path=checkpoints_dir,
        name_prefix="quad_ppo_ros2",
        save_replay_buffer=True, # Set to False if not using off-policy algos
        save_vecnormalize=True, # Set to False if not using VecNormalize wrapper
        verbose=1
    )

    # Evaluation callback: Evaluates the model periodically and saves the best one
    eval_callback = EvalCallback(
        env, # Use the monitored environment for evaluation
        best_model_save_path=best_model_dir,
        log_path=logs_dir,
        eval_freq=5000, # Evaluate every 5k steps
        n_eval_episodes=5, # Number of episodes to run for evaluation
        deterministic=True, # Use deterministic actions for evaluation
        render=False, # Cannot render directly from ROS env this way
        verbose=1
    )
    print("Stable Baselines3 callbacks configured.")

    # --- Model Loading / Creation ---
    latest_model = None
    best_model_path = os.path.join(best_model_dir, "best_model.zip")

    # 1. Check for the best model saved by EvalCallback
    if os.path.exists(best_model_path):
        latest_model = best_model_path
        print(f"Found best model: {latest_model}")
    else:
        # 2. If no best model, check for the latest checkpoint
        checkpoint_files = glob.glob(os.path.join(checkpoints_dir, "quad_ppo_ros2_*.zip"))
        if checkpoint_files:
            # Sort by modification time (most recent first)
            checkpoint_files.sort(key=os.path.getmtime, reverse=True)
            latest_model = checkpoint_files[0]
            print(f"Found latest checkpoint: {latest_model}")

    # Create or load the PPO model
    if latest_model:
        print(f"Loading existing model from: {latest_model}")
        model = PPO.load(
            latest_model,
            env=env, # Pass the env to continue training
            tensorboard_log=logs_dir,
            # You might need to re-specify learning rate or other hyperparameters if they changed
            # learning_rate=2e-4, # Example: uncomment to force learning rate
        )
        # Reset timesteps count? SB3 usually handles this, but check if needed.
        # model.num_timesteps = 0 # Uncomment if you want to reset timestep counter for logging
    else:
        print("No existing model found. Creating a new PPO model.")
        # Define PPO hyperparameters (adjust as needed)
        model = PPO(
            "MlpPolicy",            # Policy network type
            env,                    # The environment to train on
            policy_kwargs=dict(net_arch=[dict(pi=[128, 128], vf=[128, 128])]), # Network architecture (adjust size if needed)
            learning_rate=3e-4,     # Learning rate (often 3e-4 or 1e-4)
            n_steps=2048,           # Steps per PPO update (rollout buffer size)
            batch_size=64,          # Minibatch size for optimization
            n_epochs=10,            # Number of optimization epochs per PPO update
            gamma=0.99,             # Discount factor
            gae_lambda=0.95,        # Factor for GAE (Generalized Advantage Estimation)
            clip_range=0.2,         # PPO clipping parameter
            ent_coef=0.0,           # Entropy coefficient (0 means no entropy bonus)
            vf_coef=0.5,            # Value function coefficient
            max_grad_norm=0.5,      # Max gradient norm for clipping
            verbose=1,              # Verbosity level (1=info, 0=quiet, 2=debug)
            tensorboard_log=logs_dir # Directory for TensorBoard logs
        )
        print("New PPO model created.")

    # --- Training ---
    total_timesteps = 1_000_000 # Set total number of steps for training (e.g., 1 million)
    print(f"Starting training for {total_timesteps} timesteps...")
    try:
        # Train the model with both callbacks
        model.learn(
            total_timesteps=total_timesteps,
            callback=[checkpoint_callback, eval_callback],
            log_interval=1, # Log TensorBoard stats every update
            reset_num_timesteps=False # Set to True if you loaded a model but want logs to start from 0
        )
        # Save the final model after training is complete
        final_model_path = os.path.join(base_log_dir, "final_model_ros2.zip")
        model.save(final_model_path)
        print(f"Training finished. Final model saved to: {final_model_path}")

    except KeyboardInterrupt:
        # Save the model if training is interrupted manually
        interrupt_model_path = os.path.join(base_log_dir, "interrupted_model_ros2.zip")
        model.save(interrupt_model_path)
        print(f"\nTraining interrupted by user. Model saved to: {interrupt_model_path}")
    finally:
        # --- Cleanup ---
        print("Cleaning up resources...")
        # Close the environment (which also destroys the ROS node)
        env.close() # This calls env_unwrapped.close() -> destroy_node()
        # Shutdown ROS client library
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS2 shutdown complete. Exiting.")


if __name__ == '__main__':
    main()