#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
from example_interfaces.msg import Float64
import time
import matplotlib.pyplot as plt
import sys

class DroneMonitor(Node):
    def __init__(self):
        super().__init__('drone_monitor')
        
        # Configure QoS profiles
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

        # Initialize matplotlib figure
        self.fig, (self.ax_img, self.ax_pos) = plt.subplots(1, 2, figsize=(12, 6))
        plt.ion()
        
        self.get_logger().info("Starting DroneMonitor node")
        self.get_logger().info(f"Subscribing to topics: /camera/image, /position")
        self.get_logger().info(f"Publishing to topic: /obstacles")
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            image_qos
        )
        
        self.pos_sub = self.create_subscription(
            Point,
            '/position',
            self.position_callback,
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
        # Initialize displays
        self.img_display = self.ax_img.imshow(np.zeros((128, 128, 3)),
                                             interpolation='nearest')
        self.ax_img.axis('off')
        self.ax_img.set_title("Camera Feed")
        
        self.pos_text = self.ax_pos.text(0.5, 0.5, 'Waiting for data...', 
                                       ha='center', va='center', fontsize=12)
        self.ax_pos.axis('off')
        self.ax_pos.set_title("Drone Position")
        
        self.image_count = 0
        self.last_image_time = self.get_clock().now()


    def send_obstacle_command(self, obstacleNum):
        msg = Float64()
        msg.data = float(obstacleNum)
        
        # Send once with confirmation
        self.obstacle_pub.publish(msg)
        self.get_logger().info(f"Sent obstacle count: {obstacleNum}")
        
        # Verify publication
        print(f"\n--- PUBLISHED OBSTACLE COUNT: {obstacleNum} ---\n")
        print(f"Verify with: ros2 topic echo /obstacles")

    def image_callback(self, msg):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_image_time).nanoseconds / 1e9
            self.last_image_time = current_time
            
            # Check if image dimensions match expected
            if len(msg.data) != 128*128*3:
                self.get_logger().error(f"Invalid image dimensions: got {len(msg.data)} bytes, expected {128*128*3}")
                return
                
            # Extract image data
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(128, 128, 3)
            
            # Check encoding format - Now expecting bgr8
            if msg.encoding == "bgr8":
                rgb_array = img_array[:, :, ::-1]  # BGR to RGB conversion
            else:
                self.get_logger().warn(f"Unexpected encoding: {msg.encoding}, treating as BGR")
                rgb_array = img_array[:, :, ::-1]  # BGR to RGB conversion
            
            # Update image display
            self.img_display.set_data(rgb_array)
            fps = 1.0 / dt if dt > 0 else 0
            self.ax_img.set_title(f"Frame: {self.image_count} ({fps:.1f} FPS)")
            self.image_count += 1
            
            # Check for mostly black image
            if np.mean(rgb_array) < 5:
                self.get_logger().warn("Received mostly black image!")
            
            plt.draw()
            plt.pause(0.001)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")

    def position_callback(self, msg):
        pos_str = f"Position (m):\nX: {msg.x:.2f}\nY: {msg.y:.2f}\nZ: {msg.z:.2f}"
        self.pos_text.set_text(pos_str)
        plt.draw()
        plt.pause(0.001)

    def destroy_node(self):
        plt.close('all')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    monitor = DroneMonitor()
    
    # Wait for connections to establish
    time.sleep(2)
    
    # Get obstacle count from command line if provided
    obstacle_count = 150
    if len(sys.argv) > 1:
        try:
            obstacle_count = int(sys.argv[1])
        except ValueError:
            print(f"Invalid obstacle count: {sys.argv[1]}, using default: 150")
    
    # Send obstacle command
    monitor.send_obstacle_command(obstacle_count)
    
    # Spin until interrupted
    try:
        print("Node running. Press Ctrl+C to exit.")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info(f"Received {monitor.image_count} images")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()