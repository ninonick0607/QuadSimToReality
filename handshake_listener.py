#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt

class DroneMonitor(Node):
    def __init__(self):
        super().__init__('drone_monitor')
        
        # Configure QoS profiles
        position_qos = QoSProfile(
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
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/quad_position/camera/image',
            self.image_callback,
            image_qos
        )
        
        self.pos_sub = self.create_subscription(
            Point,
            '/quad_position/quad_position',
            self.position_callback,
            position_qos
        )
        
        # Initialize displays (FIXED LINE)
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

    def image_callback(self, msg):
        try:
            now = self.get_clock().now()
            dt = (now - self.last_image_time).nanoseconds / 1e9
            self.last_image_time = now
            self.image_count += 1
            
            # Process image data
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(128, 128, 3)
            
            # Convert BGR to RGB (Unreal's format)
            rgb_array = img_array[:, :, ::-1]  # Reverse color channels
            
            # Update image display
            self.img_display.set_data(rgb_array)
            self.ax_img.set_title(f"Frame: {self.image_count} ({1/dt:.1f} FPS)")
            
            # Check for black images
            if np.mean(rgb_array) < 5:
                self.get_logger().warn("Received mostly black image!")
            
            plt.draw()
            plt.pause(0.001)

        except Exception as e:
            self.get_logger().error(f"Image error: {str(e)}")

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
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info(f"Received {monitor.image_count} images")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
