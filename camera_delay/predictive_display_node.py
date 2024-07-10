#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import random
from collections import deque

class RandomCameraDelayNode(Node):

    def __init__(self):
        super().__init__('random_camera_delay_node')
        
        self.declare_parameter('min_delay_ms', 200)
        self.declare_parameter('max_delay_ms', 800)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_delayed')
        self.declare_parameter('change_delay_interval_sec', 10)  # Change delay every 10 seconds

        self.min_delay_ms = self.get_parameter('min_delay_ms').get_parameter_value().integer_value
        self.max_delay_ms = self.get_parameter('max_delay_ms').get_parameter_value().integer_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.change_delay_interval_sec = self.get_parameter('change_delay_interval_sec').get_parameter_value().integer_value

        self.queue = deque()
        self.current_delay = self.generate_random_delay()

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, output_topic, 10)

        self.timer = self.create_timer(0.01, self.publish_delayed_message)
        self.delay_change_timer = self.create_timer(self.change_delay_interval_sec, self.update_delay)

        self.bridge = CvBridge()

        self.get_logger().info(f'Random camera delay node started with random delays between {self.min_delay_ms} and {self.max_delay_ms} ms.')
        self.get_logger().info(f'Change delay every {self.change_delay_interval_sec} seconds.')

    def generate_random_delay(self):
        delay = random.uniform(self.min_delay_ms / 1000.0, self.max_delay_ms / 1000.0)
        self.get_logger().info(f'Generated new delay: {delay * 1000:.2f} ms')
        return delay

    def update_delay(self):
        self.current_delay = self.generate_random_delay()

    def listener_callback(self, msg):
        current_time = time.monotonic()
        self.queue.append((current_time + self.current_delay, msg))
        self.get_logger().info(f'Introduced delay: {self.current_delay * 1000:.2f} ms')

    def publish_delayed_message(self):
        current_time = time.monotonic()
        while self.queue and (current_time >= self.queue[0][0]):
            _, msg = self.queue.popleft()
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                overlay_image = self.add_delay_bar(cv_image)
                overlay_image_msg = self.bridge.cv2_to_imgmsg(overlay_image, encoding='bgr8')
                self.publisher.publish(overlay_image_msg)
            except CvBridgeError as e:
                self.get_logger().info('Failed to convert and publish image: %s' % str(e))

    def add_delay_bar(self, image):
        delay_percentage = (self.current_delay * 1000 - self.min_delay_ms) / (self.max_delay_ms - self.min_delay_ms)
        height, width, _ = image.shape
        bar_width = 50
        bar_height = int(height * delay_percentage)

        bar = np.zeros((height, bar_width, 3), dtype=np.uint8)
        bar[height - bar_height:, :] = [0, 0, 255]  # Red color for the bar

        # Add delay text
        cv2.putText(image, f'{self.current_delay * 1000:.2f} ms', (width - bar_width - 150, height - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

        overlay_image = np.hstack((image, bar))
        return overlay_image

def main(args=None):
    rclpy.init(args=args)
    node = RandomCameraDelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
