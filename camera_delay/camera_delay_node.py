#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time
from collections import deque

class CameraDelayNode(Node):

    def __init__(self):
        super().__init__('camera_delay_node')
        
        self.declare_parameter('delay_seconds', 1.0)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_delayed')

        delay_seconds = self.get_parameter('delay_seconds').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.queue = deque()
        self.delay_seconds = delay_seconds

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, output_topic, 10)

        self.timer = self.create_timer(0.01, self.publish_delayed_message)

        self.get_logger().info(f'Camera delay node started with {self.delay_seconds} seconds delay.')

    def listener_callback(self, msg):
        current_time = time.monotonic()
        self.queue.append((current_time, msg))

    def publish_delayed_message(self):
        current_time = time.monotonic()
        while self.queue and (current_time - self.queue[0][0] >= self.delay_seconds):
            _, msg = self.queue.popleft()
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraDelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
