#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.declare_parameter('change_delay_rate', 10)  # Change delay every 10 messages

        self.min_delay_ms = self.get_parameter('min_delay_ms').get_parameter_value().integer_value
        self.max_delay_ms = self.get_parameter('max_delay_ms').get_parameter_value().integer_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        change_delay_rate = self.get_parameter('change_delay_rate').get_parameter_value().integer_value

        self.queue = deque()
        self.change_delay_rate = change_delay_rate
        self.message_count = 0
        self.current_delay = self.generate_random_delay()

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, output_topic, 10)

        self.timer = self.create_timer(0.01, self.publish_delayed_message)

        self.get_logger().info(f'Random camera delay node started with random delays between {self.min_delay_ms} and {self.max_delay_ms} ms.')
        self.get_logger().info(f'Change delay every {self.change_delay_rate} messages.')

    def generate_random_delay(self):
        delay = random.uniform(self.min_delay_ms / 1000.0, self.max_delay_ms / 1000.0)
        self.get_logger().info(f'Generated new delay: {delay * 1000:.2f} ms')
        return delay

    def listener_callback(self, msg):
        self.message_count += 1
        if self.message_count >= self.change_delay_rate:
            self.current_delay = self.generate_random_delay()
            self.message_count = 0
        current_time = time.monotonic()
        self.queue.append((current_time + self.current_delay, msg))
        self.get_logger().info(f'Introduced delay: {self.current_delay * 1000:.2f} ms')

    def publish_delayed_message(self):
        current_time = time.monotonic()
        while self.queue and (current_time >= self.queue[0][0]):
            _, msg = self.queue.popleft()
            self.publisher.publish(msg)
            self.get_logger().info(f'Published message with delay at {time.monotonic() - current_time:.2f} seconds')

def main(args=None):
    rclpy.init(args=args)
    node = RandomCameraDelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
