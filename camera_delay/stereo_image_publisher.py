#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoImagePublisher(Node):
    def __init__(self):
        super().__init__('stereo_image_publisher')

        # Subscriber for left and right camera image topics
        self.left_image_sub = self.create_subscription(Image, '/left_camera/image_raw', self.left_image_callback, 10)
        self.right_image_sub = self.create_subscription(Image, '/right_camera/image_raw', self.right_image_callback, 10)

        # Publisher for the combined stereo image
        self.stereo_image_pub = self.create_publisher(Image, '/stereo_camera/image_raw', 10)

        # Hold the latest images
        self.left_image = None
        self.right_image = None
        self.bridge = CvBridge()

    def left_image_callback(self, msg):
        self.get_logger().info('Received left image')
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.publish_stereo_image()

    def right_image_callback(self, msg):
        self.get_logger().info('Received right image')
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.publish_stereo_image()

    def publish_stereo_image(self):
        if self.left_image is not None and self.right_image is not None:
            # Combine the images side by side
            stereo_image = np.hstack((self.left_image, self.right_image))
            stereo_msg = self.bridge.cv2_to_imgmsg(stereo_image, "bgr8")
            self.stereo_image_pub.publish(stereo_msg)
            self.get_logger().info('Published stereo image')
        else:
            self.get_logger().warn('Images not yet received for both cameras')

def main(args=None):
    rclpy.init(args=args)
    stereo_image_publisher = StereoImagePublisher()
    rclpy.spin(stereo_image_publisher)
    stereo_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
