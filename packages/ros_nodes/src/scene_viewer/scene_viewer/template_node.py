#!/usr/bin/env python3
#
# Copyright 2021 Human Dataware Lab. Cc. Ltd.
# Created by Daiki Hayashi <hayashi.daiki@hdwlab.co.jp>
#

"""Template node."""

import fire
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    """Example publisher."""

    def __init__(self, message: str):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.message = message
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.message}: {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(message: str):
    """ROS node."""
    rclpy.init()

    minimal_publisher = MinimalPublisher(message)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


def cli():
    """Command-line interface."""
    fire.Fire(main)


if __name__ == "__main__":
    cli()
