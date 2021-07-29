#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf_utils import TFUtils


class TestNode(Node):
    def __init__(self):
        super().__init__('tf_lookup')

        self.tf_utils = TFUtils(self.node, False)

        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self, ):
        transform = self.tf_utils.lookup_transform(
            target_frame='base_link',
            source_frame='odom',
            convert=False,
            when=None)
        print(transform)


def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
