#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class AutomateTurtlebot(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(AutomateTurtlebot, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)
        # TODO: change pub topic name -> Done
        self.control_publisher = self.create_publisher(Twist, "/cmd_vel", qos)
        # TODO: change sub topic ->
        self.create_subscription(LaserScan, "/scan", self.subscribe_callback, qos)

    def subscribe_callback(self, msg):
        """
        this function is executed every subscription.
        """
        try:
            min_distance = self.calc_distance_to_wall(msg)
            if min_distance < 0.1:
                pub_msg = self.create_turnleft_msg()
            else:
                pub_msg = self.create_forward_msg()
            self.control_publisher.publish(pub_msg)
        except Exception as e:
            print(e)

        finally:
            stop_msg = self.create_stop_msg()
            self.control_publisher.publish(stop_msg)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    def calc_distance_to_wall(self, LaserScan) -> float:
        # TODO implement get min distance from LaserScan -> Done
        # NOTE: use range_min -> I did this similarly to the Twist
        msg = LaserScan
        msg.min_distance = min(self.scan_ranges)
        return msg

    def create_turnleft_msg(self, Twist) -> Twist:
        # TODO: implement here -> tried some values, I will change this when I see how it moves
        msg = Twist()
        msg.linear.x = 0,2
        msg.linear.y = -0,5
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg

    def create_forward_msg(self, Twist) -> Twist:
        # TODO: implement here -> done
        msg = Twist()
        msg.linear.x = 1
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg

    def create_stop_msg(self) -> Twist:
        # TODO: implement here -> Done
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        return msg


def main():
    rclpy.init()
    node = AutomateTurtlebot()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
