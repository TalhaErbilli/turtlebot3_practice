#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range




class Locomotion(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(Locomotion, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)

        # We still need the same here
        self.control_publisher = self.create_publisher(Twist, "/cmd_vel", qos)

        # subscribing to my own topic: min_distance_front, this is not working yet because I think Range is publishing 3 values now..
        self.create_subscription(Range, "/min_distance_front", self.subscribe_callback, qos)

    def subscribe_callback(self, min_distance_front: Range):
        """
        this function is executed every subscription.
        """   
           

        min_distance_front = min_distance_front.range

        #print("min distance front:  ", min_distance_front)

        if min_distance_front < 0.7:
            print("turn left")
            pub_msg = self.create_turnleft_msg()
        else:
            print("drive_fwd")
            pub_msg = self.create_forward_msg()
        self.control_publisher.publish(pub_msg)


    def create_turnleft_msg(self) -> Twist:
        turn_left = Twist()
        turn_left.linear.x = 0.0
        turn_left.linear.y = 0.
        turn_left.linear.z = 0.
        turn_left.angular.x = 0.
        turn_left.angular.y = 0.
        turn_left.angular.z = 0.4
        return turn_left

    def create_forward_msg(self) -> Twist:
        fwd_msg = Twist()
        fwd_msg.linear.x = 0.3
        fwd_msg.linear.y = 0.
        fwd_msg.linear.z = 0.
        fwd_msg.angular.x = 0.
        fwd_msg.angular.y = 0.
        fwd_msg.angular.z = 0.
        return fwd_msg

    def create_stop_msg(self) -> Twist:
        stop_drive = Twist()
        stop_drive.linear.x = 0.0
        stop_drive.linear.y = 0.0
        stop_drive.linear.z = 0.
        stop_drive.angular.x = 0.
        stop_drive.angular.y = 0.
        stop_drive.angular.z = 0.
        return stop_drive


def main():
    rclpy.init()
    node = Locomotion()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
