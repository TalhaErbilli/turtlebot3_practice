#!/usr/bin/env python

from numpy import float32
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range


class wall_follow(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(wall_follow, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)

        # We still need the same here
        self.control_publisher = self.create_publisher(Twist, "/cmd_vel", qos)

        # subscribing to my own topic: min_distance
        self.create_subscription(Float32MultiArray, "/min_dist", self.subscribe_callback, qos)

        # self.create_subscription[
        #     (Range, "/min_dis_front", self.subscribe_callback, qos),
        #     (Range, "/min_dis_left", self.subscribe_callback, qos),
        #     (Range, "/min_dis_right", self.subscribe_callback, qos)]
       

    def subscribe_callback(self, min_dis: Float32MultiArray):
        """
        this function is executed every subscription.
        """
        # Just looked at the formatting in the laser_node min_distance.range what I need.
        min_dis_front = min_dis[0]
        min_dis_left = min_dis[1]
        min_dis_right = min_dis[2]

        print("front:  ", min_dis_front, "left:  ", min_dis_left, "right:  ", min_dis_right)
        # if min_dis_left > min_dis_right:
        #     print("turn left")
        #     pub_msg = self.create_turnleft_msg()
        # else:
        #     print("drive_fwd")
        #     pub_msg = self.create_forward_msg()
        #self.control_publisher.publish(pub_msg)

    def create_turnright_msg(self) -> Twist:
        turn_right = Twist()
        turn_right.linear.x = 0.0
        turn_right.linear.y = 0.
        turn_right.linear.z = 0.
        turn_right.angular.x = 0.
        turn_right.angular.y = 0.
        turn_right.angular.z = 0.4
        return turn_right


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
    node = wall_follow()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
