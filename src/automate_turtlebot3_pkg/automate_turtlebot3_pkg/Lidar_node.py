#!/usr/bin/env python

import rclpy
import numpy as np
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan


class AutomateTurtlebot(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(AutomateTurtlebot, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)
        
        self.create_subscription(LaserScan, "/scan", self.subscribe_callback, qos)

        self.control_publisher = self.create_publisher(Float32MultiArray, "/min_dist", qos)

        # self.control_publisher = [
        #     self.create_publisher(Range, "/min_dis_front", qos),
        #     self.create_publisher(Range, "/min_dis_left", qos),
        #     self.create_publisher(Range, "/min_dis_right", qos)]


    def subscribe_callback(self, scan: LaserScan):
        """
        this function is executed every subscription.
        """
        
        min_distance_front, min_distance_left, min_distance_right = self.calc_distance_to_wall(scan)
        
        print("\n \n \n --------------------------------------- \n \n" 
        "\n min distance_front:  ", min_distance_front,"\n \n"
        "min_distance_left:  ", min_distance_left,"\n \n" 
        "min_distance_right:  ", min_distance_right,
        "\n \n \n --------------------------------------- \n \n \n \n \n")

        tmp = [min_distance_front, min_distance_left, min_distance_right]
        pub_msg = Float32MultiArray(data = tmp)
        self.control_publisher.publish(pub_msg)

        # pub_msg_left= Range()
        # pub_msg_left.range = min_distance_left        

        # pub_msg_right= Range()
        # pub_msg_right.range = min_distance_right

        # Here I am publishing min_distance_front
        # self.control_publisher[0].publish(pub_msg_front)
        # self.control_publisher[1].publish(pub_msg_left)
        # self.control_publisher[2].publish(pub_msg_right)


    def calc_distance_to_wall(self, scan: LaserScan) -> float:

        min_distance_front = scan.ranges[0]

        #90° × π/180 = 1,571rad
        left_index = int((np.pi/180 *90)/scan.angle_increment)
        right_index = int((np.pi/180 *270)/scan.angle_increment)
    
        min_distance_left = scan.ranges[left_index]
        min_distance_right = scan.ranges[right_index]
        

       
        return [min_distance_front, min_distance_left, min_distance_right]

    def create_turnleft_msg(self) -> Twist:
        # TODO: implement here -> tried some values, I will change this when I see how it moves
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
    node = AutomateTurtlebot()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
