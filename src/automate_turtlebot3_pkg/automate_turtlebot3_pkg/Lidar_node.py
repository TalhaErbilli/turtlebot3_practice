#!/usr/bin/env python

import rclpy
import numpy as np
from sensor_msgs import msg
from sensor_msgs.msg import PointCloud2 as pc2
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan
from std_msgs.msg import Float32MultiArray


class AutomateTurtlebot(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(AutomateTurtlebot, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)
        
        #Subscribing to /scan so calc_dist_to_wall has the input data to work with
        self.create_subscription(LaserScan, "/scan", self.subscribe_callback, qos)

        #Trying out a good format to publish in, so the wall_follower gets the data.
        self.control_publisher = self.create_publisher(Float32MultiArray, "/min_dis", qos)

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
        " min_distance_left:  ", min_distance_left,"\n \n" 
        " min_distance_right:  ", min_distance_right,
        "\n \n \n --------------------------------------- \n \n \n \n \n")

        float_array = Float32MultiArray(data = [min_distance_front, min_distance_left, min_distance_right])
        self.control_publisher.publish(float_array)


       # cloud_out = self.calc_distance_to_wall(scan)
       # self.control_publisher.publish(cloud_out)


        # Here I am publishing min_dist
        # self.control_publisher[0].publish(pub_msg_front)
        # self.control_publisher[1].publish(pub_msg_left)
        # self.control_publisher[2].publish(pub_msg_right)


    def calc_distance_to_wall(self, scan: LaserScan) -> float:

        min_distance_front = scan.ranges[0]

        # degree° × π/180 = 1,571rad
        left_index = int((np.pi/180 *15)/scan.angle_increment)
        right_index = int((np.pi/180 *345)/scan.angle_increment)
    
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
