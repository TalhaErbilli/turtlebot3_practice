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

    def subscribe_callback(self, scan: LaserScan):
        """
        this function is executed every subscription.
        """
        
        min_distance = self.calc_distance_to_wall(scan)
        print(min_distance)
        
        if min_distance < 0.7:
            print("turn left")
            pub_msg = self.create_turnleft_msg()
        else:
            print("drive_fwd")
            pub_msg = self.create_forward_msg()
        self.control_publisher.publish(pub_msg)



    def calc_distance_to_wall(self, scan: LaserScan) -> float:
        # TODO implement get min distance from LaserScan -> Done
        # NOTE: use range_min -> I did this similarly to the Twist
        
        #scan.range_min = 0.0
        #scan.range_max = 3.5
        #list_distance = scan.ranges
        #print(len(list_distance))
        #print(max(list_distance))  # you can see the value of scan.range_max (3.5)
        #print(min(list_distance))  # you can see the value of scan.range_min (0.0)
        #print("angle min", scan.angle_min)
        #print("angle max", scan.angle_max)
        
        min_distance = scan.ranges[0]
       
        return min_distance

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
        # TODO: implement here -> done
        fwd_msg = Twist()
        fwd_msg.linear.x = 0.3
        fwd_msg.linear.y = 0.
        fwd_msg.linear.z = 0.
        fwd_msg.angular.x = 0.
        fwd_msg.angular.y = 0.
        fwd_msg.angular.z = 0.
        return fwd_msg

    def create_stop_msg(self) -> Twist:
        # TODO: implement here -> Done
    
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
