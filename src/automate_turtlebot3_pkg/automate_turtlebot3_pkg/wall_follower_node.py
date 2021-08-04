#!/usr/bin/env python
import math
from typing import Pattern
import numpy
import sys
import termios
from numpy import float32, float_, left_shift, right_shift
from numpy.lib.function_base import angle
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.timer import Timer
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from automate_turtlebot3_pkg.tf_utils import TFUtils


class wall_follow(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(wall_follow, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)
        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.step = 1
        self.get_key_state = False
        self.init_odom_state = False  # To get the initial pose at the beginning
        
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        # Declaring cmd_vel to publish Twist
        self.control_publisher = self.create_publisher(Twist, "/cmd_vel", qos)
        # subscribing to my own topic: min_distance
        self.create_subscription(
            Float32MultiArray, "/min_dis", self.subscribe_callback, qos
        )
        # subscribing to /tf
        # self.create_subscription(TFMessage, "/tf", self.subscribe_callback, qos)
        self.tf_utils = TFUtils(self, False)
        self.turtlbot_state = None

        self.create_timer(0.1, self.tf_timer_callback)

        self.create_timer(1, self.control_timer_callback)
        self.start_time = None


    def subscribe_callback(self, min_dis: Float32MultiArray):
        """
        this function is executed every subscription.
        """
        float_array = min_dis.data
        # print("\n \n \n --------------------------------------- \n \n"
        # "\n min distance_front:  ", float_array[0],"\n \n"
        # " min_distance_left:  ", float_array[1],"\n \n"
        # " min_distance_right:  ", float_array[2],
        # "\n \n \n --------------------------------------- \n \n \n \n \n")

        self.control_publisher.publish(self.create_forward_msg())
        print("drive_fwd, \n \nfront sensor:   ", float_array[0])

        if float_array[0]  < 0.5:
            #after stopping robot, let's turn a perfect 90 degrees.
            self.control_publisher.publish(self.create_turnright_msg())
            print("turning robot _R_:  ")
            #during the turn, we need to check the base_link transform
            #and compare this to the transformation we need.
            old_yaw = self.tf_timer_callback()    
            new_yaw = self.calculate_new_yaw()
            
            if old_yaw == new_yaw:
                self.control_publisher.publish(self.create_forward_msg())
                print("fwd 2")  


        if float_array[1]  < 0.5:
            #after stopping robot, let's turn a perfect 90 degrees.
            self.control_publisher.publish(self.create_turnright_msg())
            print("turning robot _R_:  ")
            #during the turn, we need to check the base_link transform
            #and compare this to the transformation we need.
            old_yaw = self.tf_timer_callback()    
            new_yaw = self.calculate_new_yaw()

            if old_yaw == new_yaw:
                self.control_publisher.publish(self.create_forward_msg())
                print("fwd 2")  

        



        
        
        # #the code below is to practice simple commands.
        # float_array = min_dis.data
        # #drive forward until front sensor(float_array[0]< x)
        # self.control_publisher.publish(self.create_forward_msg())
        # print("drive_fwd")
        # if float_array[0] < 0.5:
        #     print("turn_left", float_array[1])
        #     self.control_publisher.publish(self.create_turnleft_msg())

        # #Now our robot is facing left, so the right sensor should face the wall, that's why we're looking at float_array[2].
        #     if float_array[2] < 0.3:
        #         print("turn_right:  ", float_array[2])
        #         self.control_publisher.publish(self.create_turnright_msg())
        # #If we are too close to the wall, we should turn back right a little bit, but not too far. 
        #     elif float_array[2] > 0.6:
        #         print("turn_left_2:  ", float_array[2])
        #         self.control_publisher.publish(self.create_turnleft_msg())
        # #in any other case we must be within acceptable ranges, so we can drive straight.
        #     else:
        #         print("drive_fwd_2:  ", float_array[0])
        #         self.control_publisher.publish(self.create_forward_msg())


    def tf_timer_callback(self, ):
        transform = self.tf_utils.lookup_transform(
            target_frame='base_link',
            source_frame='odom',
            convert=False,
            when=None)
        #print(transform)
        # TODO: calculate the yaw of turtlebot : DONE
        self.turtlbot_state = transform.transform.rotation
        #From the Euler_from quaternion function:
        Euler = self.euler_from_quaternion(self.turtlbot_state)
        yaw = Euler[2]
        # print("yaw:  ", yaw)
        return yaw


    def control_timer_callback(self, ):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds
            #print(self.start_time)

        #TODO: something is wrong with this line below, try to fix it (added .seconds_nanoseconds)
        if (self.get_clock().now().nanoseconds - self.start_time) < 3 *10**9:
            # forward
            control_msg = self.create_forward_msg()
        else:
            # stop
            control_msg = self.create_stop_msg()
        #self.control_publisher.publish(control_msg)

    def calculate_new_yaw(self):
        old_yaw = self.tf_timer_callback()
        #print("old yaw:  ", old_yaw)
        angle_of_turn = 90
        #we are adding the turn to the yaw
        new_yaw = old_yaw + angle_of_turn
        #print("new yaw:  ", new_yaw)

        return new_yaw

        # self.control_publisher.publish(self.create_turnright_msg())
        # print("turn_right")
        # if old_yaw == new_yaw:
        #     self.control_publisher.publish(self.create_stop_msg())
        #     print("turned ", angle_of_turn, "degrees")


    def create_turnright_msg(self) -> Twist:
        turn_right = Twist()
        turn_right.linear.x = 0.0
        turn_right.linear.y = 0.0
        turn_right.linear.z = 0.0
        turn_right.angular.x = 0.0
        turn_right.angular.y = 0.0
        turn_right.angular.z = -0.2
        return turn_right

    def create_turnleft_msg(self) -> Twist:
        turn_left = Twist()
        turn_left.linear.x = 0.0
        turn_left.linear.y = 0.0
        turn_left.linear.z = 0.0
        turn_left.angular.x = 0.0
        turn_left.angular.y = 0.0
        turn_left.angular.z = 0.2
        return turn_left

    def create_forward_msg(self) -> Twist:
        fwd_msg = Twist()
        fwd_msg.linear.x = 0.2
        fwd_msg.linear.y = 0.0
        fwd_msg.linear.z = 0.0
        fwd_msg.angular.x = 0.0
        fwd_msg.angular.y = 0.0
        fwd_msg.angular.z = 0.0
        return fwd_msg

    def create_stop_msg(self) -> Twist:
        stop_drive = Twist()
        stop_drive.linear.x = 0.0
        stop_drive.linear.y = 0.0
        stop_drive.linear.z = 0.0
        stop_drive.angular.x = 0.0
        stop_drive.angular.y = 0.0
        stop_drive.angular.z = 0.0
        return stop_drive

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""

    def euler_from_quaternion(self, quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw.
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


def main():
    rclpy.init()
    node = wall_follow()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
