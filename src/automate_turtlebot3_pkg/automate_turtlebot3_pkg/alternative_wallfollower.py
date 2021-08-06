from typing import ForwardRef
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from automate_turtlebot3_pkg.tf_utils import TFUtils

class WallFollower(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(WallFollower, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)

        # sensor variable
        self.distance_array = np.zeros(3)

        # state variable
        self.current_yaw = 0.

        # logic variable
        self.target_yaw = 0.
        self.current_state = 'start'
        self.turn_left_deg = 87.

        # Declaring cmd_vel to publish Twist
        self.control_publisher = self.create_publisher(Twist, "/cmd_vel", qos)

        # initialize subscription
        self.create_subscription(Float32MultiArray, '/min_dis', self.sensor_subscribe_callback, qos)

        # initialize timer function for control
        self._control_period = 0.1
        self.create_timer(self._control_period, self.control_timer_callback)

        # initialize timer function for state acquisition
        self._state_period = 0.1
        self.tf_utils = TFUtils(self, False)
        self.create_timer(self._state_period, self.state_timer_callback)

    def sensor_subscribe_callback(self, msg: Float32MultiArray):
        """read sensor data and update self.distance_array"""
        self.distance_array = msg.data

    def state_timer_callback(self, ):
        """ read tf and update current yaw"""
        transform = self.tf_utils.lookup_transform(
            target_frame='base_link',
            source_frame='odom',
            convert=False,
            when=None)
        self.current_yaw = self.euler_from_quaternion(
            transform.transform.rotation)[2]

    
    def create_turnright_msg(self) -> Twist:
        turn_right = Twist()
        turn_right.linear.x = 0.0
        turn_right.linear.y = 0.0
        turn_right.linear.z = 0.0
        turn_right.angular.x = 0.0
        turn_right.angular.y = 0.0
        turn_right.angular.z = -0.1
        return turn_right

    def create_turnleft_msg(self) -> Twist:
        turn_left = Twist()
        turn_left.linear.x = 0.0
        turn_left.linear.y = 0.0
        turn_left.linear.z = 0.0
        turn_left.angular.x = 0.0
        turn_left.angular.y = 0.0
        turn_left.angular.z = 0.1
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
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw


    def control_timer_callback(self,):
        """logic"""
        print(self.current_state,"1")

        # 1. start forward, init
        if self.current_state == 'start':
            print(self.current_state)
            self.current_yaw = 0.

            self.current_state = 'forward'
            print(self.current_state, "1")

        # 2. Conditions for driving forward    
        elif self.current_state == 'forward':
            if self.distance_array[0] > 0.5:
                self.control_publisher.publish(self.create_forward_msg())
                print( self.current_state,"2","\nFront sensor:  ", self.distance_array[0],"[m], \n\nCurrent_yaw:  ", np.rad2deg(self.current_yaw), "\n", "Target yaw:  ", np.rad2deg(self.target_yaw), "\n\n")
            else:
                self.current_state = 'turn_right'

        # 3. condition for turning right
        elif self.current_state == 'turn_right':
            self.target_yaw = self.current_yaw + np.deg2rad(self.turn_deg)
            if self.target_yaw > np.deg2rad(180):
                self.target_yaw = self.target_yaw - np.deg2rad(360)
            self.control_publisher.publish(self.create_turnright_msg())
            print( self.current_state,"3","\nFront sensor:  ", self.distance_array[0],"[m], \n\nCurrent_yaw:  ", np.rad2deg(self.current_yaw), "\n", "Target yaw:  ", np.rad2deg(self.target_yaw), "\n\n")
        
        # 4. Conditions for turning left
        elif self.target_yaw < self.current_yaw:
            self.current_state = 'turn_left'        
            if self.target_yaw <np.deg2rad(-180):
                self.target_yaw = self.target_yaw + np.deg2rad(360)
            self.control_publisher.publish(self.create_turnleft_msg())
            print( self.current_state,"4","\nFront sensor:  ", self.distance_array[0],"[m], \n\nCurrent_yaw:  ", np.rad2deg(self.current_yaw), "\n", "Target yaw:  ", np.rad2deg(self.target_yaw), "\n\n")
                    
        # 5. conditions to drive back straight.
        elif self.current_yaw == self.target_yaw:
            self.current_state = 'forward'
            print( self.current_state,"5")





def main():
    rclpy.init()
    node = WallFollower()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
