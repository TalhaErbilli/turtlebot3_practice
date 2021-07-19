#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan, Range


class Look_to_front(Node):
    def __init__(self, node_name: str = 'automate_turtlebot', **kwargs: dict):
        super(Look_to_front, self).__init__(node_name=node_name, **kwargs)
        qos = QoSProfile(depth=1)

        self.create_subscription(LaserScan, "/scan", self.subscribe_callback, qos)

        # Creating a separate topic, can't publish as LaserScan, it's a single value so Publish as Range
        self.control_publisher = self.create_publisher(Range, "/min_distance_front", qos)        

    def subscribe_callback(self, scan: LaserScan):
        """
        this function is executed every subscription.
        """     
        
        pub_msg = Range()
        pub_msg.range = self.calc_distance_to_wall(scan)
        print("min_distance front:  ", pub_msg.range)

        # Here I am publishing min_distance_front
        self.control_publisher.publish(pub_msg)



    def calc_distance_to_wall(self, scan: LaserScan) -> float:
        #scan.range_min = 0.0
        #scan.range_max = 3.5
        #list_distance = scan.ranges
        #print(len(list_distance))
        #print(max(list_distance))  # you can see the value of scan.range_max (3.5)
        #print(min(list_distance))  # you can see the value of scan.range_min (0.0)
        #print("angle min", scan.angle_min)
        #print("angle max", scan.angle_max)
        
        # This looks at the LIDAR angle right in front of the robot
        min_distance_front = scan.ranges[0]
       
        return min_distance_front

    
def main():
    rclpy.init()
    node = Look_to_front()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
    