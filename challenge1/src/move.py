#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time
import math
import random
#message to publish:
from geometry_msgs.msg import Twist

rosNode = None

def scan_callback(scanMsg, value):
    global rosNode
    angle = scanMsg.angle_min
    obstacles = []
    cmd_debug_points_left = []
    cmd_debug_points_right = []
    
    
    
      

    for aDistance in scanMsg.ranges:
        if 0.1 < aDistance < 5.0:
            aPoint = [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance,
                0.0
            ]
            obstacles.append(aPoint)
            if (0.01 < aPoint[1] < 0.2 and 0.1 < aPoint[0] < 0.5) :
                cmd_debug_points_left.append(aPoint)
            if (-0.2 < aPoint[1] < -0.01 and 0.1 < aPoint[0] < 0.5) :
                cmd_debug_points_right.append(aPoint)
        angle += scanMsg.angle_increment

    velo = Twist()
         
    if len(cmd_debug_points_right) > 0:
        print("avoid Left")
        velo.angular.z = 1.0
        velo.linear.x = 0.05
       
        
    elif len(cmd_debug_points_left) > 0:
        print("avoid right")
        velo.angular.z = -1.0
        velo.linear.x = 0.05

    else:
        velo.linear.x = 0.15
        velo.angular.z = 0.0 
       


    
    print(velo.linear.x)
    print(velo.angular.z)
    print(len(cmd_debug_points_right) + len(cmd_debug_points_left))
    print(len(cmd_debug_points_right) - len(cmd_debug_points_left))
      

    

    velocity_publisher.publish(velo)
    cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'),obstacles)
    cloud_publisher.publish(cloudPoints)


if __name__ == '__main__':
    print("move move move")
    rclpy.init()
    rosNode = Node('PC_Publisher')
    velocity_publisher = rosNode.create_publisher(Twist, '/multi/cmd_nav', 10)
    cloud_publisher = rosNode.create_publisher(pc2.PointCloud2,'laser_link',10)
    rosNode.create_subscription( LaserScan, 'scan', scan_callback, 10)
    while True :
        rclpy.spin_once(rosNode , timeout_sec=0.1 )
    scanInterpret.destroy_node()
    rclpy.shutdown() 