#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time
import math

#message to publish:
from geometry_msgs.msg import Twist

rosNode = None

def scan_callback(scanMsg):
    global rosNode
    angle = scanMsg.angle_min + math.pi/2
    obstacles_left = False
    obstacles_right = False
    obstacles = []
    cmd_debug_points = []

    for aDistance in scanMsg.ranges:
        if 0.1 < aDistance < 5.0:
            aPoint = [
                math.cos(angle) * aDistance,
                math.sin(angle) * aDistance,
                0
            ]
            obstacles.append(aPoint)
            if (0.01 < aPoint[0] < 0.5 and 0.15 < aPoint[1] < 1):
                obstacles_right = True
                cmd_debug_points.append(aPoint)
            if (-0.5 < aPoint[0] < -0.01 and 0.15 < aPoint[1] < 1):
                obstacles_left = True
                cmd_debug_points.append(aPoint)
        angle += scanMsg.angle_increment

    velo = Twist()
    if obstacles_left:
        print("Left")
        # print(len(cmd_debug_points), cmd_debug_points)
        velo.angular.z = -0.5
        velo.linear.x = 0.0
        time.sleep(0.2)
        
    elif obstacles_right:
        print("Right")
        # print(len(cmd_debug_points), cmd_debug_points)
        velo.angular.z = 0.5
        velo.linear.x = 0.0
        time.sleep(0.2)
    else:
        velo.linear.x = 0.2
        velo.angular.z = 0.0

    velocity_publisher.publish(velo)
    cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'),cmd_debug_points)
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