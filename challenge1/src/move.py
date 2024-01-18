#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from std_msgs.msg import Float32
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time
import math
import random
import signal
#message to publish:
from geometry_msgs.msg import Twist


class Move(Node):
    def __init__(self, fps= 60):
        super().__init__('move')
        self.create_subscription( String, 'detection',self.detection_callback, 10)
        self.create_subscription( Float32, 'distancebottle',self.distancebottle_callback, 10)
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2,'laser_link',10)
        self.isOk = True

    def scan_callback(self, scanMsg):
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
            velo.angular.z = 1.5
            velo.linear.x = 0.05
        
            
        elif len(cmd_debug_points_left) > 0:
            print("avoid right")
            velo.angular.z = -1.5
            velo.linear.x = 0.05

        else:
            velo.linear.x = 0.15
            velo.angular.z = 0.0 

        print(velo.linear.x)
        print(velo.angular.z)
        print(len(cmd_debug_points_right) + len(cmd_debug_points_left))
        print(len(cmd_debug_points_right) - len(cmd_debug_points_left))
        cloudPoints = pc2.create_cloud_xyz32(Header(frame_id='laser_link'),obstacles)
        self.publish_move(velo, cloudPoints)
        
    def detection_callback(self,detectionMsg):
        if detectionMsg.data == "bottle unfounded":
            print("bottle unfounded")
            self.detection = False
        elif detectionMsg.data == "bottle founded":
            print("bottle founded")
            self.detection = True

    def distancebottle_callback(self,distancebottleMsg):
        self.distancebottle = distancebottleMsg.data
        print(distancebottleMsg.data)

    def publish_move(self, velo, cloudPoints):
        self.velocity_publisher.publish(velo)
        self.cloud_publisher.publish(cloudPoints)



    def signalInteruption(signum, frame):
        print( "\nCtrl-c pressed" )
        self.isOk= False

    



if __name__ == '__main__':
    print("move move move")
    rclpy.init()
    rosNode = Move()
    signal.signal(signal.SIGINT, rosNode.signalInteruption)
    while rosNode.isOk:
        rclpy.spin_once(rosNode, timeout_sec=0.1)
    rosNode.destroy_node()
    print("Ending...")
    rclpy.shutdown() 