#!/usr/bin/python3
from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
import time
import math
import random
import signal
import math
#message to publish:
from geometry_msgs.msg import Twist
import math
import signal, time, numpy as np
import sys, cv2, rclpy
from rclpy.node import Node
#from nav_msgs.msg import Path
#from geometry_msgs.msg import PointStamped
#from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
#from robot_navigator import BasicNavigator, NavigationResult
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_msgs.msg import Float32
import cv2
import numpy as np
import os
from scipy.cluster.vq import *
import time

class Move(Node):
    def __init__(self, fps= 60):
        super().__init__('move')
        #navigator = BasicNavigator()
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.initial_pose = PoseWithCovarianceStamped()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.initial_pose.header.frame_id = 'map'
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.orientation.w = 0.0
        self.goal_pose.pose.orientation.z = 1.0
        self.initial_pose.pose.pose.position.x = -2.27
        self.initial_pose.pose.pose.position.y = 1.23
        self.initial_pose.pose.pose.orientation.w = 0.114
        self.initial_pose.pose.pose.orientation.z = 1.0
        rclpy.spin_once(self, timeout_sec=0.1)
        #navigator.setInitialPose(initial_pose)
        self.pose_publisher.publish(self.initial_pose)
        self.pose_publisher.publish(self.goal_pose)
        print("initial good")
        self.create_subscription( Twist, 'cmd_vel',self.cmdvel_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/multi/cmd_nav', 10)
        self.isOk = True
        self.detection = False



    def scan_callback(self, scanMsg):
        self.pose_publisher.publish(self.initial_pose)
        self.pose_publisher.publish(self.goal_pose)


    def cmdvel_callback(self,msg):
        #print("good")
        velo = Twist()
        velo.linear.x = msg.linear.x
        velo.angular.z = msg.angular.z
        self.velocity_publisher.publish(velo)


    
    def signalInteruption(self,signum, frame):
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