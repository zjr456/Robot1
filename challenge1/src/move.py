#!/usr/bin/python3
from visualization_msgs.msg import Marker
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
import math
#message to publish:
from geometry_msgs.msg import Twist


class Move(Node):
    def __init__(self, fps= 60):
        super().__init__('move')
        self.marker_publisher = self.create_publisher( Marker,'marker_test',10)
        self.create_subscription( Float32, 'bottlepostion',self.bottlepostion_callback, 10)
        self.create_subscription( String, 'detection',self.detection_callback, 10)
        self.create_subscription( Float32, 'distancebottle',self.distancebottle_callback, 10)
        self.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cloud_publisher = self.create_publisher(pc2.PointCloud2,'laser_link',10)
        self.isOk = True
        self.detection = False
        self.distancebottle = -1.0
        self.x_label = -1.0

    def bottlepostion_callback(self,msg):
        self.x_label = msg.data
        #print('good')


    def marker_callback(self,marker):
        print('obstacle!!!')
        pass
        

    def scan_callback(self, scanMsg):
        angle = scanMsg.angle_min
        obstacles = []
        cmd_debug_points_left = []
        cmd_debug_points_right = []

        if self.detection == True:
            print('hello')
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.type = Marker.SPHERE

            self.x_label = int(self.x_label)
            print('x_label')
            print(self.x_label)

            if(0<=self.x_label )and(self.x_label<424):
                ratio = ((424-int(self.x_label))/424*43)
                x_real_label = self.distancebottle*math.sin(ratio)
                y_real_label = self.distancebottle*math.cos(ratio)
            
            elif (424>=self.x_label )and(self.x_label<=848):
                ratio = ((848-int(self.x_label))/424*43)
                x_real_label = self.distancebottle*math.sin(ratio)
                y_real_label = self.distancebottle*math.cos(ratio)
            else :
                x_real_label = None
                y_real_label = None

            print('x\n')
            print(x_real_label)
            print('y\n')
            print(y_real_label)
            
            if (x_real_label and y_real_label):
                marker.pose.position.x = x_real_label
                marker.pose.position.y = y_real_label
                marker.pose.position.z = 0.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                self.marker_publisher.publish(marker)

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

        #elif self.detection == True:
        else:
            velo.linear.x = 0.15
            velo.angular.z = 0.0 

        #print(velo.linear.x)
        #print(velo.angular.z)
        #print(len(cmd_debug_points_right) + len(cmd_debug_points_left))
        #print(len(cmd_debug_points_right) - len(cmd_debug_points_left))
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
        #print(distancebottleMsg.data)

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