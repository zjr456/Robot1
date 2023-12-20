# -*- coding: utf-8 -*-
"""
Created on Wed Dec 20 09:33:54 2023

ROBOT
@author: BIENVENUE
"""

#!python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

rosNode= None
obstacles= []
information = []

class Robot():
    

    def __init__(self,name):
        self.name = name
    

    def scan_callback(scanMsg):
        global rosNode,obstacles
        rosNode.get_logger().info( f"scan:\n{scanMsg}" )
        angle= scanMsg.angle_min
        for aDistance in scanMsg.ranges :
            if 0.1 < aDistance and aDistance < 5.0 :
                aPoint= [
                    math.cos(angle) * aDistance,
                    math.sin(angle) * aDistance
                ]
                obstacles.append(aPoint)
            angle+= scanMsg.angle_increment
            
        aPoint= Point32()
        aPoint.x= (float)(math.cos(angle) * aDistance)
        aPoint.y= (float)(math.sin( angle ) * aDistance)
        aPoint.z= (float)(0)
        triple_data = (aPoint.x, aPoint.y, aPoint.z)
        information.append(triple_data)
        
        
    
    def point_cloud(self):
        sample= [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[10:20] ]
        self.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )

    def main(self):
        rclpy.init()
        rosNode= Node('scan_interpreter')
        rosNode.create_subscription( LaserScan, 'scan', self.scan_callback, 10)
        while True :
            rclpy.spin_once( rosNode )
        scanInterpret.destroy_node()
        rclpy.shutdown() 
        
        
if __name__ == '__main__' :
    robot1 = Robot("ROBOT1")
    robot1.main()
    
    









    