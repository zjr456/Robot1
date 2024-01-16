import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import os
from sklearn.svm import LinearSVC
from scipy.cluster.vq import *
from sklearn.preprocessing import StandardScaler
from sklearn import preprocessing

# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode= Realsense()

    signal.signal(signal.SIGINT, rsNode.signalInteruption)

    while rsNode.isOk:
        rsNode.read_imgs()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.001)

    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()





# Realsense Node:
class Realsense(Node):
    def __init__(self, fps= 60):
        super().__init__('realsense')
        self.image_publisher = self.create_publisher(Image,'image',10)
        self.detection_publisher = self.create_publisher(String,'detection',10)
        self.bridge=CvBridge()
        self.pipeline = rs.pipeline()
        config = rs.config()
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        found_rgb = True
        for s in device.sensors:
            print( "Name:" + s.get_info(rs.camera_info.name) )
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
        if not (found_rgb):
            print("Depth camera equired !!!")
            exit(0)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

        self.align_to = rs.stream.depth
        self.align = rs.align(self.align_to)

# Start streaming
        self.pipeline.start(config)
        self.isOk = True
        self.color_info = (0, 0, 255)
       
    
    
    def read_imgs(self):
        sys.stdout.write("-")
        count= 1
        refTime= time.process_time()
        freq= 60
        self.test = False

        
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()


        #Aligning color frame to depth frame
        aligned_frames =  self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not (depth_frame and color_frame):
            return

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        print(depth_image)
        self.color_image = np.asanyarray(color_frame.get_data())


        hsv = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(hsv)
         
        #print(h)
        #print(s)
        #print(v)
        hmin = 45
        hmax = 75
        mask = np.uint8((hmin < h)&(h < hmax) & (s> 100) & (v> 50))

        if(np.sum(mask)>1000):
            self.test = True
        #print (np.sum(tmp))
        mask = mask*255
        #self.color_image=cv2.merge((tmp,tmp,tmp))




        kernel = np.ones((3,3),np.uint8)
        mask = cv2.erode(mask, kernel, iterations = 2)
        mask = cv2.dilate(mask,kernel,iterations = 2)
        mask = cv2.blur(mask, (7, 7))

        print(mask.shape)

        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>30:
                cv2.circle(self.color_image, (int(x), int(y)), int(rayon), self.color_info, 2)
                cv2.circle(self.color_image, (int(x), int(y)), 5, self.color_info, 10)
                cv2.line(self.color_image, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(self.color_image, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth = depth_frame.get_distance(x, y)
            print(depth)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = self.color_image.shape

        sys.stdout.write( f"\r- {color_colormap_dim} - {depth_colormap_dim} - ({round(freq)} fps)" )

        # Show images
        images = np.hstack((self.color_image, depth_colormap))

        # Show images
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

        # Frequency:
        if count == 10 :
            newTime= time.process_time()
            freq= 10/((newTime-refTime))
            refTime= newTime
            count= 0
        count+= 1

    # Capture ctrl-c event
    def signalInteruption(signum, frame):
        print( "\nCtrl-c pressed" )
        self.isOk= False

    def publish_imgs(self):
        msg_image = self.bridge.cv2_to_imgmsg(self.color_image,"bgr8")
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image"
        self.image_publisher.publish(msg_image)
        myStr = String()
        if self.test == True:
            myStr.data = "bottle founded"
        else:
            myStr.data = "bottle unfounded"
        self.detection_publisher.publish(myStr)
        



process_img()

