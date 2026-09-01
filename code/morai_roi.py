import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:

    def __init__(self):
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback) #이미지 받음
        self.img_bgr = None


    def callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)          #이미지 처리
            self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print(e)
            
    def mouse_callback(self,event, x,y, flag, param):
        if event == cv2.EVENT_LBUTTONUP:
            print(x,y)

def start(): 
    rospy.init_node("find_roi",anonymous=True)

    image_parser = IMGParser()
    
    cv2.imshow("roi",image_parser.img_bgr)
    cv2.setMouseCallback("roi",image_parser.mouse_callback)   
   
    rospy.spin()
    
if __name__ =="__main__":
    start()