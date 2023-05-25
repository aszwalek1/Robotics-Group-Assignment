#! /usr/bin/env python3

import rospy
import sys
from pathlib import Path 
# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

# Initialisations: 
node_name = "task5_node"
rospy.init_node(node_name)
print(f"Launched the '{node_name}' node")
rate = rospy.Rate(5)

base_image_path = Path.home().joinpath("/home/student/catkin_ws/src/com2009_team6/snaps/")
base_image_path.mkdir(parents = True, exist_ok = True) 

cvbridge_interface = CvBridge() 
waiting_for_image = True

class task5(object):

    def save_image(self, img, img_name):
        full_image_path = base_image_path.joinpath(f"{img_name}.jpg") 
        
        # Saving image to a .jpg file
        cv2.imwrite(str(full_image_path), img)
        
    def take_picture(self, img_data):
        global waiting_for_image
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8") 
        except CvBridgeError as e:
            print(e)

        # crop the image
        height, width, _ = cv_img.shape

        crop_img = cv_img[int(height*0.25):int(height*0.75), int(width*0.25):int(width*0.75)]

        self.isBeacon = self.color_detection(crop_img)

        
        if waiting_for_image == True and self.isBeacon: 
            self.save_image(crop_img, img_name = "the_beacon")
            waiting_for_image = False

    def color_detection(self, img_data):
        self.hsv_img = cv2.cvtColor(img_data, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv_img, np.array(self.colours[self.target_colour][0]), np.array(self.colours[self.target_colour][1]))
        
        return round(np.count_nonzero(self.mask) / (len(self.mask) * len(self.mask[1])), 2) > 0.05
    
    def __init__(self):
        self.cvbridge_interface = CvBridge()
        rospy.on_shutdown(self.shutdown_ops)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.take_picture)
        self.colours = {
        "red":    ([169, 150, 75], [180, 255, 255]),
        "green":   ([75, 150, 75], [85, 255, 255]),
        "yellow": ([20, 140, 75], [30, 230, 255]),
        "blue":   ([95, 200, 75], [105, 255, 255]),     
        }
        self.target_colour = rospy.myargv(argv=sys.argv)[-1]
        print(f"TASK 5 BEACON: The target is {self.target_colour}.")

        while waiting_for_image: 
            rate.sleep()

        self.shutdown_ops()

    def shutdown_ops(self):
        cv2.destroyAllWindows()

if __name__ == '__main__':
    tk5 = task5()