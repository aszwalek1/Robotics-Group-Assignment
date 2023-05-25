#! /usr/bin/python3
# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import all the necessary ROS message types:
from sensor_msgs.msg import LaserScan, Image

# Import some other modules from within this package
from tb3 import Tb3LaserScan, Tb3Odometry, Tb3Move
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Import other modules
import numpy as np
import math

class maze_nav():

    def __init__(self):
        rospy.init_node('maze_navigation', anonymous=True)


        self.move = Tb3Move()
        self.odom = Tb3Odometry()
        self.scan = Tb3LaserScan()

        # Robot control vars
        self.move.set_move_cmd(0.0, 0.0)

        self.counter = 0
        self.yaw = 0
 
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_hook)

        self.rate = rospy.Rate(10)

    def shutdown_hook(self):
        self.move.set_and_pub(0,0)
        self.ctrl_c = True
        
   
    def launch(self):
        l, a = 0, 0
        
        while not self.ctrl_c: 
            if self.counter == 0:
                if self.scan.min_front < 0.25:
                    l,a = -0.2,0

                elif self.scan.min_dfront < 0.55 and min(self.scan.fleft) < 0.45 and min(self.scan.fright) < 0.45:
                    if min(self.scan.dright) > min(self.scan.dleft):
                        l,a = 0,1
                    else:
                        l,a = 0,-1
                    self.counter += 1

                else:
                    l,a = 0.2, 0

                    if min(self.scan.fleft) < 0.5 and min(self.scan.fright) < 0.5:
                        if min(self.scan.fright) > min(self.scan.fleft):
                            l,a = 0.26, -0.2
                        else:
                            l,a = 0.26, 0.2
                    
                    elif self.scan.min_dfront < 0.7:
                        if min(self.scan.fright) > min(self.scan.fleft):
                            l,a = 0.26, -1.2
                        else:
                            l,a = 0.26, 1.2

                    elif min(self.scan.fleft) < 0.35 or min(self.scan.fright) < 0.35:
                        if min(self.scan.fright) > min(self.scan.fleft):
                            l,a = 0.26, -0.5
                        else:
                            l,a = 0.26, 0.5

                self.move.set_and_pub(l,a)

            elif self.counter == 1:
                self.counter += 1
                self.yaw = self.odom.yaw
            
            elif self.counter > 1:
                self.counter += 1
                if self.scan.max_front > 0.65 or self.counter > 60:
                    self.counter = 0
            self.rate.sleep()
        

if __name__ == '__main__':
    obj = maze_nav()
    try:
        obj.launch()
    except rospy.ROSInterruptException:
        pass