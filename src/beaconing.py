#! /usr/bin/python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import math
import time 
# Import some image processing modules:
import cv2
from cv_bridge import CvBridge, CvBridgeError
# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
# Import some other modules from within this package
from tb3 import Tb3Move, Tb3Odometry, Tb3LaserScan


from actionlib_msgs.msg import GoalStatusArray

# Import other modules
import numpy as np
import math

import subprocess

class Beaconing():
    
    def __init__(self):
        

        node_name = "Task4"
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(200)
        
        self.move = Tb3Move()
        self.odom = Tb3Odometry()
        self.scan = Tb3LaserScan()

        self.start_zone = 'A'
        self.start_pos = dict()
        self.start_pos['A'] = [-2.067, -1.973, 1.571]
        self.start_pos['B'] = [-1.240, 2.067, 3.142]
        self.start_pos['C'] = [2.067, 1.973, -1.571] 

        self.goals = dict()
        self.goals['A'] = [-1.45, 0.215, -2.45]
        self.goals['B'] = [-0.46, -0.395, 1.20]
        self.goals['C'] = [1.73, -0.765, -1.50]
        self.goals['D'] = [1.46, 1.175, -2.25]
        self.goals['E'] = [0.45, 1.850, 2.90]

        self.target = None # target color
        self.colour_ranges = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [90, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }

        self.target = None # target color
        self.order = ['A', 'B', 'C', 'D', 'E']
        self.target_colour_bounds = ([0,0,0], [255,255,255])
        
        # subscribe to move base to send goals to slam
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1)

        # subscribe to slam move base to monitor the status of a goal
        self.status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.mbs_callback)
        self.pillar_reached = False  # flag to monitor if the slam goal is reached
        self.found_target = False

        self.cam = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.hsv_img = np.ones((1920, 1080, 3), np.uint8)
        self.target_colour_bounds = ([0, 0, 100], [255, 255, 255])
        self.mask = np.zeros((1920, 1080, 1), np.uint8)
        self.colour_ranges = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Yellow": ([20, 100, 100], [30, 255, 255]),
            "Green":   ([50, 200, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [90, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }

        self.m00 = 0
        self.m00_min = 10000
        self.width_crop = 800
        self.cvbridge_interface = CvBridge()
        
        self.ctrl_c = False
        
        rospy.on_shutdown(self.shutdown_hook)
    
    def shutdown_hook(self):
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.height, self.width, channels = cv_img.shape

        crop_width = self.width - 800
        crop_height = 300
        offset = 50

        crop_y = int((self.height - crop_height - offset) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, 0:self.width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.hsv_img, np.array(self.target_colour_bounds[0]), np.array(self.target_colour_bounds[1]))

        image_mask = cv2.moments(self.mask)
        self.m00 = image_mask['m00']
        self.cy = image_mask['m10'] / (image_mask['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (math.ceil(int(self.cy)), math.ceil(crop_height / 2)), 10, (0, 0, 255), 2)
        cv2.waitKey(1)
       
    def color_detection(self):
        for name, (lower, upper) in self.colour_ranges.items():
            lower_threshold = np.array(lower)
            upper_threshold = np.array(upper)
            mask_checker = cv2.inRange(self.hsv_img, lower_threshold, upper_threshold)

            if mask_checker.any():
                return name, (lower, upper)
    
    def send_goal(self, x, y, theta):
        ps = PoseStamped()

        ps.header.seq = 1
        ps.header.stamp = rospy.Time.now()
        ps.header.frame_id = "map"

        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0

        q = quaternion_from_euler(0, 0, theta)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        self.pub.publish(ps)            
    
    def launch(self):
        self.detect_wall()
        self.detect_color()
        self.set_order()
        self.slam()
        rospy.sleep(4)
        self.beacon()


    def detect_wall(self):
        rospy.sleep(1)
        distance = self.scan.ranges[0]
        if distance > 1 and distance < 1.5:
            self.start_zone = 'B'        
        elif distance > 1.5:
            self.start_zone = 'C'

    def detect_color(self):
        self.move.rotate(120, 1.5)
        rospy.sleep(1)
        self.target, self.target_colour_bounds = self.color_detection()
        self.move.rotate(-120,1.5)
        self.move.set_and_pub(0.0, 0.0)
        
        print("SEARCH INITITATED: The target beacon colour is {}".format(self.target) + '.')

    def set_order(self):
        sz = self.start_zone
        if sz == 'C':
            self.order = ['D', 'C', 'B', 'A', 'E']
        

    def send_goal(self, x, y, theta):
        goal = PoseStamped()

        goal.header.seq = 1
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0

        q = quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.pub.publish(goal)            
            


    def slam(self):
        subprocess.Popen(['roslaunch', 'com2009_team6', 'dwa.launch',
                            'initial_pose_x:=' + str(self.start_pos[self.start_zone][0]),
                            'initial_pose_y:=' + str(self.start_pos[self.start_zone][1]),
                            'initial_pose_a:=' + str(self.start_pos[self.start_zone][2])],
                            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

    def beacon(self):
        for i in self.order:
            self.move_pillar(i)
            if self.complete:
                print('TARGET DETECTED: Beaconing initiated.')
                while self.scan.min_front > 0.3:
                    self.move.set_and_pub(0.2,0)

                self.move.set_and_pub(0.0, 0.0)
                print('BEACONING COMPLETE: The robot has now stopped')
                break
            
    def move_pillar(self, id):
        goal = self.goals[id]
        self.send_goal(goal[0], goal[1], goal[2])
        rospy.sleep(2)

        while not self.pillar_reached:
            rospy.sleep(1)

        if self.pillar_reached:
            self.pub.publish(PoseStamped())
            self.detect_pillar()
            self.pillar_reached = False

        rospy.sleep(1)
    

    def detect_pillar(self):
        selection = self.color_detection()
        if selection is not None:
            name, bounds = selection
            self.complete = bounds == self.target_colour_bounds
        rospy.sleep(1)
        self.move.set_and_pub(0, 0)
       

    def mbs_callback(self, status_array):
        if len(status_array.status_list) > 0:
            latest_status = status_array.status_list[-1].status
            # Check if the latest status represents successful goal completion
            if latest_status == 3:
                self.pillar_reached = True

            
                
if __name__ == "__main__":
    beaconing = Beaconing()
    try:
        beaconing.launch()
    except rospy.ROSInterruptException:
        pass