#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import degrees, radians
import numpy as np

class Tb3Move(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)
    
    def stop(self):
        self.set_move_cmd()
        self.publish()

    def set_and_pub(self, linear = 0.0, angular = 0.0):
        self.set_move_cmd(linear, angular)
        self.publish()
    
    def rotate(self, deg, vel = 0.8):
        rospy.sleep(1)
        l, a = self.vel_cmd.linear.x, self.vel_cmd.angular.z
        time = radians(abs(deg))/vel

        if deg > 0:
            self.set_and_pub(0, vel)
        else:
            self.set_and_pub(0, -vel)
        
        rospy.sleep(time)
        self.set_and_pub(l, a)

class Tb3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)
    
    def __init__(self):
        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

class Tb3LaserScan(object):
    def laserscan_cb(self, scan_data):

        self.ranges = np.array(scan_data.ranges)
        left = scan_data.ranges[:36]
        right = scan_data.ranges[-35:]
        self.min_front = min(np.array(left[::-1] + right[::-1]))
        self.max_front = max(np.array(left[::-1] + right[::-1]))

        self.fleft = scan_data.ranges[43:48]
        self.fright = scan_data.ranges[313:318]
        
        self.dleft = scan_data.ranges[88:93]
        self.dright = scan_data.ranges[268:273]

        tright = scan_data.ranges[355:]
        tleft = scan_data.ranges[:6]
        self.min_dfront = min(np.array(tleft[::-1] + tright[::-1]))
        self.max_dfront = max(np.array(tleft[::-1] + tright[::-1]))
        
        arc_angles = np.arange(-35, 36)
        self.closest_object_position = arc_angles[np.argmin(np.array(left[::-1] + right[::-1]))]

    def __init__(self):
        self.min_front = 0
        self.max_front = 0

        self.fleft = 0
        self.fright = 0
        
        self.dleft = 0
        self.dright = 0

        self.min_dfront = 0

        self.ranges = 0

        self.min_distance = 0.0
        self.closest_object_position = 0.0 # degrees
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.laserscan_cb) 