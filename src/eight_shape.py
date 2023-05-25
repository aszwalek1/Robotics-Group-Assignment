#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class EightShape():
    def callback_function(self, topic_data: Odometry):
        # obtain relevant topic data: pose (position and orientation):
        pose = topic_data.pose.pose
        position = pose.position
        orientation = pose.orientation

        # convert orientation co-ords to roll, pitch & yaw 
        # (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w], "sxyz"
        )

        if self.first_measurement:
            self.first_measurement = False
            self.start_orientation = yaw

        self.x = pose.position.x
        self.y = pose.position.y
        self.theta_z = yaw - self.start_orientation
    
    def __init__(self):
        self.circle_count = 0
        self.circle_check_timer = -1
        self.rate = 10 # hz
        self.first_measurement = True
        self.start_orientation = 0
        
        # setup a '/cmd_vel' publisher and an '/odom' subscriber:
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
        self.sub = rospy.Subscriber("/odom", Odometry, self.callback_function)

        rospy.init_node("eight_shape", anonymous=True)
        rospy.Rate(self.rate)
        # define the robot pose variables and initialise them to zero:
        # variables for the robot's "current position":
        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0

        # define a Twist message instance, to set robot velocities
        self.vel = Twist()

        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the /eight_shape node has been initialised...")

    def shutdownhook(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

        self.pub.publish(Twist())

    def check_for_origin(self):
        if round(self.theta_z, 1) == 0:
            if self.circle_count == 0 and self.circle_check_timer < 0:
                self.vel.linear.x = 0.11
                self.vel.angular.z =  self.vel.linear.x / 0.5
                self.circle_check_timer = 10
                self.circle_count += 1

            elif self.circle_count == 1 and self.circle_check_timer < 0:
                self.vel.angular.z = -self.vel.linear.x / 0.5
                self.circle_check_timer = 10
                self.circle_count += 1

            elif self.circle_count > 1 and self.circle_check_timer < 0:
                self.vel.linear.x = 0
                self.vel.angular.z = 0

        self.circle_check_timer -= 1
            
    def main_loop(self):
        counter = 0

        while not rospy.is_shutdown():
            self.check_for_origin()
            if counter % self.rate == 0:
                print(f"x={self.x:.2f} [m], y={self.y:.2f} [m], yaw={math.degrees(self.theta_z):.1f} [degrees].")

            self.pub.publish( self.vel)
            counter += 1
            rospy.sleep(1.0/self.rate)

if __name__ == "__main__":
    node = EightShape()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass