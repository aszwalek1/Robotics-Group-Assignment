#! /usr/bin/env python3

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from tuos_ros_msgs.msg import SearchAction, SearchFeedback, SearchResult, SearchGoal

# Import the tb3 modules from tb3_2.py
from tb3_2 import Tb3Move, Tb3Odometry, Tb3LaserScan

# Import some other useful Python Modules
from math import sqrt, pow

class ObstacleAvoidServer():
    feedback = SearchFeedback() 
    result = SearchResult()
    
    def __init__(self):
        self.actionserver = actionlib.SimpleActionServer(
            "/obstacle_search", 
            SearchAction, 
            self.action_server_launcher, 
            auto_start=False
        )
        self.actionserver.start()

        # pull in some useful publisher/subscriber functions from the tb3.py module:
        self.vel_controller = Tb3Move()
        self.tb3_odom = Tb3Odometry()
        self.tb3_lidar = Tb3LaserScan()

        rospy.loginfo("The server is active")

    def angle_away(self, location):
        if location > 0:
            self.vel_controller.set_move_cmd(linear=0.1, angular=0.2)
            self.vel_controller.publish()
            rospy.sleep(0.5)

        else:   
            self.vel_controller.set_move_cmd(linear=0.1, angular=-0.2)
            self.vel_controller.publish()
            rospy.sleep(0.5) 

        self.vel_controller.set_move_cmd(linear=0.0, angular=0.0)
        self.vel_controller.publish()

    
    def check_if_corner(self, loc_of_turn):
        if self.last_turn == 0:
            self.last_turn = loc_of_turn
        else:
            if (self.last_turn > 0) and (loc_of_turn < 0):
                self.turn_counter += 1
            elif (self.last_turn < 0) and (loc_of_turn > 0): 
                  self.turn_counter += 1
            elif (self.last_turn > 0) and (loc_of_turn > 0):
                self.turn_counter = 0
            elif (self.last_turn < 0) and (loc_of_turn < 0):
                self.turn_counter = 0
            if self.turn_counter == 2:
                print("in a corner")
                self.in_corner = True
                self.turn_counter = 0
        self.last_turn = loc_of_turn    

    def turn(self, location):
        print("turning......")
        self.check_if_corner(location)
        
        if self.in_corner == True:
            sleep_time = 12
            self.in_corner = False
        elif -2 < location < 2:
            sleep_time = 10
        elif -6 < location < 6:
            sleep_time = 8
        elif -10 < location < 10:
            sleep_time = 6
        elif -14 < location < 14:
            sleep_time = 4  
        else:
            sleep_time = 3

        if location > 0:
            self.vel_controller.set_move_cmd(linear=0.0, angular=0.2)
            self.vel_controller.publish()
            rospy.sleep(sleep_time)

        else:   
            self.vel_controller.set_move_cmd(linear=0.0, angular=-0.2)
            self.vel_controller.publish()
            rospy.sleep(sleep_time)   

        
        self.vel_controller.set_move_cmd(linear=0.0, angular=0.0)
        self.vel_controller.publish()


    # The action's "callback function":
    def action_server_launcher(self, goal: SearchGoal):
        rate = rospy.Rate(10)

        success = True
        vel = goal.fwd_velocity # m/s
        dist = goal.approach_distance # m
        min_dist = 0.5
        self.last_turn = 0
        self.turn_counter = 0
        self.in_corner = False
        constant_left = False

        if vel > 0.26 or vel < 0:
            print("invalid velocity!")
            success = False

        if min_dist < 0.2:
            print("invalid distance")
            success = False

        if dist < 0.2:
            print("invalid distance")
            success = False

        if not success:
            self.result.total_distance_travelled = -1.0
            self.result.closest_object_distance = -1.0
            self.result.closest_object_angle = -1.0
            self.actionserver.set_aborted(self.result)
            return
        
        print(f"Search goal recieved : fwd_vel = {vel}, distance = {dist}")
        while success:
            
            # Get the robot's current odometry from the Tb3Odometry() class:
            self.posx0 = self.tb3_odom.posx
            self.posy0 = self.tb3_odom.posy
            # Get information about objects up ahead from the Tb3LaserScan() class:
            self.closest_object = self.tb3_lidar.min_distance
            self.closest_object_location = self.tb3_lidar.closest_object_position

            if constant_left:
                self.vel_controller.set_move_cmd(linear=vel, angular=0.05)
            else:
                self.vel_controller.set_move_cmd(linear=vel, angular=-0.05)
        
            while self.tb3_lidar.min_distance > dist:
                self.yaw0 = self.tb3_odom.yaw

                # update LaserScan data:
                self.closest_object = self.tb3_lidar.min_distance
                self.closest_object_location = self.tb3_lidar.closest_object_position

                self.vel_controller.publish()

                # determine how far the robot has travelled so far:
                self.distance = sqrt(pow(self.posx0 - self.tb3_odom.posx, 2) + pow(self.posy0 - self.tb3_odom.posy, 2))

                self.result.total_distance_travelled = self.distance
                self.result.closest_object_distance = self.closest_object
                self.result.closest_object_angle = self.closest_object_location

                # check if there has been a request to cancel the action mid-way through:
                if self.actionserver.is_preempt_requested():
                    print("Pre-empt request")
                    self.actionserver.set_preempted(self.result)
                    self.vel_controller.stop()
                    success = False
                    # exit the loop:
                    break
            
                self.feedback.current_distance_travelled = self.distance
                self.actionserver.publish_feedback(self.feedback)

                rate.sleep()
            
            if constant_left:
                constant_left = False
            else:
                constant_left =True

            if self.tb3_lidar.min_distance > min_dist:
                print("Object detected")
                self.angle_away(self.closest_object_location)

            else:
                self.vel_controller.stop()
                print("Object detected")
                self.turn(self.closest_object_location)

        if success:
            self.vel_controller.stop()
            self.actionserver.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node("obstacle_avoid_server")
    ObstacleAvoidServer()
    rospy.spin()

