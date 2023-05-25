#! /usr/bin/env python3
# search_client.py

from cgitb import reset
from unittest import result
import rospy
import actionlib

from tuos_ros_msgs.msg import SearchAction, SearchGoal, SearchFeedback

class ObstacleAvoidClient():
    goal = SearchGoal()

    def feedback_callback(self, feedback_data: SearchFeedback):

        self.distance = feedback_data.current_distance_travelled
        if self.msg_counter > 5:
            self.msg_counter = 0
        else:
            self.msg_counter+=1
 

    def __init__(self):
        self.distance = 0.0
        self.msg_counter = 0

        self.goal.approach_distance = 0.8
        self.goal.fwd_velocity = 0.2

        self.action_complete = False
        rospy.init_node("obstacle_avoid_client")
        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient(
            "/obstacle_search",
            SearchAction
        )
        self.client.wait_for_server()
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        if not self.action_complete:
            rospy.logwarn("Received a shutdown request. Cancelling Goal...")
            self.client.cancel_goal()
            rospy.logwarn("Goal Cancelled...")
        rospy.sleep(1)

    def main_loop(self):

        
        rospy.sleep(1)
        self.client.send_goal(self.goal, feedback_cb=self.feedback_callback)

        while self.client.get_state() < 2:
            self.rate.sleep()
        
        self.action_complete = True if self.client.get_state() == 3 else False

if __name__ == '__main__':
    node = ObstacleAvoidClient()
    try:
        node.main_loop()
    except rospy.ROSInterruptException:
        pass
