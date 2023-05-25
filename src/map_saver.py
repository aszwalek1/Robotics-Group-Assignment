#!/usr/bin/env python3

import roslaunch
import rospy
import subprocess
import time

loop = True
while loop:
    map_path = "/home/student/catkin_ws/src/com2009_team6/maps/task5_map"

    rospy.init_node("map_getter", anonymous=True)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    print(f"Saving map at time: {rospy.get_time()}...")
    node = roslaunch.core.Node(package="map_server",
                        node_type="map_saver",
                        args=f"-f {map_path}")
    process = launch.launch(node)
    time.sleep(5)
