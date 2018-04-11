#!/usr/bin/env python

# ---------------------------------- Imports -------------------------------- #
from allImports import *
import pathfind as pf
import pubsAndSubs as pub
import utilityFunctions as util

#
# import rospy
# from nav_msgs.msg import GridCells
# from std_msgs.msg import String
# from nav_msgs.msg import Path
# from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry, OccupancyGrid
# from kobuki_msgs.msg import BumperEvent
# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_from_euler
# import tf
# import numpy
# import math
# import rospy, tf, numpy, math
# from Queue import PriorityQueue


# ------------------------------- Main Functions ---------------------------- #

# Main handler of the project
def run():

    print("")
    print(" RBE 3002 D18 Lab 3")
    print("")
    print("Initializing Pubs and Subs")

    PSUB = pub.pubsAndSubs()

    PSUB.setup_pubs()
    PSUB.setup_subs()

    # Wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    print("Pubs and Subs Initialized")
    print("- - Begin Operation - -")

    while (not rospy.is_shutdown()):
        util.publishCells(util.map.get_mapData(PSUB.map_sub)) #publishing map data every 2 seconds
        rospy.sleep(2)
    print("- - End Operation - -")

# Standalone operation
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
