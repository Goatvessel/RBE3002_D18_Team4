#!/usr/bin/env python

# ---------------------------------- Imports -------------------------------- #

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf
import numpy
import math
import rospy, tf, numpy, math
from Queue import PriorityQueue




# ----------------------------- Helper Functions ---------------------------- #


# ------------------------------- Main Functions ---------------------------- #

# Main handler of the project
def run():
    # Set publishers as Global Variables
    global mapPub
    global pubPath
    global startPub
    global goalPub
    global frontierPub
    global openSetPub
    global currentIndexPub
    global gridPathPub
    global wayGridPub
    global wayPathPub

    # Set Important Indices as Global Variables
    global startCell
    global goalCell
    global wallIndices

    # Initialize Variables
    wallIndices = []
    startCell = None
    goalCell = None
    print("")
    print(" RBE 3002 D18 Lab 3")
    print("")
    print("Initializing Pubs and Subs")

    # Set Pubs and Subs
    rospy.init_node('lab3')
    mapSub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    mapPub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    startPub = rospy.Publisher("/start_cell", GridCells, queue_size=1)
    goalPub = rospy.Publisher("/goal_cell", GridCells, queue_size=1)
    frontierPub = rospy.Publisher("/frontier", GridCells, queue_size=1)
    openSetPub = rospy.Publisher("/openSet", GridCells, queue_size=1)
    currentIndexPub = rospy.Publisher("/currentIndex", GridCells, queue_size=1)
    gridPathPub = rospy.Publisher("/realPath", GridCells, queue_size=1) # Gridcell path from start to end
    wayPathPub = rospy.Publisher("/wayPath", Path, queue_size=1) # Path path of waypoints from start to end
    wayGridPub = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # Wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    print("Pubs and Subs Initialized")
    print("- - Begin Operation - -")

    while (not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)
    print("- - End Operation - -")

# Standalone operation
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
