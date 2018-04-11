#!/usr/bin/env python

# ---------------------------------- Imports -------------------------------- #
from allImports import *
#from utilityFunctions import *
import utilityFunctions as util

class pubsAndSubs(object):
    def __init__(self):
        rospy.init_node('lab4')
    def setup_pubs(self):
        self.map_pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
        self.start_pub = rospy.Publisher("/start_cell", GridCells, queue_size=1)
        self.goal_pub = rospy.Publisher("/goal_cell", GridCells, queue_size=1)
        self.frontier_pub = rospy.Publisher("/frontier", GridCells, queue_size=1)
        self.openSet_pub = rospy.Publisher("/openSet", GridCells, queue_size=1)
        self.currentIndex_pub = rospy.Publisher("/currentIndex", GridCells, queue_size=1)
        self.gridPath_pub = rospy.Publisher("/realPath", GridCells, queue_size=1) # Gridcell path from start to end
        self.wayPath_pub = rospy.Publisher("/wayPath", Path, queue_size=1) # Path path of waypoints from start to end
        self.wayGrid_pub = rospy.Publisher("/waypoints", GridCells, queue_size=1)


    def setup_subs(self):
        mapInstance = util.map()
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, mapInstance.mapCallBack)
        self.goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, util.readGoal, queue_size=1) #change topic for best results
        self.start_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, util.readStart, queue_size=1) #change topic for best results
