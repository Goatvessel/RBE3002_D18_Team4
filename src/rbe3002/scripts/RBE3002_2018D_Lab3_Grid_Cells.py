#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math
import rospy, tf, numpy, math
from Queue import PriorityQueue




# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def getIndex(xPos,yPos):
    xPos = xPos
    yPos = yPos
    x = int(xPos/resolution - offsetX/resolution)
    y = int(yPos/resolution - offsetY/resolution)
    index = int(y*width + x)
    return index

def generateGridCells(indexList):
    cells=GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width=resolution
    cells.cell_height=resolution
    for index in indexList:
        point = Point()
        point.x = (index%width)*resolution+offsetX + (0.5 * resolution)
        point.y = (index//width)*resolution+offsetY + (0.5 * resolution)
        cells.cells.append(point)
    return cells



def readGoal(goal):
    global goalX
    global goalY
    global goalPub
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    goalIndex = []
    goalIndex.append(getIndex(goalX,goalY))
    cells = generateGridCells(goalIndex)
    goalPub.publish(cells)
    print goal.pose

def readStart(startPos):
    global startPosX
    global startPosY
    global startPub
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    startIndex = []
    startIndex.append(getIndex(startPosX,startPosY))
    cells = generateGridCells(startIndex)
    startPub.publish(cells)
    print startPos.pose.pose

def aStar(start,goal):
    pass
    # create a new instance of the map

    # generate a path to the start and end goals by searching through the neighbors, refer to aStar_explanied.py

    # for each node in the path, process the nodes to generate GridCells and Path messages
    while (not openSet.empty()):
        pass

    # Publish points

#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution

    for i in range(0,height): #height should be set to hieght of grid
        for j in range(0,width): #width should be set to width of grid

            # grid values are probability percentages of occupancy. The following condition gives all the cells where
            # occupancy probability is more than 50%
            if (grid[k] > 50):
                point=Point()
                # 0.5*resolution moves the point to the center of the grid cell
                point.x=(j*resolution)+offsetX + (0.5 * resolution)
                point.y=(i*resolution)+offsetY + (0.5 * resolution)
                point.z=0
                cells.cells.append(point)
            k = k + 1
    pub.publish(cells)

#Main handler of the project
def run():
    global pub
    global startPub
    global goalPub
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    startPub = rospy.Publisher("/start_cell", GridCells, queue_size=1)
    goalPub = rospy.Publisher("/goal_cell", GridCells, queue_size=1)
    frontierPub = rospy.Publisher("/frontier", GridCells, queue_size=1)
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    #pathCells = generateGridCells([110,111,112,113])



    while (not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        #frontierPub.publish(pathCells)
        rospy.sleep(2)
        print("Complete")



if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
