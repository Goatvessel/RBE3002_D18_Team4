#!/usr/bin/env python

# ---------------------------------- Imports -------------------------------- #
from allImports import *
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

# ------------------------------ Map Functions ------------------------------ #
class map(object):

    def get_mapData(self):
        return self.mapgrid
    # Function: Map Callback
    # Input: Global Map
    # Operation: Reads in global map
    def mapCallBack(self,data):
        # global mapData
        # global width
        # global height
        # global mapgrid
        # global resolution
        # global offsetX
        # global offsetY
        self.mapgrid = data
        self.resolution = data.info.resolution
        self.mapData = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.offsetX = data.info.origin.position.x
        self.offsetY = data.info.origin.position.y
        print("Map Loaded")
        return mapData
        #print data.info

# Function: Publish Map to rviz using GridCells type
# Input: OccupancyGrid.data
def publishCells(grid):
    global wallIndices

    # Initialize variables
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
                index = i*width+j

                # Store a list of Wall Indices - for checking neighbors
                if index not in wallIndices:
                    wallIndices.append(index)
            k = k + 1
    # Display walls in rviz
    mapPub.publish(cells)




# ----------------------------- Helper Functions ---------------------------- #

# Function: Returns Index given X and Y Map Coordinate
# Input: X Coordinate, Y Coordinate
# Output: Map Index
def getIndex(xPos,yPos):
    x = int(xPos/resolution - offsetX/resolution)
    y = int(yPos/resolution - offsetY/resolution)
    index = int(y*width + x)
    return index

# Function: returns X and Y Map Coordinate given Map Index
# Input: Map Index
# Output: X Coordinate, Y Coordinate
def getXY(index):
    y = (index//width)*resolution+offsetY + (0.5 * resolution)
    x = (index%width)*resolution+offsetX + (0.5 * resolution)
    return x, y

# Function: Generate GridCells Message from a list of indices
# Input: List of Map Indices, (Optional) height
# Output: Gridcells
def generateGridCells(indexList,height=0):
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution
    for index in indexList:
        point = Point()
        point.x = (index % width) * resolution + offsetX + (0.5 * resolution)
        point.y = (index // width) * resolution + offsetY + (0.5 * resolution)
        point.z = height
        cells.cells.append(point)
    return cells

# Function: Get neighbor cells of given cell
# Input: Map Index
# Output: List of Map Indices
def getNeighbors(index):
    neighborIndices = []
    #neighborOffsets = [-1,1,-width,width]
    neighborOffsets = [-1,1,-width,width,-width-1,-width+1,width-1,width+1]
    for offset in neighborOffsets:
        neighborIndex = index+offset
        if neighborIndex not in wallIndices:
            neighborIndices.append(neighborIndex)
    return neighborIndices

# Function: Calculate Euclidean distance between two Map Indices
# Input: Start Map Index, Goal Map Index
# Output: Float
def getEuclidean(start,goal):
    startX, startY = getXY(start) # Get Start X and Y coordinates
    goalX, goalY = getXY(goal)    # Get Goal X and Y coordinates
    distance = math.sqrt(pow(goalX-startX,2)+pow(goalY-startY,2)) # Distance Formula
    return distance

# Function: Extract Goal Index for Pathfinding
# Input: Goal Pose Message
# Output: Run Pathfinding
def readGoal(goal):
    global goalCell
    goalX = goal.pose.position.x #gets the x and y position from goal
    goalY = goal.pose.position.y
    goalIndex = []
    goalCell = getIndex(goalX,goalY) #call getIndex function to get the index of goal cell
    #print("Goal Index: ",goalCell)
    goalIndex.append(goalCell)
    cells = generateGridCells(goalIndex,7) #generates goal cell of height 3 i.e highest priority
    goalPub.publish(cells) #publishes goal cell
    #print goal.pose
    pathFinding()

# Function: Extract Start Index for Pathfinding
# Input: Start Pose Message
# Output: Run Pathfinding
def readStart(startPos):
    global startCell
    startPosX = startPos.pose.pose.position.x #gets the x and y position from start position
    startPosY = startPos.pose.pose.position.y
    startIndex = []
    startCell = getIndex(startPosX,startPosY)  #call getIndex function to get the index of goal cell
    #print("Start Index: ",startCell)
    startIndex.append(startCell)
    cells = generateGridCells(startIndex,7) #generates start cell of height 3 i.e highest priority
    startPub.publish(cells)
    #print startPos.pose.pose
    pathFinding()
