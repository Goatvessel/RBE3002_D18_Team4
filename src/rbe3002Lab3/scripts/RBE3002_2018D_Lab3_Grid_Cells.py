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


# ------------------------------ Map Functions ------------------------------ #

# Function: Map Callback
# Input: Global Map
# Operation: Reads in global map
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
    print("Map Loaded")
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


# ---------------------------- Pathfinding Functions ------------------------ #

# Function: Perform all aspects of pathfinding
# Operation:
    # Use A* algorithm to find optimal path from start to goal
    # Generate a set of waypoints along optimal path
    # Generate a set of poses for each waypoint
def pathFinding():
    global startCell
    global goalCell
    if (startCell is not None and goalCell is not None):
        pathList = aStar(startCell,goalCell)
        #print(pathList)
        waypointList = waypoints(pathList)
        #print(waypointList)
        wayPath = wayposes(waypointList)
    else:
        print(" = Select a START and END pose =")
        return
    print("")
    print(" = Select a new START or END pose =")

# Function: A* (A-Star) path planning algorithm
# Input: Start Index, Goal Index
# Output: List of Indices
def aStar(start,goal):
    # Send blank messages to clear rviz
    gridPathPub.publish(generateGridCells([]))
    wayGridPub.publish(generateGridCells([]))
    blankPath = Path()
    blankPath.header.frame_id = 'map'
    wayPathPub.publish(blankPath)

    # Initialize Variables
    frontier = PriorityQueue() # PriorityQueue of Frontier
    frontier.put((0, start)) # PriorityQueue sorted by F-Score
    parent = {} # Dictionary of best parents
    openSet = [] # List of visited indices - for visualizing
    frontierList = [start] # List of indices on the frontier - for visualizing
    cost = {start:0} # Dictionary of true costs

    print(" A*: Initializing A* Algorithm")
    print(" A*: Searching for Best Path")

    while not frontier.empty():
        # Extract Index and F-score from top of PriorityQueue
        currentTuple = frontier.get()
        currentFScore = currentTuple[0]
        currentIndex = currentTuple[1]
        # Add current Index to the openSet of visited indices
        openSet.append(currentIndex)
        # Remove current Index from the frontierList - for visualizing
        frontierList.remove(currentIndex)

        # Check for completion of pathfinding
        if currentIndex == goal:
            print(" A*: Found Goal")
            break

        # Locate valid neighbors
        neighbors = getNeighbors(currentIndex)
        # Add neighbors to the frontier if they haven't been seen before
        # Update F-score of neighbors that have a better parent (lower G-Score)
        for neighbor in neighbors:
            gScore = cost[currentIndex]+getEuclidean(neighbor,currentIndex)
            if (neighbor not in cost) or (cost[neighbor] > gScore ):
                hScore = getEuclidean(neighbor,goal)
                fScore = gScore + hScore
                cost.update({neighbor:gScore})
                frontier.put((fScore,neighbor))
                frontierList.append(neighbor)
                parent.update({neighbor:currentIndex})

        rospy.sleep(.01) # Delay for visual effect
        # Display Frontier and Current Index in rviz
        frontierCells = generateGridCells(frontierList)
        currentIndexCells = generateGridCells([currentIndex], 1)
        openSetPub.publish(generateGridCells(openSet))
        frontierPub.publish(frontierCells)

    # Generate a list of indices between startCell and goalCell
    pathList = [goal]
    parentIndex = parent[goal]
    while (parent[parentIndex] != start):
        pathList.append(parentIndex)
        parentIndex = parent[parentIndex]
    pathList.append(parentIndex)
    print(" A*: Generated Path List")

    # Display path of GridCells in rviz
    pathCells = generateGridCells(pathList,2)
    currentIndexPub.publish(generateGridCells([]))
    gridPathPub.publish(pathCells)
    return pathList

# Function: Generates a list of waypoints from a path
# Input: List of Indices
# Output: List of Indices
def waypoints(indexList):
    # Initialize variables
    waypointList = [startCell] # List of Waypoints as Map Indices
    pathList = [] # Path list from start index to goal index
    lastNode = startCell

    print(" WP: Initializing Waypoints")
    # Re-order path list from startCell to goalCell
    for i in range (1, len(indexList)+1):
        pathList.append(indexList[-i])

    # Generate list of indices that require a change in orientation
    for node in pathList:
        if (getXY(node)[0] != getXY(waypointList[-1])[0] and getXY(node)[1] != getXY(waypointList[-1])[1]):
            #print("Current X: ", getXY(node)," WP X: ",getXY(waypointList[-1])[0])
            #print("Current Y: ", getXY(node)," WP Y: ",getXY(waypointList[-1])[1])
            waypointList.append(lastNode)
        lastNode = node

    # Add goalCell to the list of waypoints if it was not there previously
    if goalCell not in waypointList:
        waypointList.append(goalCell)

    print(" WP: Generated Waypoints List")
    # Display waypoints in rviz
    waypointGridCells = generateGridCells(waypointList,5)
    currentIndexPub.publish(generateGridCells([]))
    wayGridPub.publish(waypointGridCells)

    return waypointList

# Function: Generate a Path Message from a list of waypoints
# Input: List of Map Indices
# OutputL Path Message
def wayposes(waypointList):
    # Initialize variables
    wayPoses = []

    print(" Path: Initializing Path")
    # Generate a list of PoseStamped Messages from list of waypoints
    for waypoint in range(1,len(waypointList)):
        # Initialize new PoseStamped Message
        lastPose = PoseStamped()
        lastPose.header.frame_id = 'map'

        # Calculate previous position
        lastXY = getXY(waypointList[waypoint-1])
        lastxCoord = lastXY[0]
        lastyCoord = lastXY[1]

        # Calculate current position
        currentXY = getXY(waypointList[waypoint])
        currentxCoord = currentXY[0]
        currentyCoord = currentXY[1]

        # Set position to previous position
        lastPose.pose.position.x = lastxCoord
        lastPose.pose.position.y = lastyCoord
        lastPose.pose.position.z = 0

        # Calculate angle between last position and current position
        theta = math.atan2(currentyCoord-lastyCoord,currentxCoord-lastxCoord)
        (roll, pitch, yaw) = (0, 0, theta)
        q = quaternion_from_euler(roll, pitch, yaw)

        # Set orientation to angle between last position and current position
        lastPose.pose.orientation.x = q[0]
        lastPose.pose.orientation.y = q[1]
        lastPose.pose.orientation.z = q[2]
        lastPose.pose.orientation.w = q[3]

        # Add new PoseStamped Message to list of waypoint Poses
        wayPoses.append(lastPose)

    print(" Path: Generated List of Poses")
    # Generate Path Message from PoseStamped Messages
    wayPath = Path()
    wayPath.header.frame_id = 'map'
    wayPath.poses = wayPoses

    print(" Path: Generated Path Message")
    # Display Path in rviz
    wayPathPub.publish(wayPath)
    return wayPath


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
