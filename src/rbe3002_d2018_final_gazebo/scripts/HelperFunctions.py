#!/usr/bin/env python

# ---------------------------------- Imports -------------------------------- #

import rospy
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import GridCells, Odometry
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, PolygonStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
import tf
import numpy
import copy
import math
import rospy, tf, numpy, math
from Queue import PriorityQueue
from random import randint


# ------------------------------ WIP Functions ------------------------------ #
# Function : Updates status of the explore
def exploreStatus(theStatus):
    global explore
    PENDING = 0     # The goal has yet to be processed by the action server
    ACTIVE = 1      # The goal is currently being processed by the action server
    PREEMPTED = 2   # The goal received a cancel request after it started executing
                    #   and has since completed its execution (Terminal State)
    SUCCEEDED = 3   # The goal was achieved successfully by the action server (Terminal State)
    ABORTED = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    REJECTED = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    PREEMPTING = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    RECALLING = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    RECALLED = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    LOST = 9   # An action client can determine that a goal is LOST. This should not be
        #    sent over the wire by an action server

    if theStatus.status_list == []:
        realStatus = 0
    else:
        realStatus = theStatus.status_list[0].status
    print(" Exploration Status: ", realStatus)

    if (realStatus != 1):
        explore = True
    elif (realStatus == 1):
        explore = False
    #print("Explore? ",explore)

    if realStatus == PENDING:
        print(" -   Pending")
    elif realStatus == ACTIVE:
        print(" -   Active")
    elif realStatus == PREEMPTED:
        print(" -   Preempted")
    elif realStatus == SUCCEEDED:
        print(" -   Succeeded")
    elif realStatus == ABORTED:
        print(" -   Aborted")
    elif realStatus == REJECTED:
        print(" -   Rejected")
    elif realStatus == PREEMPTING:
        print(" -   Preemting")
    elif realStatus == RECALLING:
        print(" -   Recalling")
    elif realStatus == RECALLED:
        print(" -   Recalled")
    elif realStatus == LOST:
        print(" -   Lost")
    else:
        print("__Very very confused__")
    print("")


def checkNearWall(index):
    global wallIndices
    nearWall = False

    nearby = getNearbyIndices(index)
    for near in nearby:
        if near in wallIndices:
            nearWall = True
            break
    return nearWall

# DORA
# Function: main exploratioin function loads map and goes to new goal indice to explore the whole map and stops once the map is explored
# Input:
# Output:
#Might have to be in robot class
def theExplorer(data):
    global turtle
    #print ("data",data)
    #print("begining to Explore")
    print(" - - Exploring - - ")
    # runs through all the frontierLists
    frontierList = []
    #for i in frontierLists:
    # Make of frontierLists using the global map

    #while len(frontierLists) == 0:
    frontierList = GenFrontierList(data)
    print("Frontier List: ",len(frontierList))
    frontierGroups = SortFrontierList(frontierList)
    #print("Groups: ",frontierGroups)
    #print ("frontierLists",frontierList)

    # Find the longest frontier group
    currentFrontier = longestFrontierList(frontierGroups)
    print("Found Frontier")
    #print("Current Frontier: ",currentFrontier)
    # Find its middle  ......................................needs work
    currentMiddle = findMiddle(currentFrontier)

    #print("Goal: ",currentMiddle)
    currentX = turtle._current.position.x
    currentY = turtle._current.position.y
    print("Goal XY: ", getXY(currentMiddle))
    print("Current XY: ",currentX," ",currentY)
    goalPub.publish(generateGridCells([currentMiddle],15))

    goalPose = PoseStamped()
    x,y = getXY(currentMiddle)

    goalPose.header.frame_id = 'map'
    goalPose.pose.position.x = x
    goalPose.pose.position.y = y
    goalPose.pose.position.z = 0
    goalPose.header.stamp = rospy.Time.now()
    goalPose.pose.orientation.w = 1

    #print("X: ",goalPose.pose.position.x," Y: ",goalPose.pose.position.y)

    tolerance = 1
    currentIndex = getIndex(turtle._current.position.x,turtle._current.position.y)
    if getEuclidean(currentMiddle,currentIndex) < tolerance:
        currentMiddle = currentIndex


    navstackPub.publish(goalPose)
    # rospy.sleep(0.10)


    # navigate.........might use astar also but blehh or gen trajectory
    #turtle.navToPose(currentMiddle)
    #FIXME use nav stack by calling a posestampeed message to publist to move to goal simple


#FIXME do we need to call the map server at this point and "store" the map to another location, so we can pass it to A*?


#Fuction: Take in map list of indices and return frontier lists
def GenFrontierList(data):
    print(" - Generating Frontier List - ")
    frontierList = []
    unexplored = -1
    wall = 3


    for index in range(0,len(data)):
        isFrontier = False
        willAdd = False

        # if (index > 77828):
        #     #print("FINISHED 14000 FRONTIERS!!!")
        #     #print("index ", i)
        #     break


        tempIndices = []
        if data[index] != unexplored and data[index] <= wall: #if not obstacle(100) or unkown(-1), check its neighbors
            tempIndices = getNearbyIndices(index)
        for neighbor in tempIndices:
            #if one of the neighbors is unkown
            if data[neighbor] == unexplored:
                frontierList.append(index)
                break
    groupCells = generateGridCells(frontierList,4)
    frontierPub.publish(groupCells)

    return frontierList

def SortFrontierList(frontierList):
    doneWithNode = False
    frontierGroups = []
    frontierGroups.append([frontierList[0]]) #!FIXME Do we need to initialie this with a value?
    #remainingList = frontierList
    for index in frontierList:
        isLonely = True
        neighbors = getNearbyIndices(index)
        doneWithNode = False

        for neighbor in neighbors:
            if doneWithNode:
                break
            for group in range(0, len(frontierGroups)):
                if doneWithNode:
                    break
                if neighbor in frontierGroups[group]:
                    frontierGroups[group].append(index)
                    isLonely = False
                    doneWithNode = True
        if isLonely:
            frontierGroups.append([index])
        #remainingList.remove(index)

    return frontierGroups




#Function: Takes in list of frontier indices and calculates mid node
def findMiddle(frontList):
    isNearWall = True
    while(isNearWall):
        #print("NEAR A WALL")
        if len(frontList) == 0:
            break
        #midIndex = len(frontList)//2
        checkHere = randint(0, len(frontList) - 1)
        middle = frontList[checkHere]
        isNearWall = checkNearWall(middle)
        if isNearWall:
            frontList.remove(middle)

    #print("middle vb mvb",middle)
    return middle
#Function takes in list of lists of indices and returns the longest list
def longestFrontierList(frontierGroupList):
    longest = []
    thresh = 3
    print("Frontier Groups: ")
    for group in frontierGroupList:
        print("Length of group: ",len(group))

        if len(group) > len(longest):
            longest = group
    if len(longest) <= thresh:
        longest = []
    print("Longest ", longest)
    #groupCells = generateGridCells(longest,thresh)
    #frontierPub.publish(groupCells)

    return longest

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
    global count
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    #print("mapdata : ", mapData)
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    #resizeCells(mapData)
    theExplorer(mapData)
    print(" / Map Loaded / ")
    #
    # filename = "blank"
    # count = count + 1
    # if count%500 == 0:
    #     yaml = filename + String(count)
    #     image = "Map_v" + String(count//500)
    #






    #getResizedNodes(mapData)
    #getGridUpdate(mapData)
    #print data.info

# This is a new function
def newFunction(arg1, arg2, arg3):
    return arg1 + arg2 + arg3

# Function: Publish Map to rviz using GridCells type
# Input: OccupancyGrid.data
def publishCells(grid):
    global wallIndices

    # Initialize variables
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution
    k = 0

    for i in range(0,height):
        for j in range(0,width):
                if (grid[k] > 50):
                    indexChecklist = getNearbyIndices(k)
                    for index in indexChecklist:
                        if index not in wallIndices:
                            wallIndices.append(index)
                k = k + 1
    wallPoints = getPointsFromIndicies(wallIndices)
    #print("Length of wallPoints: ",len(wallIndices))
    for point in wallPoints:
        outPoint = Point()
        outPoint.x = point[0]
        outPoint.y = point[1]
        outPoint.z = 0

        cells.cells.append(outPoint)
        #print(point)
    mapPub.publish(cells)

def getPointsFromIndicies(wallIndices):
    listOfPoints = []
    for index in wallIndices:
        listOfPoints.append(getXY(index))
    return listOfPoints

def getNearbyPoints(point):
    pX = point.x
    pY = point.y
    listOfPoints = []
    for i in range(-3,4):
        for j in range(-3,4):
            newPointX = pX + i*resolution
            newPointY = pY + j*resolution
            point.x = newPointX
            point.y = newPointY
            listOfPoints.append(point)
    return listOfPoints

def getNearbyIndices(index):
    global radius
    indexChecklist = []
    for i in range(-radius,+radius+1):
        for j in range(-radius,radius+1):
            newIndex = index + width*j + i
            if newIndex not in indexChecklist:
                indexChecklist.append(newIndex)
                #print(len(indexChecklist))
    return indexChecklist

def getSquare(index):
    pass
# First pass: width / 3

# Function: Publish Map to rviz using GridCells type
# Input: OccupancyGrid.data
def publishCells01(grid):
    global wallIndices
    global thicc

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
                index = i*width+j
                point=Point()
                point.z=0
                # 0.5*resolution moves the point to the center of the grid cell
                wallX = (j*resolution)+offsetX + (0.5 * resolution)
                wallY = (i*resolution)+offsetY + (0.5 * resolution)
                for k in range(0,6):
                    for m in range(0,6):
                        point.x = wallX + (k-3)*resolution
                        point.y= wallY + (m-3)*resolution
                        cells.cells.append(point)
                        indexCheck = index + width*m + k
                        # Store a list of Wall Indices - for checking neighbors
                        if indexCheck not in wallIndices:
                            wallIndices.append(indexCheck)
            k = k + 1
    # Display walls in rviz
    mapPub.publish(cells)

#Groups cells together to "optimize" the Astar algorithm
# def resizeCells(rfactor = 0):
#     #global newIndexList[]
#
#     global mapData
#     global width
#     global height
#     global mapgrid
#     global resolution
#     global offsetX
#     global offsetY
#     mapgrid = data
#     resolution = data.info.resolution
#     mapData = data.data
#     width = data.info.width
#     height = data.info.height
#     offsetX = data.info.origin.position.x
#     offsetY = data.info.origin.position.y
#
#     newRes = resolution/rfactor
#
#
#     for i in mapData:
#
#         for j in mapdata:
#             j = j + rfactor
#
#         i = i+rfactor
#
#     return newRes


def getGridUpdate(grid):
    global gridIndices

    # Initialize variables
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution
    cells.cell_height = resolution
    k = 0

    for i in range(0,height):
        for j in range(0,width):
            if (grid[k] < 50):
                indexChecklist = getNearbyIndices(k)
                for index in indexChecklist:
                    if index not in wallIndices:
                        wallIndices.append(index)
                k = k + 1
    gridIndices = 1
    wallPoints = getPointsFromIndicies(wallIndices)
    #print("Length of wallPoints: ",len(wallIndices))
    for point in wallPoints:
        outPoint = Point()
        outPoint.x = point[0]
        outPoint.y = point[1]
        outPoint.z = 0

        cells.cells.append(outPoint)
        #print(point)
    mapPub.publish(cells)


# Function:
# Input: grid and cell distance
# Output:
#
# def filterIndeces(grid, mode=4): # mode determines how many empty cells between checks
#         global width
#         global height
#         global something
#
#         i = 0
#         while (i < width*height):
#             i = i + mode
#         #    if ()
#                 i = i + width
#
#
#     for i in range(0,height):
#         for j in range(0,width):
#                 if (grid[k] > 50):
#                     indexChecklist = getNearbyIndices(k)
#                     for index in indexChecklist:
#                         if index not in wallIndices:
#                             wallIndices.append(index)
#                 if (j%4 == 0):
#                     k = k + mode
#                 else
#                     k = 1
#
#         i = i + 2



# ----------------------------- Helper Functions ---------------------------- #

# Function: Returns Index given X and Y Map Coordinate
# Input: X Coordinate, Y Coordinate
# Output: Map Index
def getIndex(xPos,yPos):
    x = int(xPos/resolution - offsetX/resolution)
    y = int(yPos/resolution - offsetY/resolution)
    index = int(y*width + x)
    return index

# Function: Returns X and Y Map Coordinate given Map Index
# Input: Map Index
# Output: X Coordinate, Y Coordinate
def getXY(index):
    y = (index//width)*resolution+offsetY + (0.5 * resolution)
    x = (index%width)*resolution+offsetX + (0.5 * resolution)
    return x, y

def getAngleBetweenIndices(startIndex,goalIndex):
    pass

# Function: Converts a Pose message into a list of
# Input: Pose() Message
# Output: [X-Position, Y-Position, Yaw]
def convertPose(myPose):
    #print(type(myPose))
    typePoseCove = type(PoseWithCovarianceStamped())
    typePoseStamp = type(PoseStamped())
    typePose = type(myPose)
    q = []
    xPos = []
    yPos = []
    if typePose == typePoseCove:
        #
        # Where Yaw is the rotation about the Z-Axis
        q = [myPose.pose.pose.orientation.x,
    		myPose.pose.pose.orientation.y,
    		myPose.pose.pose.orientation.z,
    		myPose.pose.pose.orientation.w]
        xPos = myPose.pose.pose.position.x
        yPos = myPose.pose.pose.position.y
    elif typePose == typePoseStamp:
        q = [myPose.pose.orientation.x,
    		myPose.pose.orientation.y,
    		myPose.pose.orientation.z,
    		myPose.pose.orientation.w]
        xPos = myPose.pose.position.x
        yPos = myPose.pose.position.y

    if len(q) == 0:
        print('We have an issue')
    (roll, pitch, yaw) = euler_from_quaternion(q)

    #return [xPos, yPos, yaw]
    return [xPos, yPos]

# Function: Given two indices, locate the center between them
# Input: Index A, Index B
# Output: (X,Y) position
def getCenter(indexCornerA, indexCornerB):
    (aX, aY) = getXY(indexCornerA)
    (bX, bY) = getXY(indexCornerB)
    (midX, midY) = (.5*(aX + bX), .5*(aY + bY))
    return (midX, midY)

# Function: Generate GridCells Message from a list of (x,y) points
# Input: List of (x,y) points, (optional) layer, (optional) height, (optional) width
# Output: Gridcells
def generateGridCellsByPoints(pointList, layer=0, width=1,height=1):
    pass

# Function: Generate GridCells Message from a list of indices
# Input: List of Map Indices, (Optional) height
# Output: Gridcells
def generateGridCells(indexList, height=0, gridWidth=1, gridHeight=1):
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = gridWidth*resolution
    cells.cell_height = gridHeight*resolution
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
def readGoal(goalPose):
    print("READ GOAL :D")
    global goalCell
    global goalX
    global goalY
    global goalYaw
    print("In readGoal")
    goalIndex = []
    #(goalX, goalY, goalYaw) = convertPose(goalPose)
    (goalX, goalY) = convertPose(goalPose)
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
def readStart(startPose):
    global startCell
    global startX
    global startY
    global startYaw
    startIndex = []

    (startX, startY, startYaw) = convertPose(startPose)
    startCell = getIndex(startX,startY)  #call getIndex function to get the index of goal cell
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

        #rospy.sleep(.01) # Delay for visual effect
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
    pointAngle = {startCell:startYaw1}

    print " WP: Initializing Waypoints"
    print "     - Length of indexList: ", len(indexList)
    # Re-order path list from startCell to goalCell
    for i in range (1, len(indexList)+1):
        pathList.append(indexList[-i])

    N=math.pi/2
    NE=math.pi/4
    E=0
    SE=-math.pi/4
    S=-math.pi/2
    SW=-3*math.pi/4
    W=math.pi
    NW=3*math.pi/4


    # Generate list of indices that require a change in orientation
    for i in range(1,len(pathList)-1):
        # If the angle between current pose and next movement is noticable
        (lastX, lastY) = getXY(pathList[i-1])
        (startX, startY) = getXY(pathList[i])
        (nextX, nextY) = getXY(pathList[i+1])
        angleBetweenLastStart = math.atan2(startY-lastY,startX-lastX)
        angleBetweenStartNext = math.atan2(nextY-startY,nextX-startX)
        if (abs(angleBetweenLastStart - angleBetweenStartNext) > .2):
            waypointList.append(pathList[i])


        # #print("Current X: ", getXY(node)," WP X: ",getXY(waypointList[-1])[0])
        #     #print("Current Y: ", getXY(node)," WP Y: ",getXY(waypointList[-1])[1])
        # waypointList.append(lastNode)
        # lastNode = node

    # Add goalCell to the list of waypoints if it was not there previously
    if goalCell not in waypointList:
        waypointList.append(goalCell)
    print (" WP: Generated Waypoints List")
    print "     - Length of Waypoint List: ", len(waypointList)
    # Display waypoints in rviz
    waypointGridCells = generateGridCells(waypointList,5)
    currentIndexPub.publish(generateGridCells([]))
    wayGridPub.publish(waypointGridCells)

    return waypointList

# Function: Generate a Path Message from a list of waypoints
# Input: List of Map Indices
# Output: Path Message
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
        if (waypoint+1 < range(1,len(waypointList))):
            turtle.navToPose(waypointList[waypoint])
            #aStar(waypointList[waypoint], waypointList[-1]) #Start current pos of robot
            #astar(startCell, waypointList[-1])
    print(" Path: Generated List of Poses")
    # Generate Path Message from PoseStamped Messages
    wayPath = Path()
    wayPath.header.frame_id = 'map'
    wayPath.poses = wayPoses

    print(" Path: Generated Path Message")
    # Display Path in rviz
    wayPathPub.publish(wayPath)
    return wayPath

# ---------------------------- Robot Control Functions ------------------------ #

class Robot:

    # Function:
    # Input:
    # Output:
    def __init__(self):
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Publisher Twist messages to cmd_vel topic
        self._current = Pose() # Position and orientation of the robot
        self._odom_list = tf.TransformListener() # Subscribe to transform messages
        rospy.Timer(rospy.Duration(.1), self.timerCallback) # Setup callback - not hard real-time

        #rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # Subscribe to navigation goal messages
        #rospy.Subscriber('goto', PoseStamped, self.navToPose, queue_size=1) # Subscribe to navigation goal messages

    # Function:
    # Input:
    # Output:
    def navToPose(self,goal):
		# Callback function when a navigation goal is defined
		# Extract data from Goal, rotate to Goal position, drive straight to Goal position, rotate to Goal orientation

        global startYaw1
        global startCell
        if startCell == None:
            startCellXY = (turtle._current.position.x, turtle._current.position.y)
        else:
            startCellXY = getXY(startCell)
        while (startCellXY is []):
            rospy.sleep(0.1)

        #Current Pose
        #origin = copy.deepcopy(self._current)
        #initialPose = convertPose(origin)
        #xInitial = initialPose[0] # Current X
        xCurrent = startCellXY[0] # Current X
        #yInitial = initialPose[1] # CUrrent Y
        yCurrent = startCellXY[1] # CUrrent Y
        yawCurrent = startYaw1 # Current Yaw.
        print("current x : ",xCurrent)
        print("current y : ",yCurrent)
        print("current yaw : ",yawCurrent)

        # Goal Pose
        #self._odom_list.waitForTransform('odom', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
        # transform the nav goal from the global coordinate system to the robot's coordinate system
        # GoalPoseStamped = self._odom_list.transformPose('base_link', goal)
        # GoalPose = convertPose(GoalPoseStamped.pose)
        # xGoal = GoalPose[0] # Desired X
        # yGoal = GoalPose[1] # Desired Y
        # yawGoal = GoalPose[2] # Desired yaw - angle about z axis

        #using world frame to navigate instead of the robot frame
        poseType = type(Pose)
        indexType = type(5)
        goalType = type(goal)
        if goalType == indexType:
            goalxy = getXY(goal)
        else:
            goalxy = convertPose(goal)

        xGoal = goalxy[0] # Desired X
        yGoal = goalxy[1] # Desired Y
        yawGoal = math.atan2(yGoal-yCurrent,xGoal-xCurrent)
        print("Goal X: ", xGoal)
        print("Goal Y: ", yGoal)
        print("Goal Yaw: ", yawGoal)


        # Distance to the Goal from the Start
        #straightline = math.sqrt(math.pow(xGoal,2) + math.pow(yGoal,2))
        finalX = xGoal - xCurrent
        finalY = yGoal - yCurrent
        straightline = math.sqrt(math.pow(finalX,2) + math.pow(finalY,2))

        #First rotate towards Goal
        self.ExtremeRotate(yawGoal)

        #Drive towards goal
        self.driveStraighter(0.06,straightline)



        # # Angle between the Initial orientation and the Goal orientation
        # initialYaw = math.atan2(yGoal,xGoal)
        #
        # # Rotate to face the Goal orientation
        # ExtremeRotate(yawGoal - initialYaw)
        # # Drive straight to the Goal position
        # driveStraighter(.2,straightline)
        # # Rotate to face the Goal position
        # ExtremeRotate(initialYaw)

    #navToPose(self, currentMiddle)

    # ---------------------------- Robot Movement Functions ------------------------ #

    # Function:
    # Input:
    # Output:
    def driveStraight(self, speed, distance, _DEBUG_=False):
    	# Drive both wheels at the same speed for a set distance
        self.spinWheels(0,0,.1) # Initialize odometry by spinning wheels with no velocity - !FIXME There must be a better way
        #_DEBUG_ = True

        interval = .01 # [seconds] Check current position against goal position after this time interval
        v_left = speed  # [m/s]
        v_right = speed # [m/s]
        distance = distance # [m]

        origin = copy.deepcopy(self._current)
        currentPose = convertPose(origin)
        initial_x = current_x = currentPose[0]
        initial_y = current_y = currentPose[1]
        current_yaw = currentPose[2]

        goal_location_x = initial_x + distance*math.cos(current_yaw)
        goal_location_y = initial_y + distance*math.sin(current_yaw)

        distanceTraveled = 0
        distanceToGo = distance - distanceTraveled

        while distanceToGo > 0:
            self.spinWheels(v_left, v_right, interval)
            currentPose = convertPose(copy.deepcopy(self._current))
            current_x = currentPose[0]
            current_y = currentPose[1]

            distanceTraveled = math.sqrt(math.pow(current_x - initial_x,2) + math.pow(current_y - initial_y,2))
            distanceToGo = distance - distanceTraveled # Update remaining distance
    		# Debug print current position, distance traveled, and remaining distance to travel
            if _DEBUG_:
                print("Current X: ",current_x)
                print("Current Y: ",current_y)
                print("Distance Traveled: ",distanceTraveled)
                print("Distance To Go: ", distanceToGo)
                print("") # Newline

    # Function:
    # Input:
    # Output:
    def spinWheels(self, v_left, v_right, forTime):
    	# Spin the left and right wheels at set velocities for a length of time

        wheelbase = 0.16 # [meters] based on wheelbase http://www.robotis.us/turtlebot-3-burger-us/

        # Generate Twist Message
        linearVel = (v_right+v_left)/2
        angularVel = (v_right-v_left)/wheelbase
        timeDrive = time.time() + forTime
        Twist = self._get_twist(linearVel,angularVel)

        # Drive wheels for set amount of time
        while (time.time() < timeDrive):
            self._vel_pub.publish(Twist)

        # Stop the robot - to be safe
        StopTwist = self._get_twist(0,0)
        self._vel_pub.publish(StopTwist)

    # Function:
    # Input:
    # Output:
    def rotate(self,angle, deg=False, _DEBUG_=False):
    	# Rotate by an angle [radians]
    	# self.rotate(angle,True) to use degrees instead of radians

        tolerance = 0.04 # [radians]
        speed = 0.04 # [m/s]
        interval = 0.01 # [s] Time between checking Goal Yaw vs Current Yaw
        _DEBUG_ = True
        if angle == 0: # Easy case - rotate zero degrees
    		return
        if deg == True: # Convert degrees to radians
            angle = math.radians(angle)

        self.spinWheels(0,0,.1) # Initialize odometry by spinning wheels with no velocity - !FIXME There must be a better way
        origin = copy.deepcopy(self._current) # Current orientation
        q = [origin.orientation.x,
    		origin.orientation.y,
    		origin.orientation.z,
    		origin.orientation.w] # quaternion nonsense
        (roll, pitch, yaw) = euler_from_quaternion(q)
        initialPose = convertPose(copy.deepcopy(self._current))
        currentYaw = initialPose[2]
        goalYaw = angle + currentYaw # Goal angle

        # Take into account [-pi,pi] range
        if goalYaw > math.pi:
            goalYaw = math.pi-goalYaw
        elif goalYaw < -math.pi:
             goalYaw = math.pi+goalYaw

        while abs(goalYaw - currentYaw) > tolerance:
            if (angle < 0): # Clockwise
                self.spinWheels(speed,-speed,interval)
            else: # Withershins
                self.spinWheels(-speed,speed,interval)
            currentYaw = convertPose(copy.deepcopy(self._current))[2]
            if _DEBUG_:
                print ("Current Angle: ",currentYaw," Distance To Go: ",abs(goalYaw-currentYaw))


    def ExtremeRotate(self, rotation):

        print ("Starting to rotate")

        #convert incoming degrees angle to radians
        radian_angle = rotation #*math.pi/180
        #print "angle :", angle

        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)

        #calculate desired angle
        desired_angle = radian_angle #+ startYaw1

        if (desired_angle > math.pi):
            desired_angle += -2*math.pi
        elif (desired_angle < -math.pi):
            desired_angle += 2*math.pi

        #create a new message of the type twist()
        rotate_message = Twist()

        #fix direction turning
        rotate_message.linear.x = 0.0
        if (radian_angle - startYaw1 > 0):
            rotate_message.angular.z = 0.4
        elif (radian_angle - startYaw1 < 0):
            rotate_message.angular.z = -0.4

        #testing
        # print desired_angle
        # print yaw

        #To get to desired angle from current angle
        while (abs(desired_angle - startYaw1) > (math.pi/90)):
            q = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(q)
            rospy.sleep(.5)
            self._vel_pub.publish(rotate_message)

        # print desired_angle
        # print yaw
        rotate_message.angular.z = 0.0
        self._vel_pub.publish(rotate_message)


    def driveStraighter(self, speed,distance):

        #self.spinWheels(speed, speed, distance/speed)
        #start driving
        print ("Begining to drive")
        origin = copy.deepcopy(self._current) #use this

        #creating a message of the type twist()
        drive_message = Twist()

        #initilizing linear and angular components
        drive_message.linear.x = speed
        drive_message.angular.z = 0

        initialx = origin.position.x
        initialy = origin.position.y
        # to check if we reach destination
        reached_destination = False
        while(not reached_destination):

            accelerateRatio = 0.7
            speedingUp = 0.6
            slowingDown = 0.6

            #get current x and y position of our robot
            currentx = self._current.position.x
            currenty = self._current.position.y
            #Use distance formula to find the distance
            current_distance = math.sqrt(((currentx - initialx)**2) +((currenty - initialy)**2 ))

            if(current_distance >= distance):
                reached_destination = True
                drive_message.linear.x = 0
                self._vel_pub.publish(drive_message)
                print "reached_destination"
                print "distance travelled",current_distance

            #to ramp up speed when distance travelled is less than 80% of the journey
            elif(current_distance < accelerateRatio*distance):
                drive_message.linear.x = speedingUp * speed
                #print "my speed2",drive_message.linear.x

            #to slow down when destination is close
            elif(current_distance >= accelerateRatio*distance):
                drive_message.linear.x = slowingDown * speed

            #to often reach faster to any goal
            else:
                drive_message.linear.x = 2 * speed
            self._vel_pub.publish(drive_message)


    def spinThemWheels(self, v_left, v_right, time):

        #get ros time
        driveStartTime = rospy.Time.now().secs
        L = 0.16 #in centself.imeters # based on wheel track from https://yujinrobot.github.io/kobuki/doxgoalyen/enAppendixKobukiParameters.html
        if (v_right != v_left):
            #calculate R
            R = (L/2)*(v_left + v_right)/(v_right - v_left)
            #Calculate omega
            omega = (v_right - v_left)/L
            #calculate average speed, V
            V = (v_left + v_right)/2
        else:
            omega = 0.0
            V = v_left

        #creating a new message of the type twist()
        spinwheel_message =Twist()
        spinwheel_message.linear.x = V
        spinwheel_message.angular.z = omega
        print "Publishing message : ", spinwheel_message

        #publishing message spin
        self._vel_pub.publish(spinwheel_message)

        stopwheel_message =Twist()
        stopwheel_message.linear.x = 0.0
        stopwheel_message.angular.z = 0.0
        print "Publishing message : ", stopwheel_message
        self._vel_pub.publish(stopwheel_message)

        #To spin wheels for a given time
        while (driveStartTime + time > rospy.Time.now().secs):
            self._vel_pub.publish(spinwheel_message)
        #To stop spinning after desired time has elasped
        self._vel_pub.publish(stopwheel_message)

    # ------------------------------- Robot Helper Functions ---------------------------- #

    # Function:
    # Input:
    # Output:
    def _get_twist(self, linear, angular):
    	# Construct Twist message for differential drive robot based on linear and angular velocity
    	twist = Twist()
    	twist.linear.x = linear
    	twist.angular.z = angular
    	return twist

    # Function:
    # Input:
    # Output:
    def timerCallback(self,evprent):
    	# Wait for and get the transform between two frames
    	self._odom_list.waitForTransform('/odom', '/base_footprint', rospy.Time(0), rospy.Duration(1.0))
    	(position, orientation) = self._odom_list.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
       	# Save the current position and orientation
    	self._current.position.x = position[0]
    	self._current.position.y = position[1]
    	self._current.orientation.x = orientation[0]
    	self._current.orientation.y = orientation[1]
    	self._current.orientation.z = orientation[2]
    	self._current.orientation.w = orientation[3]
    	# Create a quaternion
       	q = [self._current.orientation.x,
    		self._current.orientation.y,
    		self._current.orientation.z,
    		self._current.orientation.w]
    	# convert the quaternion to roll pitch yaw
        (roll, pitch, self.yaw) = euler_from_quaternion(q)

        global startYaw1
        startYaw1 = self.yaw
        #print("Start Yaw 1: ",startYaw1)
        global startCell
        startCell = getIndex(position[0],position[1])
        #print startCell
        #print("Current X: ", position[0])
        #print("Current Y: ", position[1])
        #print("Current Yaw: ", startYaw1)
        #print("")


    # Function:
    # Input:
    # Output:
    def planTraj(self, b, t):
    	pass

    	"""
    	Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
    	"""

    # Function:
    # Input:
    # Output:
    def Poly5(self, startTime, endTime, startVelocity, endVelocity, startAcceleration, endAcceleration, startPosition, endPosition):
        # Returns the coefficients for the quintic polynomial used for generating a smooth trajectory
    	# !FIXME I wish I had time to implement this
    	t0 = startTime; # [seconds]
    	tf = endTime; # [seconds]
    	v0 = startVelocity; # [degrees/second]
    	vf = endVelocity; # [degrees/second]
    	a0 = startAcceleration; # [degrees/second^2]
    	af = endAcceleration; # [degrees/second^2]
    	p0 = startPosition; #[degrees]
    	pf = endPosition; # [degrees]

    	matrix = np.array([
    		[1., t0, t0^2,  t0^3,    t0^4,     t0^5],
    		[0., 1., 2.*t0, 3.*t0^2, 4.*t0^3,  5.*t0^4],
    		[0., 0., 2.,    6.*t0,   12.*t0^2, 20.*t0^3],
    		[1., tf, tf^2,  tf^3,    tf^4,     tf^5],
    		[0., 1., 2.*tf, 3.*tf^2, 4.*tf^3,  5.*tf^4],
    		[0., 0., 2.,    6.*tf,   12.*tf^2, 20.*tf^3]
    		])
    	startEnd = np.array([
    		[p0],
    		[v0],
    		[a0],
    		[pf],
    		[vf],
    		[af]
    		])

    	#matrix = np.array([[1.,2.],[3.,4.]])
    	matrixInverse = np.linalg.inv(matrix)
    	#np.allclose(np.dot(matrix, matrixInverse), np.eye(2))
    	#np.allclose(np.dot(matrixInverse, matrix), np.eye(2))
    	result = np.matmul(matrixInv,startEnd) # Coefficients for the quintic polynomial
    	return result



# ------------------------------- Main Functions ---------------------------- #

def defGlobals():
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
    global turtle
    global explore

    # Set Important Indices as Global Variables
    global startCell
    global goalCell
    global wallIndices
    global radius

    # Initialize Variables

    wallIndices = []

    startCell = None
    goalCell = None
    radius = 3
    explore = True

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
    global navstackPub
    global odomSub
    global turtle
    turtle = Robot()
    global explore
    global count
    count = 1

    explore = True


    # Set Important Indices as Global Variables
    global startCell
    global goalCell
    global wallIndices
    global radius

    # Initialize Variables

    wallIndices = []

    startCell = None
    goalCell = None
    radius = 2
    print("")
    print(" RBE 3002 D18 Lab 4")
    print("")
    print("Initializing Pubs and Subs")

    # Set Pubs and Subs

    mapSub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    frontierPub = rospy.Publisher("/frontier", GridCells, queue_size=1)
    mapPub = rospy.Publisher("/map_check", GridCells, queue_size=1)
    startPub = rospy.Publisher("/start_cell", GridCells, queue_size=1)
    goalPub = rospy.Publisher("/goal_cell", GridCells, queue_size=1)

    openSetPub = rospy.Publisher("/openSet", GridCells, queue_size=1)
    currentIndexPub = rospy.Publisher("/currentIndex", GridCells, queue_size=1)
    gridPathPub = rospy.Publisher("/realPath", GridCells, queue_size=1) # Gridcell path from start to end
    wayPathPub = rospy.Publisher("/wayPath", Path, queue_size=1) # Path path of waypoints from start to end
    wayGridPub = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    navstackPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    #goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goto_sub = rospy.Subscriber('/goto', PoseStamped, readGoal, queue_size=1) #change topic for best results
    #goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    odomSub = rospy.Subscriber("/odom", Odometry, turtle.timerCallback)
    #costmapSub = rospy.Subscriber("/move_base/local_costmap/footprint")

    #front_sub = rospy.Subscriber('/move_base/local_costmap/footprint', PolygonStamped, theExplorer, queue_size=1 )
    #front_sub = rospy.Subscriber('/map', OccupancyGrid, theExplorer, queue_size=1 )


    #exploreCallbackSub = rospy.Subscriber('move_base/status',actionlib_msgs/GoalStatusArray,exploreStatus)
    #exploreCallbackSub = rospy.Subscriber('move_base/status',GoalStatusArray,exploreStatus)
    exploreCallbackSub = rospy.Subscriber('move_base/status',GoalStatusArray,exploreStatus)


    # Wait a second for publisher, subscribers, and TF
    rospy.sleep(.5)
    print("Pubs and Subs Initialized")
    print("- - Begin Operation - -")
    r = rospy.Rate(10)

    while (not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        # if (explore):
        #     #Call the explorer function
        #     print("- - - - - - - - - - - - Explore = True")
        #     theExplorer(mapData)
        # else:
        #     print("- - - - - - - - - - - - Explore = False")
        # r.sleep()
    print("")
    print("- - End Operation - -")

# Standalone operation
if __name__ == '__main__':
    try:
        rospy.init_node('lab5')
        run()

    except rospy.ROSInterruptException:
        pass
else:
    try:
        defGlobals()

    except rospy.ROSInterruptException:
        pass
