#!/usr/bin/env python

# ---------------------------------- Imports -------------------------------- #
from allImports import *
from utilityFunctions import *
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

# ---------------------------------- Functions -------------------------------- #
# Function: Perform all aspects of pathfinding
# Operation:
    # Use A* algorithm to find optimal path from start to goal
    # Generate a set of waypoints along optimal path
    # Generate a set of poses for each waypoint
def pathFinding(startCell, goalCell):
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
    return wayPath

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
