#!/usr/bin/env python
# Imports
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


# -------------------- Helper Functions ------------------------------ #
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
    print data.info

# Function: Returns Index given X and Y Map Coordinate
# Input: X Coordinate, Y Coordinate
# Output: Map Index
def getIndex(xPos,yPos):
    xPos = xPos
    yPos = yPos
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
    cells=GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width=resolution
    cells.cell_height=resolution
    for index in indexList:
        point = Point()
        point.x = (index%width)*resolution+offsetX + (0.5 * resolution)
        point.y = (index//width)*resolution+offsetY + (0.5 * resolution)
        point.z = height
        cells.cells.append(point)
    return cells

# Function: Get neighbor cells of given cell
# Input: Map Index
# Output: List of Map Indices
def getNeighbors(index):
    tempNeighborIndices = [] #temporary neighnours
    neighborIndices = []     #Final neighbour list
    tempNeighborIndices.append(index+width) #to get the cell beneath current cell
    tempNeighborIndices.append(index-width) #to get the cell above current cell
    tempNeighborIndices.append(index-1)  #to get the cell to the left of current cell
    tempNeighborIndices.append(index+1)  #to get the cell to the right of current cell
    for temp in tempNeighborIndices:
        if (temp not in wallIndices): #checks tempneighbourlist for wall indices and
            neighborIndices.append(temp) #generates a final list of neighbour indices excluding all wall cells
    return neighborIndices

# Function: Calculate Euclidean distance between two Map Indices
# Input: Start Map Index, Goal Map Index
# Output: Float
def getEuclidean(start,goal):
    startX, startY = getXY(start) #calls getXY function to get x and y values of start
    goalX, goalY = getXY(goal)    # and goal
    x_distance = goalX - startX #calculats the x distance
    y_distance = goalY - startY #calculats the x distance
    distance = math.sqrt(pow(x_distance,2)+pow(y_distance,2)) #distance formula
    return distance

# Function: Extract Goal Index for Pathfinding
# Input: Goal Pose Message
# Output: Run Pathfinding
def readGoal(goal):
    global goalX
    global goalY
    global goalPub
    global goalCell
    goalX= goal.pose.position.x #gets the x and y position from goal
    goalY= goal.pose.position.y
    goalIndex = []
    goalCell = getIndex(goalX,goalY) #call getIndex function to get the index of goal cell
    #print("Goal Index: ",goalCell)
    goalIndex.append(getIndex(goalX,goalY))
    cells = generateGridCells(goalIndex,7) #generates goal cell of height 3 i.e highest priority
    goalPub.publish(cells) #publishes goal cell
    #aStar(startCell,goalCell) #calls astar
    #print goal.pose
    pathFinding()

# Function: Extract Start Index for Pathfinding
# Input: Start Pose Message
# Output: Run Pathfinding
def readStart(startPos):
    global startPosX
    global startPosY
    global startPub
    global wallIndices
    global frontierPub
    global startCell
    startPosX = startPos.pose.pose.position.x #gets the x and y position from start position
    startPosY = startPos.pose.pose.position.y
    startIndex = []
    startCell = getIndex(startPosX,startPosY)  #call getIndex function to get the index of goal cell
    #print("Start Index: ",startCell)
    startIndex.append(getIndex(startPosX,startPosY))
    cells = generateGridCells(startIndex,7) #generates start cell of height 3 i.e highest priority
    startPub.publish(cells)
    #howdyNeighbors = getNeighbors(getIndex(startPosX,startPosY))
    #neighborCells = generateGridCells(howdyNeighbors)
    #frontierPub.publish(neighborCells)
    #print startPos.pose.pose
    pathFinding()

# Function: Perform all aspects of pathfinding - !FIXME list these
def pathFinding():
    if (startCell is not None and goalCell is not None):
        pathList = aStar(startCell,goalCell)
        #print(pathList)
        waypointList = waypoints(pathList)
        #print(waypointList)
        wayPath = wayposes(waypointList)
    else:
        print("Select a START and END pose")

# Function: A* (A-Star) path planning algorithm
# Input: Start Index, Goal Index
# Output: List of Indices
def aStar(start,goal):
    # Send blank messages to clear rviz
    gridPathPub.publish(generateGridCells([]))
    wayGridPub.publish(generateGridCells([]))
    blankPath = Path()
    blankPath.header.frame_id = 'map'
    #blankPose = Pose()
    #blankPath.pose = [blankPose]
    wayPathPub.publish(blankPath)


    frontier = PriorityQueue()
    frontier.put((0, start))
    parent = {}
    openSet = []
    frontierList = [start]
    cost = {start:0}
    #currentCost[startIndex] = 0
    # create a new instance of the map
    while not frontier.empty():
        currentTuple = frontier.get()
        currentFScore = currentTuple[0]
        currentIndex = currentTuple[1]
        #print("CURRENT INDEX: ",currentIndex," Current F Score: ",currentFScore)
        openSet.append(currentIndex)
        frontierList.remove(currentIndex)

        if currentIndex == goal:
            break

        neighbors = getNeighbors(currentIndex)
        #print(neighbors)
        for neighbor in neighbors:
            if (neighbor not in cost) or (cost[neighbor] > cost[currentIndex] + 1):
                #print(neighbor)
                gScore = cost[currentIndex]+1
                #print(gScore)
                hScore = getEuclidean(neighbor,goal)
                fScore = gScore + hScore
                #print("currentIndex: ",currentIndex," Neighbor: ",neighbor," F Score",fScore)
                cost.update({neighbor:gScore})
                frontier.put((fScore,neighbor))
                frontierList.append(neighbor)
                parent.update({neighbor:currentIndex})
                #print(frontierList)

        rospy.sleep(.01) # Delay for visual effect
        #frontierList = list(frontier.queue)
        frontierCells = generateGridCells(frontierList)
        currentIndexCells = generateGridCells([currentIndex], 1)
        # Pubs and subs
        openSetPub.publish(generateGridCells(openSet))
        frontierPub.publish(frontierCells)


    #print("A WINNER IS YOU")

    pathList = [goal]
    parentIndex = parent[goal]
    while (parent[parentIndex] != start):
        pathList.append(parentIndex)
        parentIndex = parent[parentIndex]
    pathList.append(parentIndex)

    pathCells = generateGridCells(pathList,2)
    currentIndexPub.publish(generateGridCells([]))
    gridPathPub.publish(pathCells)
    return pathList

# Function: Generates a list of waypoints from a path
# Input: List of Indices
# Output: List of Indices
def waypoints(indexList):
    waypointList = [startCell] # List of Waypoints as Map Indices
    pathList = [] # Path list from start index to goal index
    lastNode = startCell

    for i in range (1, len(indexList)+1):
        pathList.append(indexList[-i])

    for node in pathList:
        if (getXY(node)[0] != getXY(waypointList[-1])[0] and getXY(node)[1] != getXY(waypointList[-1])[1]):
            #print("Current X: ", getXY(node)," WP X: ",getXY(waypointList[-1])[0])
            #print("Current Y: ", getXY(node)," WP Y: ",getXY(waypointList[-1])[1])
            waypointList.append(lastNode)
        lastNode = node

    if goalCell not in waypointList:
        waypointList.append(goalCell)

    waypointGridCells = generateGridCells(waypointList,5)
    currentIndexPub.publish(generateGridCells([]))
    wayGridPub.publish(waypointGridCells)
    #currentIndexPub.publish(currentIndexCells)
    return waypointList

# Function: Generate a Path Message from a list of waypoints
# Input: List of Map Indices
# OutputL Path Message
def wayposes(waypointList):

    wayPoses = []
    for waypoint in range(1,len(waypointList)):
        lastPose = PoseStamped()
        lastPose.header.frame_id = 'map'

        lastXY = getXY(waypointList[waypoint-1])
        #print(lastXY)
        lastxCoord = lastXY[0]
        lastyCoord = lastXY[1]
        #print(lastxCoord)

        currentXY = getXY(waypointList[waypoint])
        currentxCoord = currentXY[0]
        currentyCoord = currentXY[1]

        lastPose.pose.position.x = lastxCoord
        lastPose.pose.position.y = lastyCoord
        lastPose.pose.position.z = 0
        theta = math.atan2(currentyCoord-lastyCoord,currentxCoord-lastxCoord)
        #print(theta*180/math.pi)

        (roll, pitch, yaw) = (0, 0, theta)
        q = quaternion_from_euler(roll, pitch, yaw)
        lastPose.pose.orientation.x = q[0]
        lastPose.pose.orientation.y = q[1]
        lastPose.pose.orientation.z = q[2]
        lastPose.pose.orientation.w = q[3]

        wayPoses.append(lastPose)
        #print(wayPoses)
        wayPath = Path()
        wayPath.header.frame_id = 'map'
        wayPath.poses = wayPoses
        #print(wayPath)
    wayPathPub.publish(wayPath)
    return wayPath




#publishes map to rviz using gridcells type

def publishCells(grid):
    global pub
    global wallIndices
    #print "publishing"

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
                index = i*width+j
                if index not in wallIndices:
                    wallIndices.append(index)
            k = k + 1
    pub.publish(cells)

#Main handler of the project
def run():
    global pub
    global pubPath
    global startPub
    global goalPub
    global wallIndices
    global frontierPub
    global openSetPub
    global currentIndexPub
    global gridPathPub
    global wayGridPub
    global wayPathPub
    global startCell
    global goalCell

    wallIndices = []
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)
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

    startCell = None
    goalCell = None


    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)
    #pathCells = generateGridCells([110,111,112,113])



    while (not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        #frontierPub.publish(generateGridCells(wallIndices))
        #frontierPub.publish(pathCells)
        rospy.sleep(2)
        #print("Complete")



if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass