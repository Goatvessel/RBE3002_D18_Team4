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

def readGoal(goal):
    global goalX
    global goalY
    global goalPub
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y

    #finds the start within the cell boundaries
    actX = goalX/1

    goalCell = GridCells()
    goalCell.header.frame_id = 'map'
    goalCell.cell_width=resolution
    goalCell.cell_height=resolution
    point = Point()
    point.x = goalX
    point.y = goalY
    # point.x = 11.15
    # point.y = 10.65
    point.z = 0
    goalCell.cells.append(point)
    #print(goalCell)
    goalPub.publish(goalCell)


    print goal.pose

    # Start Astar


def readStart(startPos):

    global startPosX
    global startPosY
    global startPub
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y

    startCell = GridCells()
    startCell.header.frame_id = 'map'
    startCell.cell_width=resolution
    startCell.cell_height=resolution
    point = Point()
    point.x = round(startPos.pose.pose.position.x,0)
    point.y = round(startPos.pose.pose.position.y,0)
    # point.x = 11.15
    # point.y = 10.65
    point.z = 0
    startCell.cells.append(point)
    print(startCell)
    startPub.publish(startCell)
    #publishCells(startCell)
    print startPos.pose.pose

def aStar(start,goal):
    pass
    # create a new instance of the map
    # sudo code
    def a_star_search(graph, start, goal):
        # give start and nav 
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next) #change graph stuff to map
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    return came_from, cost_so_far
    #...

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
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    goal_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)



    while (not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)
        print("Complete")



if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
