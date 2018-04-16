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

# ------------------------------ WIP Functions ------------------------------ #

def checkOperations(width, height):

    while (true):
        blockWidth = width
        widthLength = lcd(blowck)
        
        cellWidth = width//getLCM(width)
        cellHeight = height//get(height)


def getLCM(integer):






# ------------------------------- Main Functions ---------------------------- #

# Main handler of the project
def run():

    print("- - Begin Operation - -")

    while (not rospy.is_shutdown()):
        pass
    print("")
    print("- - End Operation - -")

# Standalone operation
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
