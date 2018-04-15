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



def progress(current,end):
    prog = current*100/end
    if prog%5 == 0:
        print("Progress: " + str(prog) + "%")


def primesLessThan(number):
    primes = []
    if number <= 3:
        return primes
    else:
        primes = [2]
        i = 3
    while i < number:
        primality = True
        for val in primes:
            if val > math.sqrt(i):
                break
            if i%val == 0:
                primality = False
                break
        if primality == True:
            primes.append(i)
        i += 2
        progress(i,number)
    return primes


def getListOfFactors(number):
    factors = []
    cur = number
    checkVals = primesLessThan(number)
    for val in checkVals:
        if cur == 1:
            break
        while( cur%val == 0 ):
            cur = cur/val
            factors.append(val)
    return factors



#print(len(primesLessThan(1000)))


#print(len(primesLessThan(10000000)))

print(getListOfFactors(384))
