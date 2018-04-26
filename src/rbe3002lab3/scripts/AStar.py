#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

# What is the current position?

# What is the goal position?

# While we are not there yet

    # Are we there yet?

    # What are the neighboring nodes?

    # Add the neighboring nodes to the frontier
        # Priority List based on G + H = F Score

        # The G score is the cost to get to that node

        # The H score is the estimated cost from that node to the end goal

    # Select node with lowest F score - AKA first in priority list




# Open list
# A priority queue of nodes to check - sorted by F score

# Closed list
# A dictionary of visited nodes and their parents
# Nodes are stored with their best parent

# G score

# H score

# F Score
