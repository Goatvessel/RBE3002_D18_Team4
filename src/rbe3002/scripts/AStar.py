#!/usr/bin/env python
import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String


# Open list
# A priority queue of nodes to check - sorted by F score

# Closed list
# A dictionary of visited nodes and their parents
# Nodes are stored with their best parent

# G score

# H score

# F Score
