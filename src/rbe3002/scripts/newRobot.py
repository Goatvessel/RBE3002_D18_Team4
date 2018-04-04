#!/usr/bin/env python
import rospy, tf, copy, math, time
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """

        self._current = Pose() # Position and orientation of the robot
        self._odom_list = tf.TransformListener() # Subscribe to transform messages
        rospy.Timer(rospy.Duration(.1), self.timerCallback) # Setup callback - not hard real-time
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1) # Publisher Twist messages to cmd_vel topic
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # Subscribe to navigation goal messages


    def _get_twist(self, linear, angular):
		# Construct Twist message for differential drive robot based on linear and angular velocity
		twist = Twist()
		twist.linear.x = linear
		twist.angular.z = angular
		return twist

    def _convert_pose(self,myPose):
        # Converts a Pose message into a list of [X-Position, Y-Position, Yaw]
        # Where Yaw is the rotation about the Z-Axis
        q = [myPose.orientation.x,
			myPose.orientation.y,
			myPose.orientation.z,
			myPose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(q)
        xPos = myPose.position.x
        yPos = myPose.position.y
        return [xPos, yPos, yaw]

    def navToPose(self,goal):
		# Callback function when a navigation goal is defined
		# Extract data from Goal, rotate to Goal position, drive straight to Goal position, rotate to Goal orientation

        # Goal Pose
        self._odom_list.waitForTransform('odom', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
        # transform the nav goal from the global coordinate system to the robot's coordinate system
        GoalPoseStamped = self._odom_list.transformPose('base_link', goal)
        GoalPose = self._convert_pose(GoalPoseStamped.pose)
        xGoal = GoalPose[0] # Desired X
        yGoal = GoalPose[1] # Desired Y
        yawGoal = GoalPose[2] # Desired yaw - angle about z axis

        # Current Pose
        origin = copy.deepcopy(self._current)
        initialPose = self._convert_pose(origin)
        xInitial = initialPose[0] # Current X
        yInitial = initialPose[1] # CUrrent Y
        yawInitial = initialPose[2] # Current Yaw

        # Distance to the Goal from the Start
        straightline = math.sqrt(math.pow(xGoal,2) + math.pow(yGoal,2))
        # Angle between the Initial orientation and the Goal orientation
        initialYaw = math.atan2(yGoal,xGoal)

        # Rotate to face the Goal position
        self.rotate(initialYaw)
        # Drive straight to the Goal position
        self.driveStraight(.2,straightline)
        # Rotate to face the Goal orientation
        self.rotate(yawGoal - initialYaw)

    def executeTrajectory(self,_DEBUG_=False):
		# A set trajectory to follow
		# Drive 0.6 meters, Rotate 90 degrees right, drive 0.45 meters, rotate 135 degrees left

        self.driveStraight(.05,.6,_DEBUG_)
        self.rotate(-90,True,_DEBUG_)
        self.driveStraight(.1,.45,_DEBUG_)
        self.rotate(135,True,_DEBUG_)


    def driveStraight(self, speed, distance, _DEBUG_=False):
		# Drive both wheels at the same speed for a set distance
        self.spinWheels(0,0,.1) # Initialize odometry by spinning wheels with no velocity - !FIXME There must be a better way
        #_DEBUG_ = True

        interval = .01 # [seconds] Check current position against goal position after this time interval
        v_left = speed  # [m/s]
        v_right = speed # [m/s]
        distance = distance # [m]

        origin = copy.deepcopy(self._current)
        currentPose = self._convert_pose(origin)
        initial_x = current_x = currentPose[0]
        initial_y = current_y = currentPose[1]
        current_yaw = currentPose[2]

        goal_location_x = initial_x + distance*math.cos(current_yaw)
        goal_location_y = initial_y + distance*math.sin(current_yaw)

        distanceTraveled = 0
        distanceToGo = distance - distanceTraveled

        while distanceToGo > 0:
            self.spinWheels(v_left, v_right, interval)
            currentPose = self._convert_pose(copy.deepcopy(self._current))
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
        initialPose = self._convert_pose(copy.deepcopy(self._current))
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
            currentYaw = self._convert_pose(copy.deepcopy(self._current))[2]
            if _DEBUG_:
                print ("Current Angle: ",currentYaw," Distance To Go: ",abs(goalYaw-currentYaw))


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
		(roll, pitch, yaw) = euler_from_quaternion(q)

    # helper functions
    def planTraj(self, b, t):
		pass

		"""
		Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
		"""

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


if __name__ == '__main__':
    _TEST_ = "TRAJECTORY"
    #_TEST_ = "DRIVESTRAIGHT"

    print("-- Begin Operation --")
    rospy.init_node('drive_base')
    turtle = Robot()

    if _TEST_ == "DRIVESTRAIGHT":
        # Drive straight and print debug statements
            # Speed: 1 m/s
            # Distance: 1.5 m
        turtle.driveStraight(1,1.5,True)
    elif _TEST_ == "ROTATE":
        # Rotate and print debug statements
            # Direction: Counter-Clockwise
            # Angle: 45 degrees
        turtle.rotate(45,True,True)
    elif _TEST_ == "TRAJECTORY":
        # Execute set trajectory and print debug statements
        turtle.executeTrajectory(True)

    print("-- End Operation --")

    while not rospy.is_shutdown():
	pass # It's a bit loopy in here!
