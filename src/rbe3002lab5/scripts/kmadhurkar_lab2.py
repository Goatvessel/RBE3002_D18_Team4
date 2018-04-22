#!/usr/bin/env python
# Krishna Madhurkar RBE 3002 Lab 2 
# 30 march 2018


import rospy, tf, copy, math

from geometry_msgs.msg import Twist, Pose, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import String

class Robot:

    def __init__(self):

        """
            This constructor sets up class variables and pubs/subs
        """

        self._current = Pose() # initlize correctly
        """
        self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('/odom','/base_link', rospy.Time(0))
        """
        self._current.position.x = 0
        self._current.position.y = 0
        self._current.orientation.x = 0
        self._current.orientation.y = 0
        self._current.orientation.z = 0
        self._current.orientation.w = 0
        self._odom_list = tf.TransformListener()
        rospy.Timer(rospy.Duration(.1), self.timerCallback)
        self._vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navToPose, queue_size=1) # handle nav goal events
        # always listens for navGoal events and calls the function navGoal when a message of the type PoseStamped is received
        rospy.sleep(0.2)
        self.yaw = 0
        self.pitch = 0
        self.role = 0


    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """
        print ("Starting navigation")
        self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        rospy.sleep(0.1)
        #because it extracts information wayy before  goal is actually located, failing to do which gives errrors
        transGoal = self._odom_list.transformPose('/base_link', goal)
        # transform the nav goal from the global coordinate system to the robot's coordinate system

        #extracting the x and y from goal pose
        goalx = transGoal.pose.position.x 
        goaly = transGoal.pose.position.y
        #extracting the yaw from goal pose
        goalq = [transGoal.pose.orientation.x,
             transGoal.pose.orientation.y,
             transGoal.pose.orientation.z,
             transGoal.pose.orientation.w]

        (rollg, pitchg, yawg) = euler_from_quaternion(goalq)

        print "yawg: ", yawg
        #startx = self._current.position.x is in global cordinate is in world frame
        #starty = self._current.position.y is in world frame
        startx = 0  # is in robots frame 
        starty = 0  # is in robot frame
        

        #testing code by printing extracted goal location and initial starting location
        print "startx: ",startx
        print "starty: ",starty

        print "goalx: ",goalx
        print "goaly: ",goaly

        # to get the initial angle, (yaw)
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)



        #angle  = math.atan((goaly-starty)/(goalx-startx))
        angle = math.atan2(goaly-starty, goalx-startx)
        #convert into degrees as rotation() takes in degrees
        angle = angle * 180/math.pi
        # call rotate function to rotate towards desired goal path direction
        self.rotate(angle)
        #print angle to test code
        print (angle)
        # Calculating distance to goal using distance formula
        distance_goal =  math.sqrt(((goalx - startx)**2) +((goaly - starty)**2 ))
        #call drivestraight function to drive to goal location
        self.driveStraight(0.2, distance_goal)

        #Check the angle at goal position 
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw2) = euler_from_quaternion(q)
        # test angle
        print "yaw2: ", yaw2

        # calculate the required angle to equal desired goal angle using current angle at goal after driving forward
        yaw_degree = (yaw2-yawg)*(180/math.pi)
        print "yaw_degree: ", yaw_degree

        #call rotate function 
        self.rotate(yaw_degree)

        Print ("navigation complete")




    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute
      """
      #drive straight to 60cm , turn right to 90 degrees , drive straight 45 cm and finally turn left 135 degree
      self.driveStraight(0.1,0.6)
      self.rotate(90)
      self.driveStraight(0.1,0.45)
      self.rotate(-135)

    def driveStraight(self, speed,distance):
        """
            This method should populate a
             message type and publish it to nav_goal in order to move the robot
        """
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
            elif(current_distance < 0.8*distance):
                drive_message.linear.x = 2 * speed
                #print "my speed2",drive_message.linear.x

            #to slow down when destination is close
            elif(current_distance >= 0.8*distance):
                drive_message.linear.x = 0.5 * speed

            #to often reach faster to any goal
            else:
                drive_message.linear.x = 2 * speed
            self._vel_pub.publish(drive_message)



    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a Twist message type, and publish it to cmd_vel_mux in order to move the robot
        """

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




    def rotate(self, rotation):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """
        "origin = copy.deepcopy(self._current)"


        print ("Starting to rotate")

        #convert incoming degrees angle to radians
        radian_angle = rotation*math.pi/180
        print "angle :", angle

        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)

        #calculate desired angle
        desired_angle = radian_angle + yaw

        if (desired_angle > math.pi):
            desired_angle += -2*math.pi
        elif (desired_angle < -math.pi):
            desired_angle += 2*math.pi

        #create a new message of the type twist()
        rotate_message = Twist()

        #fix direction turning 
        rotate_message.linear.x = 0.0
        if (radian_angle > 0):
            rotate_message.angular.z = 0.4
        elif (radian_angle < 0):
            rotate_message.angular.z = -0.4

        #testing
        print desired_angle
        print yaw

        #To get to desired angle from current angle
        while (abs(desired_angle - yaw) > (math.pi/90)):
            q = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(q)
            self._vel_pub.publish(rotate_message)

        print desired_angle
        print yaw
        rotate_message.angular.z = 0.0
        self._vel_pub.publish(rotate_message)




    def timerCallback(self,evprent):
        """
            This is a callback that runs every 0.1s.
            Updates this instance of Robot's internal position variable (self._current)
        """
	# wait for and get the transform between two frames
        self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        (position, orientation) = self._odom_list.lookupTransform('/odom','/base_link', rospy.Time(0))
	# save the current position and orientation
        self._current.position.x = position[0]
        self._current.position.y = position[1]
        self._current.orientation.x = orientation[0]
        self._current.orientation.y = orientation[1]
        self._current.orientation.z = orientation[2]
        self._current.orientation.w = orientation[3]
        # create a quaternion
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]



        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(q)

    	
def planTraj(self, b, t):
    pass


# helper functions
def planTraj(self, b, t):
    """
        Bonus Question:  compute the coefs for a cubic polynomial (hint:  you did this in 3001)
    """


if __name__ == '__main__':

    rospy.sleep(2)
    rospy.init_node('drive_base')
    turtle = Robot()

#test self function calls here
    #turtle.spinWheels( 0.2, 0.3, 5)
    #turtle.driveStraight(0.5, 5)
    #turtle.rotate(90)
    #turtle.executeTrajectory()


    while  not rospy.is_shutdown():
        pass
\
