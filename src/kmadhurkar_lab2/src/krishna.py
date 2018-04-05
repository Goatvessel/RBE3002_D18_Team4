#!/usr/bin/env python
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
        rospy.sleep(0.2)
        self.yaw = 0
        self.pitch = 0
        self.role = 0


    def navToPose(self,goal):
        """
            This is a callback function. It should exract data from goal, drive in a striaght line to reach the goal and
            then spin to match the goal orientation.
        """

        self._odom_list.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(1.0))
        rospy.sleep(0.1)
        #because it extracts information wayy before it actually located the goal
        transGoal = self._odom_list.transformPose('/base_link', goal)
        # transform the nav goal from the global coordinate system to the robot's coordinate system

        xg = transGoal.pose.position.x
        yg = transGoal.pose.position.y

        qg = [transGoal.pose.orientation.x,
             transGoal.pose.orientation.y,
             transGoal.pose.orientation.z,
             transGoal.pose.orientation.w]

        (rollg, pitchg, yawg) = euler_from_quaternion(qg)

        print "yawg: ", yawg
        #xs = self._current.position.x is in global cordinate
        xs = 0
        ys = 0
        #ys = self._current.position.y

        print "xs: ",xs
        print "ys: ",ys

        print "xg: ",xg
        print "yg: ",yg
        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)

        #angle  = math.atan((yg-ys)/(xg-xs))
        angle = math.atan2(yg-ys, xg-xs)
        angle = angle * 180/math.pi
        self.rotate(angle)
        print (angle)
        distance_goal =  math.sqrt(((xg - xs)**2) +((yg - ys)**2 ))
        self.driveStraight(0.2, distance_goal)

        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw2) = euler_from_quaternion(q)
        print "yaw2: ", yaw2
        yaw_degree = (yaw2-yawg)*(180/math.pi)
        print "yaw_degree: ", yaw_degree
        self.rotate(yaw_degree)





    def executeTrajectory(self):
      """
        See lab manual for the dance the robot has to excute
      """
      self.driveStraight(0.1,0.6)
      self.rotate(90)
      self.driveStraight(0.1,0.45)
      self.rotate(135)

    def driveStraight(self, speed,distance):
        """
            This method should populate a
             message type and publish it to nav_goal in order to move the robot
        """
        #self.spinWheels(speed, speed, distance/speed)
        print ("Begining to drive")
        origin = copy.deepcopy(self._current) #use this


        drive_msg = Twist()
        drive_msg.linear.x = speed
        drive_msg.angular.z =0

        initialx = origin.position.x
        initialy = origin.position.y
        reached = False
        while(not reached):
            currentx = self._current.position.x
            currenty = self._current.position.y
            #Use distance formula to find the distance
            current_distance = math.sqrt(((currentx - initialx)**2) +((currenty - initialy)**2 ))
            if(current_distance >= distance):
                reached = True
                drive_msg.linear.x = 0
                self._vel_pub.publish(drive_msg)
                print "reached"
                print "distance travelled",current_distance

            elif(current_distance < 0.2*distance):
                drive_msg.linear.x = 2 * speed
                #print "my speed2",drive_msg.linear.x
            elif(current_distance >= 0.8*distance):
                drive_msg.linear.x = 0.5 * speed
            else:
                drive_msg.linear.x = 2 * speed
            self._vel_pub.publish(drive_msg)



    def spinWheels(self, v_left, v_right, time):
        """
           This method should use differential drive kinematics to compute V and omega (linear x = V, angular z = omega).
           It should then create a Twist message type, and publish it to cmd_vel_mux in order to move the robot
        """
        driveStartTime = rospy.Time.now().secs
    	L = 0.16 #in centself.imeters # based on wheel track from https://yujinrobot.github.io/kobuki/doxygen/enAppendixKobukiParameters.html
        if v_right != v_left:
            R = (L/2)*(v_left + v_right)/(v_right - v_left)
            omega = (v_right - v_left)/L
            V = (v_left + v_right)/2
        else:
            omega = 0.0
            V = v_left

    	move_msg =Twist()
        move_msg.linear.x = V
        move_msg.angular.z = omega
        print "Publishing message : ", move_msg
        self._vel_pub.publish(move_msg)

    	stop_msg =Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        print "Publishing message : ", stop_msg
        self._vel_pub.publish(stop_msg)


        while (driveStartTime + time > rospy.Time.now().secs):
            self._vel_pub.publish(move_msg)
        self._vel_pub.publish(stop_msg)




    def rotate(self, rotation):
        """
            This method should populate a ??? message type and publish it to ??? in order to spin the robot
        """
        "origin = copy.deepcopy(self._current)"

        angle = rotation*math.pi/180
        print "angle :", angle

        q = [self._current.orientation.x,
             self._current.orientation.y,
             self._current.orientation.z,
             self._current.orientation.w]

        (roll, pitch, yaw) = euler_from_quaternion(q)

        finalAngle = angle + yaw

        if (finalAngle > math.pi):
            finalAngle += -2*math.pi
        elif (finalAngle < -math.pi):
            finalAngle += 2*math.pi


        run_msg = Twist()
        run_msg.linear.x = 0.0
        if (angle > 0):
            run_msg.angular.z = 0.4
        elif (angle < 0):
            run_msg.angular.z = -0.4

        print finalAngle
        print yaw

        while (abs(finalAngle - yaw) > (math.pi/90)):
            q = [self._current.orientation.x,
                 self._current.orientation.y,
                 self._current.orientation.z,
                 self._current.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(q)
            self._vel_pub.publish(run_msg)

        print finalAngle
        print yaw
        run_msg.angular.z = 0.0
        self._vel_pub.publish(run_msg)




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

    	# convert the quaternion to roll pitch yaw
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
# pub = rospy.Publisher(('cmd_vel_mux/input/teleop',Twist, queue_size=1)
# initialx = origin.position.x
# initialy = origin.position.y
