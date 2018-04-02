#!/usr/bin/env python
class Robot:

    def __init__(self):
        pass

    if __name__ == '__main__':
        print("-- Begin Initial Operation --")
        rospy.init_node('drive_base')
        turtle = Robot()

        print("-- End Initial Operation --")
        while not rospy.is_shutdown():
            pass
