#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import String  # String message
from std_msgs.msg import Int8

if __name__ == '__main__':
    rospy.init_node('keypress')
    rospy.loginfo("keypress node started: enter keyboard in this window\n")
    pub = rospy.Publisher('/key', Int8, queue_size=1)  # "key" is the publisher name
    rate = rospy.Rate(100)  # try removing this line ans see what happens

    rospy.loginfo("\na: start receiving pnts \ns: finish receiving pnts\n"
                  "d: distribute tasks    \nw: flush ")
    while not rospy.is_shutdown():
        k = ord(getch.getch())
        rospy.loginfo(str(k))
        pub.publish(k)
        rate.sleep()