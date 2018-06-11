#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from keyboard.msg import Key
import roslib
import tf
import geometry_msgs.msg

'''initial flags'''
isReceiving = False
isFinishing = False
isCalc = False


search_pos=[]
listener=tf.TransformListener()
tb0_pos=[0,0]
tb1_pos=[0,0]
tb2_pos=[0,0]


'''To use this call back function '''
'''In Rviz -> tool properties -> 2D Nav goal topic name : /search_position '''

'''call back functions '''
def click_pnts_callback(data):
    if isReceiving:
        search_pos.append(data.pose.position)
        rospy.loginfo("received point : [%.4f, %.4f]",data.pose.position.x,
                      data.pose.position.y)
    # rospy.loginfo("saved pnts until now:")
    # for pos in search_pos:
    #     rospy.loginfo("[%.4f, %.4f, %.4f]",pos.x,pos.y,pos.z)

def key_sub(data):
    data=data.code
    if data == 119: # renew
        '''reset all the flags'''
        isReceiving = False
        isFinishing = False
        isCalc = False
        rospy.loginfo("renewed")
    elif data == 97: # start receiving points
        isReceiving = True
        rospy.loginfo("start receiving points")
    elif data == 115: # finish receiving points
        isFinishing = True
        rospy.loginfo("finish receiving points")
    elif data == 100: # calculate CBBA algorithm
        isCalc = True
        rospy.loginfo("finish receiving points")
    else :
        rospy.loginfo("wrong key input")


'''reg function'''

def point_receiver():
    rospy.Subscriber("/search_position",PoseStamped,click_pnts_callback)

def user_input():
    rospy.Subscriber("/keyboard/keyup",Key,key_sub)



if __name__=='__main__':

    # initialization
    rospy.init_node('CBBA_node')
    rospy.loginfo("CBBA node started")
    rospy.loginfo("\na: start receiving pnts \ns: finish receiving pnts\n"
                  "d: distribute tasks    \nw: flush ")
    # reg point receiver
    point_receiver()
    # reg keyboard input
    user_input()

    rate=rospy.Rate(30)
    # main loop
    while not rospy.is_shutdown():

        (trans0, rot0) = listener.lookupTransform('tb3_0/odom', 'map', rospy.Time(0))
        tb0_pos[0] = trans0[0]
        tb0_pos[1] = trans0[1]
        # rospy.loginfo('tb0 position: [%.4f, %.4f]', tb0_pos[0], tb0_pos[1])
        (trans1, rot1) = listener.lookupTransform('tb3_1/odom', 'map', rospy.Time(0))
        tb1_pos[0] = trans1[0]
        tb1_pos[1] = trans1[1]
        # rospy.loginfo('tb1 position: [%.4f, %.4f]', tb1_pos[0], tb1_pos[1])

        (trans2, rot2) = listener.lookupTransform('tb3_2/odom', 'map', rospy.Time(0))
        tb2_pos[0] = trans2[0]
        tb2_pos[1] = trans2[1]
        # rospy.loginfo('tb2 position: [%.4f, %.4f]', tb2_pos[0], tb2_pos[1])
        if isFinishing and isCalc:
            isCalc = False # already calculating

        rate.sleep()








