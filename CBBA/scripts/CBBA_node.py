#!/usr/bin/env python
#TODO class modulization is required

import rospy
from geometry_msgs.msg import PoseStamped
from keyboard.msg import Key
from visualization_msgs.msg import *
import roslib
import tf
import geometry_msgs.msg

'''initial flags'''

isReceiving = False
isFinishing = False
isCalc = False


'''marker scale'''


search_pos=[]
listener=tf.TransformListener()
tb0_pos=[0,0]
tb1_pos=[0,0]
tb2_pos=[0,0]

# rviz marker
search_pos_marker_pub=rospy.Publisher('/search_position_markers',MarkerArray,queue_size=1)

'''To use this call back function '''
'''In Rviz -> tool properties -> 2D Nav goal topic name : /search_position '''

'''call back functions '''
def click_pnts_callback(data):
    global isReceiving,isCalc,isFinishing
    print(isReceiving)
    if isReceiving:
        search_pos.append(data.pose.position)
        rospy.loginfo("received point : [%.4f, %.4f]",data.pose.position.x,
                      data.pose.position.y)
    # rospy.loginfo("saved pnts until now:")
    # for pos in search_pos:
    #     rospy.loginfo("[%.4f, %.4f, %.4f]",pos.x,pos.y,pos.z)

def key_sub(data):
    data=data.code
    global isReceiving, isCalc, isFinishing
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

    br=tf.TransformBroadcaster()

    rate=rospy.Rate(30)
    # main loop
    while not rospy.is_shutdown():

        # marker for search positions published
        pos_idx=0
        marker_array=MarkerArray()
        for pos in search_pos:
            # visualization marker
            marker=Marker()

            marker.type=Marker.CUBE
            marker.action=Marker.ADD
            marker.header.frame_id="map"
            marker.header.stamp=rospy.Time.now()
            marker.id=pos_idx
            marker.lifetime=rospy.Duration()
            marker.ns='search_positions'
            scale=1
            marker.scale.x=scale ; marker.scale.y=scale; marker.scale.z=scale
            marker.pose.orientation.w=1.0
            marker.color.r=1; marker.color.a=1
            marker.pose.position=pos

            marker_array.markers.append(marker)
            # transform
            pos_name='sp'+str(pos_idx)
            br.sendTransform((pos.x,pos.y,pos.z),(0,0,0,1),rospy.Time.now(),pos_name,'map')
            pos_idx+=1


        search_pos_marker_pub.publish(marker_array)

        '''
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
        '''
        if isFinishing and isCalc:
            isCalc = False # already calculating


        rate.sleep()








