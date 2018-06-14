#!/usr/bin/env python
#TODO class modulization is required

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import *
from kiro_gui_msgs.msg import PositionArray
from CBBA.srv import *
from CBBA_Main import *
import numpy as np
import tf
from geometry_msgs.msg import Pose


'''flags'''
isTask=False # have received tasks?
isAgents=True # have received agents?
isSolve=False  # have we sovled CBBA

search_pos=[]
listener=tf.TransformListener()

''' Several global variables '''

# rviz marker
search_pos_marker_pub = rospy.Publisher('/search_position_markers', MarkerArray, queue_size=1)


##################
# Agent location #
##################
Nu=3
tb0_pos=[-7,-1]
tb1_pos=[7,-1]
tb2_pos=[0.5,3.0]

agent_pos=[]

pose = Pose()
pose.position.x=tb0_pos[0]; pose.position.y=tb0_pos[1]
agent_pos.append(pose.position)

pose = Pose()
pose.position.x=tb1_pos[0]; pose.position.y=tb1_pos[1]
agent_pos.append(pose.position)

pose = Pose()
pose.position.x=tb2_pos[0]; pose.position.y=tb2_pos[1]
agent_pos.append(pose.position)

##############################
# Path of task for each unit #
##############################

allocated_paths=[] # array of Navmsg.Path


'''CBBA computation '''

def allocate_path():
    global isTask,isAgents,isSolve, allocated_paths
    allocated_paths=[]
    if isTask and isAgents:
        cbba=CBBA_solve(search_pos,agent_pos)
        # Path extraction & parsing
        cbba_path=cbba.path

        # iterate through the agents
        for i in range(cbba.Nu):
            # path of this agent
            path=Path()

            # Firstly, we put the position of agent
            # TODO how about making a function for this procedure?
            pose = Pose()
            pose.position.x=cbba.agent_list[i].x
            pose.position.y = cbba.agent_list[i].y
            pose.position.z = cbba.agent_list[i].z
            path.poses.append(pose)


            # Secondly, we put the task position of the path
            for j in range(cbba.Nt):
                if not cbba_path[i,j] == -1:
                    pose=Pose()
                    pose.position.x = cbba.task_list[cbba_path[i,j]].x
                    pose.position.y = cbba.task_list[cbba_path[i,j]].y
                    pose.position.z = cbba.task_list[cbba_path[i,j]].z
                    path.poses.append(pose)

            allocated_paths.append(path)

        return allocated_paths
    else:
        rospy.logwarn('task or agents are not provided')
        return allocated_paths



'''search positions callback'''
def sps_callback(data):
    global search_pos
    search_pos=[]
    rospy.loginfo('received search position array')
    isTask=True
    for pos in data.positions:
        search_pos.append(pos.pose.position)


def task_receiver():
    rospy.Subscriber("/search_position_array",PositionArray,sps_callback)

''' marker pusblish / tf '''

def marker_publish():
    global search_pos
     # marker for search positions published
    pos_idx = 0
    marker_array = MarkerArray()
    for pos in search_pos:
        # visualization marker
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = pos_idx
        marker.lifetime = rospy.Duration()
        marker.ns = 'search_positions'
        scale = 0.2
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.pose.orientation.w = 1.0
        marker.color.r = 1
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1
        marker.pose.position = pos

        marker_array.markers.append(marker)
        # transform
        pos_name = 'sp' + str(pos_idx)
        br.sendTransform((pos.x, pos.y, pos.z), (0, 0, 0, 1), rospy.Time.now(), pos_name, 'map')
        pos_idx += 1
    search_pos_marker_pub.publish(marker_array)


''' extract the tf for state of each robot '''
# TODO still handling the exception error ....
def state_receiver():
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



if __name__=='__main__':

    # initialization
    rospy.init_node('CBBA_node')
    rospy.loginfo("CBBA node started")
    task_receiver()
    br=tf.TransformBroadcaster()
    rate=rospy.Rate(30)
    # main loop
    while not rospy.is_shutdown():
        # search position marker
        marker_publish()
        # CBBA solve ?

        rate.sleep()

