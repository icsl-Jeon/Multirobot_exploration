#!/usr/bin/env python
#TODO class modulization is required

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import *
from kiro_gui_msgs.msg import PositionArray
from snu_task_alloc.srv import AllocateTask
from CBBA_Main import *
from path_node import *
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped
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
agent_pos_marker_pub = rospy.Publisher('/agent_position_markers', MarkerArray, queue_size=1)


# A star instance
inflate_rate = 0.3
a_star=Astar(inflate_rate)



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
path_pubs=[] # array of path publisher


'''CBBA computation and path generation '''

def allocate_path(astar):

    global isTask,isAgents,isSolve, allocated_paths
    global search_pos,agent_pos
    allocated_paths=[]
    if isTask and isAgents:
        rospy.loginfo("num of search pos: %d  / agent pos %d",len(search_pos),len(agent_pos))
        cbba=CBBA_solve(search_pos,agent_pos,astar)
        # Path extraction & parsing
        cbba_path=cbba.path

        # iterate through the agents
        for i in range(cbba.Nu):
            # path of this agent
            path=Path()
            path.header.frame_id="map"

            # Firstly, we put the position of agent
            # TODO how about making a function for this procedure?
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id="map"
            pose_stamp.pose.position.x=cbba.agent_list[i].x
            pose_stamp.pose.position.y = cbba.agent_list[i].y
            pose_stamp.pose.position.z = cbba.agent_list[i].z
            path.poses.append(pose_stamp)


            # Secondly, we put the task position of the path
            for j in range(cbba.Nt):
                if not cbba_path[i,j] == -1:
                    pose_stamp=PoseStamped()
                    pose_stamp.header.frame_id = "map"
                    pose_stamp.pose.position.x = cbba.task_list[cbba_path[i,j]].x
                    pose_stamp.pose.position.y = cbba.task_list[cbba_path[i,j]].y
                    pose_stamp.pose.position.z = cbba.task_list[cbba_path[i,j]].z
                    path.poses.append(pose_stamp)



            # Thirdly, this path is just high level waypoitns. we break them down into smaller one

            original_path_length=len(path.poses)
            dense_path = Path()
            dense_path.header.frame_id="map"

            interval_length=0.8 # interval length between dense waypoints

            for i in range(original_path_length-1):
                sub_start = path.poses[i]
                sub_goal = path.poses[i+1]
                outarg=astar.solve_path(sub_start.pose.position,sub_goal.pose.position)
                sub_path = outarg[1]
                length_path=len(sub_path.poses)
                poses=sub_path.poses
                index_step=int((interval_length/astar.res))
                extract_idx=np.ceil(np.arange(0,length_path,index_step)).astype('int')
                poses=[poses[i] for i in extract_idx]
                dense_path.poses.extend(poses)


            allocated_paths.append(dense_path)

    else:
        rospy.logwarn('task or agents are not provided')

'''construct array of path publisher'''

for i in range(Nu):
    topic_name="allocation_path"+str(i)
    path_pub=rospy.Publisher(topic_name,Path,queue_size=1)
    path_pubs.append(path_pub)


'''publish the path'''
def paths_pub():
    global allocated_paths,path_pub
    # publish each path for agent
    if len(allocated_paths) == Nu:
        for i in range(Nu):
            path=allocated_paths[i]
            path_pubs[i].publish(path)


''' calculation server registration'''

def service_callback(req):
    global a_star
    if req.calculate:
        rospy.loginfo('calculating CBBA')
        allocate_path(a_star)

    else:
        rospy.loginfo('calculation was not requested')

    return True



def CBBA_server():
    s = rospy.Service('CBBA_allocate', AllocateTask,service_callback)


######################################
# task receiving / marker publishing #
######################################

'''search positions callback'''
def sps_callback(data):
    global search_pos,isTask
    search_pos=[]
    rospy.loginfo('received search position array')
    isTask=True
    for pos in data.positions:
        search_pos.append(pos.pose.position)


def task_receiver():
    rospy.Subscriber("/search_position_array",PositionArray,sps_callback)

''' marker pusblish / tf '''

def task_marker_publish():
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

    global tb0_pos,tb1_pos,tb2_pos
    listener.waitForTransform('tb3_0/odom', 'map', rospy.Time(0),rospy.Duration(10))

    (trans0, rot0) = listener.lookupTransform('tb3_0/odom', 'map', rospy.Time(0))
    tb0_pos[0] = trans0[0]
    tb0_pos[1] = trans0[1]
    # rospy.loginfo('tb0 position: [%.4f, %.4f]', tb0_pos[0], tb0_pos[1])

    listener.waitForTransform('tb3_1/odom', 'map', rospy.Time(0),rospy.Duration(10))

    (trans1, rot1) = listener.lookupTransform('tb3_1/odom', 'map', rospy.Time(0))
    tb1_pos[0] = trans1[0]
    tb1_pos[1] = trans1[1]
    # rospy.loginfo('tb1 position: [%.4f, %.4f]', tb1_pos[0], tb1_pos[1])

    listener.waitForTransform('tb3_2/odom', 'map', rospy.Time(0),rospy.Duration(10))
    (trans2, rot2) = listener.lookupTransform('tb3_2/odom', 'map', rospy.Time(0))
    tb2_pos[0] = trans2[0]
    tb2_pos[1] = trans2[1]
    # rospy.loginfo('tb2 position: [%.4f, %.4f]', tb2_pos[0], tb2_pos[1])

''' publish marker for the agents '''


def agent_marker_publish():

    global tb0_pos,tb1_pos,tb2_pos  # marker for search positions published
    pos_idx = 0
    marker_array = MarkerArray()
    tb_poses=[tb0_pos , tb1_pos, tb2_pos] # position array

    for i in range(Nu):
        # visualization marker
        marker = Marker()
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = pos_idx
        marker.lifetime = rospy.Duration()
        marker.ns = 'agent_position'
        scale = 0.5
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.pose.orientation.w = 1.0
        marker.color.r = 1
        marker.color.g = 0.1
        marker.color.b = 1
        marker.color.a = 0.5
        marker.pose.position.x = tb_poses[i][0]
        marker.pose.position.y = tb_poses[i][1]

        marker_array.markers.append(marker)
        # transform
        pos_name = 'agent' + str(pos_idx)
        br.sendTransform((tb_poses[i][0], tb_poses[i][1], 0.0), (0, 0, 0, 1), rospy.Time.now(), pos_name, 'map')
        pos_idx += 1
    search_pos_marker_pub.publish(marker_array)




if __name__=='__main__':

    # initialization
    rospy.init_node('CBBA_node')
    rospy.loginfo("CBBA node started")
    task_receiver()
    CBBA_server()
    br=tf.TransformBroadcaster()
    rate=rospy.Rate(30)
    # main loop
    while not rospy.is_shutdown():
        '''
        # robot state receiving
        state_receiver()
        '''

        a_star.inflated_map_publish()
        task_marker_publish()
        agent_marker_publish()

        # send out path msg if any
        paths_pub()
        rate.sleep()

