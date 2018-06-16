#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from snu_task_alloc.srv import PathCompute

from scipy import ndimage

import numpy as np
from A_star import *

'''This node calculates shortest path in 2D occupancy grid'''

class Astar:

    def __init__(self,inflate_length):
        self.Nx=1 # number of width of the map
        self.Ny=1 # number of height of the map
        self.map_mat=None # numpy matrix of 2D occupancy grid
        self.map_mat_inflated=None # numpy matrix of 2D inflated occupancy grid
        self.map=OccupancyGrid() # 2D occupancy grid matrix
        self.map_inflated=OccupancyGrid() # inflated 2D occupancy grid matrix

        self.sub=rospy.Subscriber("/map",OccupancyGrid,self.occupancy_callback,queue_size=1)
        self.pub=rospy.Publisher("/a_star_solution",Path,queue_size=1)
        self.pub_map=rospy.Publisher("/map_inflated_snu",OccupancyGrid,queue_size=1)
        self.server=rospy.Service("path_computation",PathCompute,self.server_callback)

        self.path=Path() # path object to be published
        self.path.header.frame_id="map"
        self.isPathExist=False
        self.inflate_len=inflate_length # inflation length [m] for occupancy grid

    '''NOTE : callback function in class '''
    def occupancy_callback(self,data):
        # input : Occupancy grid
        self.map=data
        self.Nx=self.map.info.width
        self.Ny=self.map.info.height
        thres=30
        mat=np.array(self.map.data)

        unknown_idx=np.where(mat == -1)[0]
        occupied_idx = np.where(mat > thres)[0]
        free_idx=np.where(mat < thres)[0]

        mat[unknown_idx]=1 # set unknown = occupied
        mat[occupied_idx] = 1
        mat[free_idx] = 0

        # np.savetxt('./map.txt', mat, '%d')

        self.map_mat=np.reshape(mat,(self.Nx,self.Ny))
        # np.savetxt('./map.txt', self.map_mat, '%d')

    '''inflation of occupancy grid map for safety'''

    def inflate(self):
        # we save the inflated map / map_mat to class
        # distance field
        DF=ndimage.distance_transform_edt(1-self.map_mat)
        thres=(self.inflate_len/self.map.info.resolution)
        inflated_occupied_idx=DF<thres
        inflated_free_idx=DF>=thres
        DF[inflated_free_idx]=0
        DF[inflated_occupied_idx] = 1

        # save matrix
        self.map_mat_inflated=np.copy(DF)

        # save occupancy
        self.map_inflated=self.map
        self.map_inflated.data=(self.map_mat_inflated).flatten()




    '''service callback'''
    def server_callback(self,req):
        self.solve_path(req.start,req.goal)
        return []

    '''conversion from index into Cartesian coordinate'''
    def idx2coord(self,row,col):
        # input : ix(int) iy(int)
        # output Point object of geometry_msgs

        Ox=self.map.info.origin.position.x # origin x position
        Oy=self.map.info.origin.position.y # origin y position
        res=self.map.info.resolution
        coord=Point()
        coord.x=Ox+col*res; coord.y=Oy+row*res

        return coord

    '''conversion from Cartesian coordinate into index'''
    def coord2idx(self,point):
        # input : ix(int) iy(int)
        # output Point object of geometry_msgs

        Ox=self.map.info.origin.position.x # origin x position
        Oy=self.map.info.origin.position.y # origin y position
        res=self.map.info.resolution

        col=int(np.floor((point.x-Ox)/res))
        row=int(np.floor((point.y-Oy)/res))
        return (row,col)

    '''solving A* algorithm'''
    def solve_path(self,start,goal):
        # input : start /goal (geometry_msgs: Point)
        # firstly inflate the map
        self.inflate()

        path_idx=astar(self.map_mat_inflated,self.coord2idx(start),self.coord2idx(goal))
        if not path_idx==False:
            self.isPathExist=True
            for point_idx in path_idx:
                point=self.idx2coord(point_idx[0],point_idx[1])
                poseStamped=PoseStamped()
                poseStamped.pose.position.x = point.x
                poseStamped.pose.position.y = point.y
                self.path.poses.append(poseStamped)
        else:
            rospy.logwarn("path does not exist. do either : 1) adjust the inflation rate, 2) select feasible search point")

    ''' publish the path computed '''
    def path_publish(self):
        if self.isPathExist:
            self.pub.publish(self.path)

    def inflated_map_publish(self):
        self.pub_map.publish(self.map_inflated)


if __name__=='__main__':
    rospy.init_node('CBBA_node')
    rospy.loginfo('A_star_solver started')

    inflate_len=0.2
    a_star=Astar(inflate_len)

    rate=rospy.Rate(30)

    while not rospy.is_shutdown():
        a_star.path_publish()
        a_star.inflated_map_publish()







