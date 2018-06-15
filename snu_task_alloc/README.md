# PREREQUISITIE 

1. rviz config for 

(1) MarkerArray : /search_position_markers

(2) Path : /allocation_path0 , /allocation_path1, /allocation_path2

(3) fixed frame : /map

2. map server for occupancy grid publish

rosrun map_server map_server ~/kukmin_ws/src/KIRO_simulator/turtlebot3/turtlebot3_navigation/maps/map.yaml

# USAGE

1. rviz

2. roslaunch snu_task_alloc run_CBBA.launch

3. rosservice call 


