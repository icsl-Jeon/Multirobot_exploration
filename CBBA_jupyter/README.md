# runfile 
CBBA_runfile.py 

####################################################################
# approach 1 
####################################################################

# structure 

## GENERAL USAGE

1. press s : search position receiving start  

activate subscription of topics : please select search positions in Rviz 

received [x,y]!

2. press e : search position receiving end 

3. press d : solve CBBA and keep publishing + visualization 

## SERVICE NODE :

1. subscription : map / position of robots 

2. service : 

search positions 

call ->  sequence of search positions for each robot 

# TODO 

0. RViz-> click search positions 

1. measure the distance. should I give A* for Score_calculation ? 

2. path generation : how to set the waypoints between the goal position and initial point

most feasible method : solving A* by receiving the "costmap" : I think it is better to code on my own 
 
3. Visualization


####################################################################
# approach 2 : QT gui making  
####################################################################


