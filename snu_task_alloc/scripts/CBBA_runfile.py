from CBBA_Main import *
import scipy.io as sio
data=sio.loadmat('example_data')


agents_info=data['agents'][0]; task_info=data['tasks'][0]
Nu=len(agents_info); Nt=len(task_info)

agent_list=[]; task_list=[]


##############
# AGENT LIST #
##############

# agent list construction
for i in range(Nu):
    # PARSING DATA
    # position
    agent_pos_x=agents_info['x'][i][0][0]
    agent_pos_y = agents_info['y'][i][0][0]
    agent_pos_z = agents_info['z'][i][0][0]

    # type
    agent_type=agents_info['type'][i][0][0]-1

    agent_unit=agent(agent_pos_x,agent_pos_y,agent_pos_z,idx=i,agent_type=agent_type,fuel=1,nom_vel=2)
    agent_list.append(agent_unit)

#############
# TASK LIST #
#############

# task list construction
for j in range(Nt):
    # PARSING DATA
    # position
    task_pos_x = task_info['x'][j][0][0]
    task_pos_y = task_info['y'][j][0][0]
    task_pos_z = task_info['z'][j][0][0]
    task_type = task_info['type'][j][0][0]-1 # type
    value=100
    start_t=task_info['start'][j][0][0]
    end_t=task_info['end'][j][0][0]
    duration=task_info['duration'][j][0][0]
    discount=0.1

    task_unit=task(idx=j,task_type=task_type,x=task_pos_x,y=task_pos_y,z=task_pos_z,start_t=start_t,end_t=end_t,task_value=100,duration=duration)
    task_list.append(task_unit)

# connectivity graph
connectivity_graph=np.ones((Nu,Nu))-np.eye(Nu)

##############
# CBBA SOLVE #
##############

cbba=CBBA_main(agent_list,task_list,Lt=10,connectivity_graph=connectivity_graph)
