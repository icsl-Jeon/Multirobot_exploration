from CBBA import *

def CBBA_main(agents,tasks,Lt,connectivity_graph):

    # TODO FOR USER : CM (COMPATIBILITY MATRIX) IN CLASS CBBA

    # this costructor includes all the initilization
    cbba=CBBA(agents,tasks,Lt,connectivity_graph)
    T=1 # current iteration
    lastTime=T-1 # time when something changed
    doneFlag=0

    while doneFlag==0:
        print('current interation :{} '.format(T))
        # -------------------
        # 1. COMMUNICATE
        # -------------------
        cbba.consensus(T)

        # --------------------------
        # 2. CBBA BUNDLE BUILDING
        # --------------------------

        for n in range(cbba.Nu):
            newBid=cbba.bundle_update(n)
            if newBid:
                lastTime=T

        # --------------------
        # CONVERGENCE CHECK
        # --------------------

        if T-lastTime > cbba.Nu:
            doneFlag=1
        elif (T-lastTime > 2*cbba.Nu):
            print ('Algorithm did not converge due to communication trouble')
            doneFlag=1
        else:
            T=T+1

    # TODO for coding : mapping. but I don't get the necessity


    # Compute the total score
    total_score=0
    for n in range(cbba.Nu):
        for m in range(cbba.Lt):
            if cbba.scores[n,m]>-1:
                total_score+=cbba.scores[n,m]
            else :
                break

    print ('total score : {}'.format(total_score))
    return cbba


def CBBA_solve(task_positions,unit_positions):
    Nt=len(task_positions)
    Nu=len(unit_positions)

    agent_list=[], task_list=[]
    ##############
    # AGENT LIST #
    ##############

    # agent list construction
    for i in range(Nu):
        # PARSING DATA
        # position
        agent_pos_x = unit_positions[i].x
        agent_pos_y = unit_positions[i].y
        agent_pos_z = 0.0

        # type
        agent_type = 0

        agent_unit = agent(agent_pos_x, agent_pos_y, agent_pos_z, idx=i, agent_type=agent_type, fuel=1, nom_vel=2)
        agent_list.append(agent_unit)

    #############
    # TASK LIST #
    #############

    # task list construction
    for j in range(Nt):
        # PARSING DATA
        # position
        task_pos_x = task_positions[j].x
        task_pos_y = task_positions[j].y
        task_pos_z = 0.0
        task_type = 0 # type
        value = 100
        start_t = 0.0
        end_t = 100.0
        duration = 1
        discount = 0.1

        task_unit = task(idx=j, task_type=task_type, x=task_pos_x, y=task_pos_y, z=task_pos_z, start_t=start_t,
                         end_t=end_t, task_value=100, duration=duration)
        task_list.append(task_unit)

    # connectivity graph
    connectivity_graph = np.ones((Nu, Nu)) - np.eye(Nu)

    ##############
    # CBBA SOLVE #
    ##############

    cbba = CBBA_main(agent_list, task_list, Lt=10, connectivity_graph=connectivity_graph)
    return cbba


