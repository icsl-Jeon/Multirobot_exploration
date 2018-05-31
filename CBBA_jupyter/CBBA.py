import numpy as np

# ---------- GLOBAL VARIABLE ----------------
TASK_TYPE = [0, 1]  # type = {rover,quadrotor}
AGENT_TYPE = [0, 1]  # type = {exploration,rescue}
Lt = 3  # maximum number of task allocation


class agent:
    def __init__(self, x, y, z, idx=0, agent_type=0, fuel=1, nom_vel=1):
        # -------- PARAMETERS ------------
        self.idx = idx  # index of agent
        self.type = agent_type
        self.x = x
        self.y = y
        self.z = z
        # AVAILABLE START TIME OF THIS AGENT
        self.avail = 0  # change if needed
        self.fuel = fuel  # fuel penalty per meter
        self.nom_vel = 1  # nominal velocity in [m/s]


class task:
    def __init__(self, x, y, z, duration, idx=0, task_type=0, start_t=0, end_t=100, discount=0.1, task_value=10):
        self.idx = idx
        self.type = task_type
        self.x = x
        self.y = y
        self.z = z
        self.start_t = start_t  # mission avail start time [sec]
        self.end_t = end_t  # end time
        self.duration = duration  # duration for performing task
        self.discount = discount  # reward will be discounted in the ratio of discount
        self.value = task_value  # reward of reward


class CBBA:
    def __init__(self, agents, tasks, Lt, connectivity_graph):
        self.agent_list = agents  # number of agent = Nu
        self.task_list = tasks  # number of tasks = Nt
        self.connectivity_graph = connectivity_graph  # communication graph (Nu x Nu numpy)

        # -------- PARAMETERS ------------
        self.Nu = len(self.agent_list)
        self.Nt = len(self.task_list)
        self.Lt = Lt  # maximum number of task to an agent
        # TODO FOR USER
        self.CM = np.zeros(len(AGENT_TYPE), len(TASK_TYPE))  # compatibility matrix
        self.CM[0, 0] = 1
        self.CM[1, 1] = 1
        self.update_T = np.zeros(self.Nu, self.Nu)  # Nt x Nt matrix whose element : update time
        # --------- CBBA Data ------------
        self.bids = -np.ones(self.Nu, self.Nt)  # my bidding
        self.winners = -np.ones(self.Nu, self.Nt)  # whom does each agent regard as winner? (Z)
        self.winnerBids = -np.ones(self.Nu, self.Nt)  # bidding of winner (Y)

        self.path = -np.ones(self.Nu, self.Nt)
        self.bundle = -np.ones(self.Nu, self.Nt)
        self.times = -np.ones(self.Nu, Lt)
        self.scores = -np.ones(self.Nu, Lt)



    def insert(oldList, value, index):
        # -------- ARGUMENTS ------------
        ## INPUT
        # oldList : original list. empty loc was assigned -1
        # value : value inserted at index of oldList
        # index :

        ## OUTPUT
        # newList

        newList = -np.ones(1, len(oldList))
        newList[0:index - 1] = oldList[0:index - 1]
        newList[index] = value
        newList[index + 1:] = oldList[index:-1]  # I don't know why -1...

        return newList

    def remove(oldList, index):
        newList = -np.ones(1, len(oldList))
        newList[0:index - 1] = oldList[0:index - 1]
        newList[index:-1] = oldList[index:]

        return newList

    def computeBids(self, agent_idx, feasibility):

        # -------- ARGUMENTS ------------

        ## INPUT
        # feasiilibty : is feasible for m th task to be inserted to j th position in path ?
        # agent_idx : agent whose bid to be calculated

        ## OUTPUT
        # bestIdxs : for tasks which are not included yet, which location was best?
        # taskTimes : for tasks which are not included yet, which exe time was best?
        # feasiilibty : is feasible for m th task to be inserted to j th position in path ?

        # -------- DESCRIPTION ------------

        # Compute the bid of agent, and save it to self.bids in the class
        # Here, bid for m th task of a unit means : if not m included in bundle yet, then
        # what is maximum score we can get by insertring the task to the path of agent

        available_idx = np.where(self.path[agent_idx] == -1)[0]  # indecies which are not occupied
        isFull = available_idx.size == 0  # bundle full?

        if isFull:
            return

        bids = np.zeros(1, self.Nt)  # bids of current agent
        bestIdxs = -np.ones(1, self.Nt)  # best position of each task
        taskTimes = -np.ones(1, self.Nt)  # best exe time of each task
        cur_agent=self.agent_list[agent_idx]
        # for each task
        for m in range(self.Nt):
            # Check the compatibility
            if self.CM[cur_agent.agent_type, self.task_list[m].type]:
                # Check to make sure the path doesn't already contain task m
                if np.empty(np.where(self.path[agent_idx, 0:available_idx[0] - 1] == m)):
                    # Find the best score attainable by inserting the score into the current path
                    bestBid = 0;
                    bestIndex = -1;
                    bestTime = -1

                    # Try inserting task m in location j among []other tasks and see if it generates a better new_path.
                    for j in range(available_idx[0] + 1):
                        if feasibility[m, j] == 1:
                            skip = False

                            if j == 0:  # insert at the beginning
                                taskPrev = []
                                timePrev = []
                            else:
                                taskPrev = self.task_list[self.path[agent_idx, j - 1]]
                                timePrev = self.times[agent_idx, j - 1]

                            if j == available_idx[0]:  # insert at the end
                                taskNext = []
                                timeNext = []
                            else:
                                taskNext = self.task_list[self.path[agent_idx, j]]
                                timeNext = self.times[agent_idx, j]

                            outarg = self.CalcScore(agent_idx, self.task_list[m], taskPrev, timePrev, taskNext,
                                                    timeNext)
                            if len(outarg):  # normal  output
                                score = outarg[0];
                                minStart = outarg[1];
                                maxStart = outarg[2]
                                if minStart > maxStart:
                                    skip = True
                                    # infeasible path
                                    feasibility[m, j] = 0

                                if not skip:
                                    # we found best position of m th = it is j until the current
                                    if score > bestBid:
                                        bestBid = score
                                        bestIndex = j
                                        bestTime = minStart

                            else:
                                print ('score calculation error')
                                return

                    if bestBid > 0:
                        bids[m] = bestBid
                        bestIdxs[m] = bestIndex
                        taskTimes[m] = bestTime

        self.bids[agent_idx] = np.copy(bids)  # update into the field in class
        return (bestIdxs, taskTimes, feasibility)

    def CalcScore(self, agent_idx, taskCur, taskPrev, timePrev, taskNext, timeNext):
        # INPUT
        # agent_idx = int
        # taskCur, taskPrev, taskNext = task class

        cur_agent = self.agent_list[agent_idx]

        if np.size(np.where(AGENT_TYPE == cur_agent.type)[0]):
            if np.empty(taskPrev):  # first task in path
                # compute start time of the task
                dt = np.sqrt(np.power(cur_agent.x - taskCur.x, 2) +
                             np.power(cur_agent.y - taskCur.y, 2) +
                             np.power(cur_agent.z - taskCur.z, 2)) / cur_agent.nom_vel
                minStart = max(taskCur.start_t, cur_agent.avail + dt)

            else:  # Not first task in path
                dt = np.sqrt(np.power(taskPrev.x - taskCur.x, 2) +
                             np.power(taskPrev.y - taskCur.y, 2) +
                             np.power(taskPrev.z - taskCur.z, 2)) / cur_agent.nom_vel
                minStart = max(taskCur.start_t, timePrev + taskPrev.duration + dt)

            if np.empty(taskNext):  # last task in path
                maxStart = taskCur.end_t
            else:
                dt = np.sqrt(np.power(taskNext.x - taskCur.x, 2) +
                             np.power(taskNext.y - taskCur.y, 2) +
                             np.power(taskNext.z - taskCur.z, 2)) / cur_agent.nom_vel
                maxStart = min(taskCur.end_t, timeNext - taskCur.duration - dt)

            # compute score
            reward = taskCur.value * np.exp(-taskCur.discount * (minStart - taskCur.start_t))

            # Subtract fuel cost. Implement constant fuel to ensure DMG.
            # NOTE: This is a fake score since it double counts fuel.  Should
            # not be used when comparing to optimal score.  Need to compute
            # real score of CBBA paths once CBBA algorithm has finished running

            penalty = cur_agent.fuel * np.sqrt(np.power(cur_agent.x - taskCur.x, 2) +
                                               np.power(cur_agent.y - taskCur.y, 2) +
                                               np.power(cur_agent.z - taskCur.z, 2))

            score = reward - penalty
            return (score, minStart, maxStart)

        else:
            print ('unknown agent type')
            return ()

    def bundle(self, agent_idx):
        # Update bundles after messaging to drop tasks that are outbid
        self.bundleRemove(agent_idx)

        # Bid on new tasks and add them to the bundle
        newBid = self.bundleAdd(agent_idx)

        return newBid

    def bundleAdd(self, agent_idx):
        ##################################
        # -------- ARGUMENTS ------------#
        ##################################
        # * INPUT *#
        # agent_idx : agent whose bid to be calculated

        # * OUTPUT *#
        # newbid : is new bid flag

        ####################################
        # -------- DESCRIPTION ------------#
        ####################################

        # This function add tasks to bundle until the bundle is full (Algorithm 3 : 6-15 line)

        newBid = 0  #
        eps = 1e-6
        cur_agent = self.agents[agent_idx]

        # check if bundle is full
        isFull = np.where(self.bundle[agent_idx] == -1)[0].size == 0
        # initialize feasible matrix (to keep track of which j locations can be pruned)
        feasibility = np.ones(self.Nt, self.Lt + 1)

        while not isFull:  # numel(bi) < Lt
            outarg = self.computeBids(agent_idx, feasibility)
            bestIdxs = outarg[0];
            taskTimes = outarg[1];
            feasibility = outarg[2]

            # determine available assignments

            D1 = (self.bids[agent_idx] - self.winnerBids[agent_idx]) > eps
            D2 = np.abs(self.bids[agent_idx] - self.winnerBids[agent_idx]) < eps
            D3 = agent_idx < self.winners[agent_idx]
            D = D1 | (D2 & D3)

            # select the assignment that will improve the score the most and place bid
            value = np.max(np.multiply(D, self.bids[agent_idx]))
            bestTask = np.argmax(np.multiply(D, self.bids[agent_idx]))

            if value > 0:
                # Set new bid flag
                newBid = 1

                # check for tie
                allbestTask = np.where(np.multiply(D, self.bids[agent_idx]) == value)[0]
                if len(allbestTask) == 1:
                    bestTask = allbestTask
                else:
                    # if there are multiple best task, we select the one that starts in the eariliest (in real time)
                    earliest = 10000
                    for cur_task in allbestTask:
                        if self.task_list[cur_task].start_t < earliest:
                            earliest = self.task_list[cur_task].start_t
                            bestTask = cur_task

                # place bid : I think I have bestTask th task
                self.winners[agent_idx][bestTask] = agent_idx
                self.winnerBids[agent_idx][bestTask] = self.bids[agent_idx][bestTask]

                self.path[agent_idx] = self.insert(self.path[agent_idx], bestTask, bestIdxs[bestTask])
                self.times[agent_idx] = self.insert(self.times[agent_idx], taskTimes[bestTask], bestIdxs[bestTask])
                self.scores[agent_idx] = self.insert(self.scores[agent_idx], self.bids[agent_idx, bestTask],
                                                     bestIdxs[bestTask])

                # insert task to bundle
                insert_loc = len(np.where(self.bundle[agent_idx] > -1)[0])
                self.bundle[agent_idx, insert_loc] = bestTask

                # update feasibility
                for i in range(self.Nt):
                    feasibility[i, :] = self.insert(feasibility[i, :], feasibility[i, bestIdxs[bestTask]])

            else:  # nothing to bid
                break

            isFull = np.where(self.bundle[agent_idx] == -1)[0].size == 0

        return newBid

    def bundleRemove(self, agent_idx):
        ##################################
        # -------- ARGUMENTS ------------#
        ##################################
        # * INPUT *#

        # * OUTPUT *#

        ####################################
        # -------- DESCRIPTION ------------#
        ####################################

        # Update after communication. for the outbid agents, release from bundles
        outbidForTask = 0;

        for j in range(self.Lt):
            if self.bundle[agent_idx, j] < 0:
                break
            else:  # test if agent has been outbid for a task. if it has, release it and all subsequent tasks in its path
                if not self.winners[agent_idx, self.bundle[agent_idx, j]] == agent_idx:
                    outbidForTask = 1
                if outbidForTask:
                    if self.winners[agent_idx, self.bundle[agent_idx, j]] == agent_idx:
                        # Remove from winner list if in there
                        self.winners[agent_idx, self.bundle[agent_idx, j]] = -1
                        self.winnerBids[agent_idx, self.bundle[agent_idx, j]] = -1

                        # Clear from path and times vectors and remove from bindle
                        idx = np.where(self.path[agent_idx] == self.bundle[agent_idx, j])

                        self.path[agent_idx] = self.remove(self.path[agent_idx], idx)
                        self.times[agent_idx] = self.remove(self.times[agent_idx], idx)
                        self.scores[agent_idx] = self.remove(self.scores[agent_idx], idx)

                        self.bundle[agent_idx, j] = -1

    def consensus(self, update_time):
        # update_time : iteration step
        # perform consensus updating the winner and winner bids
        old_Z = np.copy(self.winners)
        old_Y = np.copy(self.winnerBids)
        old_t = np.copy(self.update_T)
        Z = np.copy(old_Z)
        Y = np.copy(old_Y)

        eps = 1e-6

        # sender = k
        # receiver = i
        # task = j
        for k in range(self.Nu):
            for i in range(self.Nu):
                if self.connectivity_graph[k, i] == 1:  # if two agent is connected
                    # implement table for each task

                    for j in range(self.Nt):

                        # Entries 1 to 4: Sender thinks he has the task
                        if old_Z[k, j] == k:
                            # Entry 1: Update or Leave
                            if Z[i, j] == i:
                                if old_Y[k, j] - Y[i, j] > eps:
                                    Z[i, j] = old_Z[k, j]
                                    Y[i, j] = old_Y[k, j]
                                elif abs(old_Y[k, j] - Y[i, j]) < eps:  # almost equal
                                    if Z[i, j] > old_Z[k, j]:  # if then, grant winning to smaller unit
                                        Z[i, j] = old_Z[k, j]
                                        Y[i, j] = old_Y[k, j]

                            # Entry 2: Update (redundant)
                            elif Z[i, j] == k:
                                Z[i, j] = old_Z[k, j]
                                Y[i, j] = old_Y[k, j]

                            # Entry 3: Update or Leave
                            elif Z[i, j] > -1:  # not both i,k.
                                # Update
                                if old_t[k, Z[i, j]] > self.update_T[i, Z[i, j]]:  # if sender is more recent
                                    Z[i, j] = old_Z[k, j]
                                    Y[i, j] = old_Y[k, j]
                                # Update
                                elif old_Y[k, j] - Y[i, j] > eps:
                                    Z[i, j] = old_Z[k, j]
                                    Y[i, j] = old_Y[k, j]
                                elif np.abs(old_Y[k, j] - Y[i, j]) < eps:
                                    if Z[i, j] > old_Z[k, j]:
                                        Z[i, j] = old_Z[k, j]
                                        Y[i, j] = old_Y[k, j]

                                        # Entry 4: Update
                            elif Z[i, j] == -1:
                                Z[i, j] = old_Z[k, j]
                                Y[i, j] = old_Y[k, j]

                            else:
                                print('unknown winner: {} '.format(Z[i, j]))


                        # Entries 5 to 8: Sender thinks receiver has the task
                        elif old_Z[k, j] == i:
                            # Entry 5: Leave
                            if Z[i, j] == i:
                                # do nothing
                                100
                            # Entry 6: Reset
                            elif Z[i, j] == k:
                                Z[i, j] = -1
                                Y[i, j] = -1

                            # Entry 7: Reset or Leave
                            elif Z[i, j] > -1:
                                if old_t[k, Z[i, j]] > self.update_T[i, Z[i, j]]:  # reset
                                    Z[i, j] = -1
                                    Y[i, j] = -1

                            elif Z[i, j] == -1:
                                # do nothing
                                100
                            else:
                                print('unknown winner: {} '.format(Z[i, j]))



                        # Entries 9 to 13: Sender thinks someone else has the task
                        elif old_Z[k, j] > -1:

                            # Entry 9: Update or Leave
                            if Z[i, j] == i:
                                if old_t[k, old_Z[k, j]] > self.update_T[i, old_Z[k, j]]:
                                    if old_Y[k, j] - Y[i, j] > eps:
                                        Z[i, j] = old_Z[k, j]
                                        Y[i, j] = old_Y[k, j]
                                    elif abs(old_Y[k, j] - Y[i, j]) <= eps:  # equal scores
                                        if Z[i, j] > old_Z[k, j]:
                                            Z[i, j] = old_Z[k, j]
                                            Y[i, j] = old_Y[k, j]

                            # Entry 10: Update or reset
                            elif Z[i, j] == k:
                                # update
                                if old_t[k, old_Z[k, j]] > self.update_T[i, old_Z[k, j]]:
                                    Z[i, j] = old_Z[k, j]
                                    Y[i, j] = old_Y[k, j]
                                # reset (maybe wrong information)
                                else:
                                    Z[i, j] = -1
                                    Y[i, j] = -1

                            # Entry 11: Update or Leave
                            elif Z[i, j] == old_Z[k, j]:  # same guess
                                if old_t[k, old_Z[k, j]] > self.update_T[i, old_Z[k, j]]:
                                    Z[i, j] = self.old_Z[k, j]  # redundant operation
                                    Y[i, j] = self.old_Y[k, j]

                            # Entry 12: Update, Reset or Leave
                            elif Z[i, j] > -1:  # different guess about 3rd agent
                                if old_t[k, Z[i, j]] > self.update_T[i, Z[i, j]]:  # if recieved info is more recent
                                    # update
                                    if old_t[k, old_Z[k, j]] >= self.update_T[i, old_Z[k, j]]:
                                        Z[i, j] = old_Z[k, j]
                                        Y[i, j] = old_Y[k, j]
                                    # reset
                                    elif old_t[k, old_Z[k, j]] < self.update_T[i, old_Z[k, j]]:
                                        Z[i, j] = -1
                                        Y[i, j] = -1
                                    else:
                                        print('Should not be here, please revise')
                                else:
                                    if old_t[k, old_Z[k, j]] > self.update_T[i, old_Z[k, j]]:
                                        # update
                                        if old_Y[k, j] - Y[i, j] > eps:
                                            Z[i, j] = old_Z[k, j]
                                            Y[i, j] = old_Y[k, j]
                                        # Equal score
                                        elif np.abs(old_Y[k, j] - Y[i, j]) < eps:
                                            if self.winner[i, j] > old_Z[k, j]:
                                                Z[i, j] = old_Z[k, j]
                                                Y[i, j] = old_Y[k, j]


                            # Entry 13: Update or Leave
                            elif Z[i, j] == -1:
                                if old_t[k, old_Z[k, j]] > self.update_T[i, old_Z[k, j]]:
                                    Z[i, j] = old_Z[k, j]
                                    Y[i, j] = old_Y[k, j]

                            else:
                                print('unknown winner: {} '.format(Z[i, j]))


                        # Entries 14 to 17: Sender thinks no one has the task
                        elif old_Z[k, j] == -1:
                            # Entry 14: leave
                            if Z[i, j] == i:
                                # do nothing
                                100
                            # Entry 15: update
                            elif Z[i, j] == k:
                                Z[i, j] = old_Z[k, j]
                                Y[i, j] = old_Y[k, j]
                            # Entry 16: Update or leave
                            elif Z[i, j] > -1:
                                if old_t[k, old_Z[i, j]] > self.update_T[i, old_Z[i, j]]:
                                    Z[i, j] = old_Z[k, j]
                                    Y[i, j] = old_Y[k, j]

                            # Entry 17: leave
                            elif Z[i, j] == -1:
                                # do nothing
                                100
                            else:
                                print('unknown winner: {} '.format(Z[i, j]))

                        # update time stamp
                        for n in range(self.Nu):
                            if (n != i) and (self.update_T[i, n] < old_t[k, n]):
                                self.update_T = old_t[k, n]

                        self.update_T[i, k] = update_time

        # copy data
        self.winners = np.copy(Z)
        self.winnerBids = np.copy(Y)

