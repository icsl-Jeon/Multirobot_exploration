from CBBA import *

def CBBA_main(agents,tasks,Lt,connectivity_graph):

    # TODO FOR USER : CM (COMPATIBILITY MATRIX) IN CLASS CBBA

    # this costructor includes all the initilization
    cbba=CBBA(agent,tasks,Lt,connectivity_graph)
    T=1 # current iteration
    lastTime=T-1 # time when something changed
    doneFlag=0

    while doneFlag==0:
        # -------------------
        # 1. COMMUNICATE
        # -------------------
        cbba.consensus(T)

        # --------------------------
        # 2. CBBA BUNDLE BUILDING
        # --------------------------

        for n in range(cbba.Nu):
            newBid=cbba.bundle(n)
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



