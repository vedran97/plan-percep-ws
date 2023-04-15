# -*- coding: utf-8 -*-
"""
Created on Sat Apr  8 02:23:47 2023

@author: saksham
"""
import numpy as np
import matplotlib.pyplot as plt

L = 19.5
r = 6.45/2

angle_turn = 3.14
dist = 600
#%%
##### Input#####

# Shape is (turnOrGo,riseRate/fallRate,max_WheelRPM,magnitude)

turnOrGo = 1 # 0 ofr turn, 1 for linear
rise_fall_Rate = 1 # rad/sec/sec
max_WheelRPM = 6 # rad/sec

magnitude = dist

data = (turnOrGo,rise_fall_Rate,max_WheelRPM,magnitude)
# publish (data)
#%%

####### Output ########

# Get (data)


def generateTrajec(data):
    # Decide timeStep
    timeStep = 0.05
    
    turnOrGo,rise_fall_Rate,max_WheelRPM,magnitude = data
    
    W_turn = np.array([0,0])
    
    ## Turning
    if turnOrGo == 0:
    
        dT = np.sqrt(4*abs(magnitude)/rise_fall_Rate)
        W_turn_ = np.concatenate((rise_fall_Rate*np.arange(0,dT/2,timeStep),
                                  rise_fall_Rate*np.arange(dT/2,0,-timeStep)))
        W_turn_ = (L/(2*r))*np.sign(angle_turn)*np.column_stack((-W_turn_,W_turn_))
        
        W_turn = np.vstack((W_turn,W_turn_))
    
    ## Moving
    if turnOrGo == 1:
        
        if magnitude < (L/2)*(max_WheelRPM/rise_fall_Rate):
            dT = 2*np.sqrt(2*magnitude/(L*rise_fall_Rate))
            W_turn_ = np.concatenate((np.arange(0,dT/2,timeStep),np.arange(dT/2,0,-timeStep)))
            W_turn_ = W_turn_ * rise_fall_Rate
        else:
            dT = (magnitude*2)/(L*max_WheelRPM) + (max_WheelRPM/rise_fall_Rate)
            W_turn1 = np.concatenate((rise_fall_Rate*np.arange(0,dT/2,timeStep),
                                      rise_fall_Rate*np.arange(dT/2,0,-timeStep)))
            W_turn2 = max_WheelRPM*np.ones(np.shape(W_turn1))
            W_turn_ = np.minimum(W_turn1,W_turn2)
        W_turn_ = np.column_stack((W_turn_,W_turn_))
        W_turn = np.vstack((W_turn,W_turn_))
    
    return W_turn
    
#%%
# printing and checking
plt.plot(generateTrajec(data),'.',ms=2)