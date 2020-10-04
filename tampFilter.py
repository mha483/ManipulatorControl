# -*- coding: utf-8 -*-
"""
Created on Tue Sep 29 18:21:38 2020

@author: dbr84
"""

import robolink as rl    # RoboDK API
import robodk as rdk     # Robot toolbox
import numpy as np
from HomoTrans import *

RDK = rl.Robolink()

robot = RDK.Item('UR5')
world_frame = RDK.Item('UR5 Base')
home = RDK.Item('Home')   # existing target in station
robot.setPoseFrame(world_frame)
robot.setPoseTool(robot.PoseTool()) 

#HTs
T_T_B = getT_T_B()
T_Tamp_T = getT_Tamp_T()
T_A_Filt = np.linalg.inv(getT_Filt_A())
T_Arm_A = np.linalg.inv(getT_A_Arm())
T_Tamp_T_prepos = getT_Tamp_T_prepos()


testMat = [T_T_B,T_Tamp_T,T_A_Filt,T_Arm_A]

# Tamp Prepos


target_prepos = matrixMultiply([T_T_B,T_Tamp_T_prepos,T_A_Filt,T_Arm_A])

# Under Tamp
P = np.array([-40,0,0])
T_pos_Tamp = makeTransMat(np.eye(3),P)




# In Tamp
target2 = matrixMultiply([T_T_B,T_Tamp_T,T_A_Filt,T_Arm_A]) 

target1_r = rdk.Mat(target1.tolist())
target2_r = rdk.Mat(target2.tolist())



# Directly use the RDK Matrix object from to hold pose (its an HT)
T_home = rdk.Mat([[     0.000000,     0.000000,     1.000000,   523.370000 ],
     [-1.000000,     0.000000,     0.000000,  -109.000000 ],
     [-0.000000,    -1.000000,     0.000000,   607.850000 ],
      [0.000000,     0.000000,     0.000000,     1.000000 ]])

robot.MoveJ(home)
# Move to neutral pos 
robot.MoveJ([30.000000, -90.000000, -106.070000, -154.280000, 289.290000, 0.000000])
robot.MoveJ([30.000000, -90.000000, -151.070000, -118.930000, 273.200000, 0.000000])
rdk.pause(1)
# Tamping prepos
robot.MoveJ([-1.698923, -95.818653, -146.511129, -108.825103, 238.089400, 144.702398])
rdk.pause(1)
# Under Tamping
robot.MoveJ(target1_r, blocking=True)
rdk.pause(1)
# Tamp
robot.MoveL(target2_r, blocking=True)
rdk.pause(1)
# Untamp
robot.MoveL(target1_r, blocking=True)
#return to prepos
robot.MoveJ([-1.698923, -95.818653, -146.511129, -108.825103, 238.089400, 144.702398])


# and... move home to an existing target
robot.MoveJ(home)