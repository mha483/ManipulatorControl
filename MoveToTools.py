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

'''
Machine Positions
'''
# Grinder in Base
T_G_B = getT_G_B()
# Tamping stand in Base
T_T_B = getT_T_B()
# Coffee machine in Base
T_M_B = getT_M_B()
# Cup Stack in Base
T_C_B = getT_C_B()

# Tool in Arm
T_Arm_A = np.linalg.inv(getT_A_Arm())

'''
Tool Transforms
'''
# Portafilter in Arm
T_A_Filt = np.linalg.inv(getT_Filt_A())

# Pushy part of grinder tool
T_Pushy_G = np.linalg.inv(getT_Pushy_G())
# Pully part of grinder tool
T_Pully_G = np.linalg.inv(getT_Pully_G())

# Centre of cup tool
T_Centre_C = getT_Centre_C()

'''
Grinder Positions
'''
# Ball bearing (PF2) in Grinder
T_PF_G = getT_PF_G()
# Button 1 in Grinder
T_B1_G = getT_B1_G()
# Button 2 in Grinder
T_B2_G = getT_B2_G()
# Pull tab in grinder
T_Pull_G = getT_Pull_G()

'''
Tamping Stand Positions
'''
# Tamper in Tamping Stand
T_Tamp_T = getT_Tamp_T()
# Scraper in Tamping Stand
T_Scrape_T = getT_Scrape_T()

'''
Coffee Machine Positions
'''
# Button 1 in Machine
T_B1_M = getT_B1_M()
# Button 2 in Machine
T_B2_M = getT_B2_M()
# Button 3 in Machine
T_B3_M = getT_B3_M()
# Button 4 in Machine
T_B4_M = getT_B4_M()
# Portafilter in Machine
T_PF_M = getT_PF_M()

'''
Cup Positions
'''
# Stack
T_Cup_C = getT_Cup_C()



'''
General
'''
# Out in Z for buttons
P_But = np.array([0,0,-20])
T_Pre_But = makeTransMat(np.eye(3),P_But)

# Up in x for attaching portafilter to grinder
P_G = np.array([100,0,0])
T_Pre_G = makeTransMat(np.eye(3),P_G)

# Down in x for tamping
P_Tamp = np.array([-40,0,0])
Pre_Tamp = makeTransMat(np.eye(3),P_Tamp)

# Pull grinder lever
P_Pull = np.array([0,0,-50])
T_Pull = makeTransMat(np.eye(3),P_Pull)

# Pre scrape
P_Pre_Scrape = np.array([0,0,-50])
Pre_Scrape = makeTransMat(np.eye(3),P_Pre_Scrape)

# Post scrape
P_Scrape = np.array([0,0,50])
Scrape = makeTransMat(np.eye(3),P_Scrape)

# 


# Start home pos
robot.MoveJ(home)

# Home to tool stand
robot.MoveJ([-93.210000, -90.000000, -89.990000, -89.990000, 90.000000, 0.000000])
robot.MoveJ([-102.860000, -80.360000, -61.070000, -122.140000, 90.000000, 0.000000])
robot.MoveJ([-154.290000, -80.360000, -73.930000, -115.710000, 90.000000, -182.59])

# Attach portafilter
RDK.RunProgram("Portafilter Tool Attach (Stand)", True)
rdk.pause(1)


# Move from tool stand to home
robot.MoveJ([-102.860000, -80.360000, -61.070000, -122.140000, 90.000000, 0.000000])
robot.MoveJ(home)

# Transform protafilter in grinder Pos
PF_Grinder = matrixMultiply([T_G_B,T_PF_G,T_Pre_G,T_A_Filt, T_Arm_A])
PF_Grinder_r = rdk.Mat(PF_Grinder.tolist())

# Move close to grinder
robot.MoveJ([-12.860000, -86.790000, -154.290000, -102.860000, -61.070000, 128.570000])

# Move to portafilter in grinder
robot.MoveJ(PF_Grinder_r, blocking=True)

# Attach portafilter to grinder, detatch
RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)
rdk.pause(1)
RDK.RunProgram("Portafilter Tool Detach (Grinder)", True)

# Move home then to tool stand
robot.MoveJ(home)
robot.MoveJ([-93.210000, -90.000000, -89.990000, -89.990000, 90.000000, 0.000000])
robot.MoveJ([-102.860000, -80.360000, -61.070000, -122.140000, 90.000000, 0.000000])
robot.MoveJ([-154.290000, -80.360000, -73.930000, -115.710000, 90.000000, -182.59])

# Attach grinder tool
RDK.RunProgram("Grinder Tool Attach (Stand)", True)

# Move back round to grinder, over coffee machine
robot.MoveJ([-102.860000, -80.360000, -61.070000, -122.140000, 90.000000, 0.000000])

# Move close to buttons on grinder
robot.MoveJ([-67.500000, -131.790000, -90.000000, -86.790000, 186.430000, -167.140000])

# Push button 1
G_Pre_But1 = matrixMultiply([T_G_B,T_B1_G,T_Pre_But,T_Pushy_G, T_Arm_A])
G_Pre_But1_r = rdk.Mat(G_Pre_But1.tolist())
robot.MoveJ(G_Pre_But1_r, blocking=True)
rdk.pause(1)
G_But1 = matrixMultiply([T_G_B,T_B1_G,T_Pushy_G, T_Arm_A])
G_But1_r = rdk.Mat(G_But1.tolist())
robot.MoveJ(G_But1_r, blocking=True)
robot.MoveJ(G_Pre_But1_r, blocking=True)
rdk.pause(1)

# Push button 2
G_Pre_But2 = matrixMultiply([T_G_B,T_B2_G,T_Pre_But,T_Pushy_G, T_Arm_A])
G_Pre_But2_r = rdk.Mat(G_Pre_But2.tolist())
robot.MoveJ(G_Pre_But2_r, blocking=True)
rdk.pause(1)
G_But2 = matrixMultiply([T_G_B,T_B2_G,T_Pushy_G, T_Arm_A])
G_But2_r = rdk.Mat(G_But2.tolist())
robot.MoveJ(G_But2_r, blocking=True)
robot.MoveJ(G_Pre_But2_r, blocking=True)
rdk.pause(1)


robot.MoveJ([-59.520000, -160.560000, -26.210000, -173.220000, -102.860000, -131.790000])
rdk.pause(2)
robot.MoveJ([-51.430000, -112.420000, -102.210000, -145.360000, -90.000000, -130.000000])


# Get ready to pull tab
G_Pre_Pull = matrixMultiply([T_G_B,T_Pull_G,T_Pully_G, T_Arm_A])
G_Pre_Pull_r = rdk.Mat(G_Pre_Pull.tolist())
robot.MoveJ(G_Pre_Pull_r, blocking=True)
rdk.pause(1)

G_Pull = matrixMultiply([T_G_B,T_Pull_G,T_Pull,T_Pully_G, T_Arm_A])
G_Pull_r = rdk.Mat(G_Pull.tolist())
robot.MoveJ(G_Pull_r, blocking=True)
rdk.pause(1)
robot.MoveJ(G_Pre_Pull_r, blocking=True)

robot.MoveJ([-102.860000, -80.360000, -61.070000, -122.140000, 90.000000, 0.000000])
robot.MoveJ([-154.290000, -80.360000, -73.930000, -115.710000, 90.000000, -182.59])
RDK.RunProgram("Grinder Tool Detach (Stand)", True)

# Move close to grinder
robot.MoveJ([-12.860000, -86.790000, -154.290000, -102.860000, -61.070000, 128.570000])

# Move to portafilter in grinder
robot.MoveJ(PF_Grinder_r, blocking=True)
RDK.RunProgram("Portafilter Tool Attach (Grinder)", True)

# Questionable move........
robot.MoveJ([-19.730000, -93.210000, -144.640000, -112.470000, -64.650000, 136.500000])


T_Pre_Scrape = matrixMultiply([T_T_B,T_Scrape_T, Pre_Scrape, T_A_Filt, T_Arm_A])
T_Pre_Scrape_r = rdk.Mat(T_Pre_Scrape.tolist())
robot.MoveJ(T_Pre_Scrape_r, blocking=True)
rdk.pause(1)

T_Scrape = matrixMultiply([T_T_B,T_Scrape_T, Scrape, T_A_Filt, T_Arm_A])
T_Scrape_r = rdk.Mat(T_Scrape.tolist())
robot.MoveJ(T_Scrape_r, blocking=True)
rdk.pause(1)

robot.MoveJ([-7.620000, -86.790000, -141.430000, -129.840000, -127.750000, 145.850000])

T_Pre_Tamp = matrixMultiply([T_T_B,T_Tamp_T,Pre_Tamp,T_A_Filt,T_Arm_A])
T_Pre_Tamp_r = rdk.Mat(T_Pre_Tamp.tolist())
robot.MoveJ(T_Pre_Tamp_r, blocking=True)
rdk.pause(1)


T_Tamp = matrixMultiply([T_T_B,T_Tamp_T,T_A_Filt,T_Arm_A])
T_Tamp_r = rdk.Mat(T_Tamp.tolist())
robot.MoveJ(T_Tamp_r, blocking=True)
rdk.pause(1)
robot.MoveJ(T_Pre_Tamp_r, blocking=True)

robot.MoveJ([-7.620000, -86.790000, -141.430000, -129.840000, -127.750000, 145.850000])

robot.MoveJ([-115.710000, -86.780000, -141.420000, -129.840000, -127.750000, 145.850000])

robot.MoveJ([-154.290000, -125.360000, -115.710000, -38.570000, -176.790000, 212.140000])
T_Standoff = matrixMultiply([T_M_B,T_PF_M,T_A_Filt,T_Arm_A])
T_Standoff_r = rdk.Mat(T_Standoff.tolist())
robot.MoveJ(T_Standoff_r, blocking=True)
rdk.pause(1)

# Not Sure if this is right
RDK.RunProgram("Portafilter Tool Detach (Silvia)", True)
'''
robot.MoveJ([-151.070000, -45.000000, -112.500000, -115.700000, 89.990000, -182.590000])
robot.MoveJ([-154.290000, -80.360000, -73.930000, -115.710000, 90.000000, -182.59])

RDK.RunProgram("Cup Tool Attach (Stand)", True)


RDK.RunProgram("Cup Tool Open", True)
robot.MoveJ([-79.110000, -120.010000, -149.420000, 86.220000, 90.350000, -41.790000])
'''
robot.MoveJ([-102.860000, -80.360000, -61.070000, -122.140000, 90.000000, 0.000000])
robot.MoveJ([-154.290000, -80.360000, -73.930000, -115.710000, 90.000000, -182.59])
#RDK.RunProgram("Cup Tool Detach (Stand)", True)

RDK.RunProgram("Grinder Tool Attach (Stand)", True)

# Press machine Button 1
robot.MoveJ([-163.930000, -64.290000, -112.500000, -35.360000, 186.430000, -166.990000])
T_Pre_B1_M = matrixMultiply([T_M_B,T_B1_M,T_Pre_But,T_Pushy_G,T_Arm_A])
T_Pre_B1_M_r = rdk.Mat(T_Pre_B1_M.tolist())
robot.MoveJ(T_Pre_B1_M_r, blocking=True)
rdk.pause(1)
T_B1_M = matrixMultiply([T_M_B,T_B1_M,T_Pushy_G,T_Arm_A])
T_B1_M_r = rdk.Mat(T_B1_M.tolist())
robot.MoveJ(T_B1_M_r, blocking=True)
rdk.pause(1)
robot.MoveJ(T_Pre_B1_M_r, blocking=True)

# Press machine Button 1
T_Pre_B2_M = matrixMultiply([T_M_B,T_B2_M,T_Pre_But,T_Pushy_G,T_Arm_A])
T_Pre_B2_M_r = rdk.Mat(T_Pre_B2_M.tolist())
robot.MoveJ(T_Pre_B2_M_r, blocking=True)
rdk.pause(1)
T_B2_M = matrixMultiply([T_M_B,T_B2_M,T_Pushy_G,T_Arm_A])
T_B2_M_r = rdk.Mat(T_B2_M.tolist())
robot.MoveJ(T_B2_M_r, blocking=True)
rdk.pause(1)
robot.MoveJ(T_Pre_B2_M_r, blocking=True)

# Press machine Button 1
T_Pre_B3_M = matrixMultiply([T_M_B,T_B3_M,T_Pre_But,T_Pushy_G,T_Arm_A])
T_Pre_B3_M_r = rdk.Mat(T_Pre_B3_M.tolist())
robot.MoveJ(T_Pre_B3_M_r, blocking=True)
rdk.pause(1)
T_B3_M = matrixMultiply([T_M_B,T_B3_M,T_Pushy_G,T_Arm_A])
T_B3_M_r = rdk.Mat(T_B3_M.tolist())
robot.MoveJ(T_B3_M_r, blocking=True)
rdk.pause(1)
robot.MoveJ(T_Pre_B3_M_r, blocking=True)

# Press machine Button 1
T_Pre_B4_M = matrixMultiply([T_M_B,T_B4_M,T_Pre_But,T_Pushy_G,T_Arm_A])
T_Pre_B4_M_r = rdk.Mat(T_Pre_B4_M.tolist())
robot.MoveJ(T_Pre_B4_M_r, blocking=True)
rdk.pause(1)
T_B4_M = matrixMultiply([T_M_B,T_B4_M,T_Pushy_G,T_Arm_A])
T_B4_M_r = rdk.Mat(T_B4_M.tolist())
robot.MoveJ(T_B4_M_r, blocking=True)
rdk.pause(1)
robot.MoveJ(T_Pre_B4_M_r, blocking=True)
