#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 28 11:39:53 2020

@author: maxharrison
"""

import numpy as np
import math

def origin():
    return np.array([0,0,0])

def unit_x():
    return np.array([1,0,0])

def unit_y():
    return np.array([0,1,0])

def unit_z():
    return np.array([0,0,1])

def normalise(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def matrixMultiply(matrices):
    N = len(matrices)
    mat = matrices[0]
    for i in range(N-1):
        mat = np.matmul(mat,matrices[i+1])
    return mat
        

def calcAngle(a_p1,a_p2,b_p1,b_p2):
    # Calculates angle from vector a to vector b
    # each vector is defines as starting as p1 and ending at p2
    A = normalise(a_p2-a_p1)
    B = normalise(b_p2-b_p1)
    
    theta = math.atan2(A[1],A[0]) - math.atan2(B[1],B[0])
    theta = (theta + np.pi) % (2 * np.pi) - np.pi
    
    return theta

def calcRotMat(Ax,Ay,Az,Bx,By,Bz):
    R = np.array([[np.dot(Bx,Ax), np.dot(By,Ax), np.dot(Bz,Ax)],
                  [np.dot(Bx,Ay), np.dot(By,Ay), np.dot(Bz,Ay)],
                  [np.dot(Bx,Az), np.dot(By,Az), np.dot(Bz,Az)]])
    return R
    
def makeTransMat(R,P):
    
    a = np.concatenate((R,P.reshape(3,1)),axis=1)
    b = np.array([0,0,0,1]).reshape(1,4)
    T = np.concatenate((a,b))
    
    return T

def getT_G_B():
    # Grinder axis
    pos = np.array([484.51,-426.6,318.38])
    a = np.array([484.51,-426.6,0])  
    b = np.array([369.74,-320.06,0])                
    z = np.array([0,0,1])
    
    x = normalise(b - a)
    y = np.cross(z,x)
    
    R = calcRotMat(unit_x(),unit_y(),unit_z(),x,y,z)
    
    T = makeTransMat(R,pos)
    
    return T

def getT_T_B():
    # Tamper tool axes
    pos = np.array([598.10,4.31,212.58])
    a = np.array([557.5,73.18,0])
    b = np.array([598.10,4.31,0])
    z = np.array([0,0,1])
    
    x = normalise(b-a)
    y = np.cross(z,x)
    
    R = calcRotMat(unit_x(),unit_y(),unit_z(),x,y,z)
    
    T = makeTransMat(R,pos)
    
    return T

def getT_M_B():
    # Coffee Machine axes
    pos = np.array([-359.9,-387.38,341.24])
    a = np.array([-359.9,-387.38,0])
    b = np.array([-573.54,-443.66,0])
    z = np.array([0,0,1])
    
    y = normalise(b - a)
    x = np.cross(y,z)
    R = calcRotMat(unit_x(),unit_y(),unit_z(),x,y,z)
    T = makeTransMat(R,pos)
    
    return T

def getT_C_B():
    # Grinder axis
    pos = np.array([1.92,-595.89,-20])
    a = np.array([1.92,-595.89,0])  
    b = np.array([0,-595.89,0])                
    z = np.array([0,0,1])
    
    y = normalise(b - a)
    x = np.cross(y,z)
    
    R = calcRotMat(unit_x(),unit_y(),unit_z(),x,y,z)
    
    T = makeTransMat(R,pos)
    
    return T

def getT_B1_M(): # Oriented to align with pointer frame
    P = np.array([50.67,35.25,-27.89])
    R = np.array([[0,0,-1],
                  [0,1,0],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_B2_M(): # Oriented to align with pointer frame
    P = np.array([50.67,35.25,-61.39])
    R = np.array([[0,0,-1],
                  [0,1,0],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_B3_M(): # Oriented to align with pointer frame
    P = np.array([50.67,35.25,-94.89])
    R = np.array([[0,0,-1],
                  [0,1,0],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_B4_M(): # Oriented to align with pointer frame
    P = np.array([50.67,98.75,-27.89])
    R = np.array([[0,0,-1],
                  [0,1,0],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_PF_M(): # Oriented to align with grinder frame
    P = np.array([-12.68,72,-290])
    R = np.array([[0,0,-1],
                  [0,1,0],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_PF_G(): # PF2 in Grinder frame
    P = np.array([157.61,0,-250.45])
    R = np.array([[0,0,-1],
                  [0,1,0],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_B1_G(): # Oriented to align with pointer frame
    P = np.array([-64.42,89.82,-227.68])
    R = np.array([[0,-1,0],
                  [0,0,-1],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_B2_G(): # Oriented to align with pointer frame
    P = np.array([-80.71,94.26,-227.68])
    R = np.array([[0,-1,0],
                  [0,0,-1],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_Pull_G(): # Oriented to align with correct puller frame
    P = np.array([-35.82,83.8,-153])
    R = np.array([[0,0,-1],
                  [1,0,0],
                  [0,-1,0]])
    return makeTransMat(R,P)

def getT_Tamp_T():
    P = np.array([-80,0,-55])
    R = np.array([[0,1,0],
                  [0,0,1],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_Scrape_T():
    P = np.array([70,0,-32])
    R = np.array([[0,1,0],
                  [0,0,1],
                  [1,0,0]])
    return makeTransMat(R,P)

def getT_Filt_A():
    P = np.array([4.71,0,144.76])
    R = np.array([[np.cos(7.5*np.pi/180),0,-np.sin(7.5*np.pi/180)],
                  [0,1,0],
                  [np.sin(7.5*np.pi/180),0,np.cos(7.5*np.pi/180)]])
    return makeTransMat(R,P)

def getT_A_Arm():
    P = np.array([0,0,0])
    R = np.array([[np.cos(50*np.pi/180),np.sin(50*np.pi/180),0],
                  [-np.sin(50*np.pi/180),np.cos(50*np.pi/180),0],
                  [0,0,1]])
    return makeTransMat(R,P)

def getT_Tamp_T_prepos():
    P = np.array([-80,-100,-100])
    R = np.array([[0,1,0],
                  [0,0,1],
                  [1,0,0]])
    return makeTransMat(R,P)


def getT_Pushy_G(): # Pushy bit in grinder tool
    P = np.array([0,0,102.82])
    R = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]])
    return makeTransMat(R,P)

def getT_Pully_G(): # Pully bit in grinder tool
    P = np.array([-50,0,67.06])
    R = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]])
    return makeTransMat(R,P)

def getT_Cup_C(): # Part on cup stack in cup
    P = np.array([-36.08,0,147.25])
    R = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]])
    return makeTransMat(R,P)

def getT_Centre_C(): # centre of cup tool
    P = np.array([-47.0,0,186.11])
    R = np.array([[1,0,0],
                  [0,1,0],
                  [0,0,1]])
    return makeTransMat(R,P)

#def getT_Scrape_T
    
    

def main():
    # Get transfer matrices
    T_Tamp_Base = getT_T_B()        # Tamper tool
    T_Grind_Base = getT_G_B()       # Grinder tool
    T_Mach_Base = getT_M_B()        # Coffee Machine
    
    

    print(T_T_G)
    print(T_C_B)

#main()

    
    