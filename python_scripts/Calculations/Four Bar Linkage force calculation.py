"""MODULES"""
import numpy as np
from math import * #no prefixing
import matplotlib.pyplot as plt
import sys

def format_float(num):
    return np.format_float_positional(num,trim='-')

"""FUNCTIONS"""


"""CONSTANTS"""

#Links lengths: (in m)
L1 = 17.4544/1000
L2 = 35.5/1000
L3 = 41.5415/1000
L4 = 46.1494/1000
#L3 = 46.1494/1000
#L4 = 41.5415/1000


#angles in degrees:
t1 = 40 #theta1
t2 = 20 #theta2
t3 = 127 #theta3
t4 = 101 #theta4

a = 103 #alpha

t = t3 - 180 + a - 90
t = abs(t)

Rpx = 0.15 #m

C = Rpx


#T_input = 36.3 #input torque in Nm

#Enter any input torque value (No limits)
T_input = float(input('Enter any Torque measured at Crank (Nm): '))
T_input = -T_input

x = 0

while x<20 or x>70:
    x = int(input('Enter the crank angle between 20 and 70 deg: '))



"""EQUATIONS"""

R12x = (L2/2)*cos(radians(t2)+pi)
R12y = (L2/2)*sin(radians(t2)+pi)

R32x = (L2/2)*cos(radians(t2))
R32y = (L2/2)*sin(radians(t2))

R23x = (L3/2)*cos(radians(t3))
R23y = (L3/2)*sin(radians(t3))

R43x = (L3/2)*cos(radians(t3)+pi)
R43y = (L3/2)*sin(radians(t3)+pi)

R34x = (L4/2)*cos(radians(t4))
R34y = (L4/2)*sin(radians(t4))

R14x = (L4/2)*cos(radians(t4)+pi)
R14y = (L4/2)*sin(radians(t4)+pi)


"""MATRICES"""

A = np.array([[1,0,1,0,0,0,0,0,0],
              [0,1,0,1,0,0,0,0,0],
              [-R12y,R12x,-R32y,R32x,0,0,0,0,0],
              [0,0,-1,0,1,0,0,0,cos(radians(t))],
              [0,0,0,-1,0,1,0,0,sin(radians(t))],
              [0,0,R23y,-R23x,-R43y,R43x,0,0,C],
              [0,0,0,0,-1,0,1,0,0],
              [0,0,0,0,0,-1,0,1,0],
              [0,0,0,0,R34y,-R34x,-R14y,R14x,0]])


A_inverse = np.linalg.inv(A) #inverse

#print(A)
#print(A_inverse)

constant_matrix = np.array([[0],
                           [0],
                           [T_input],
                           [0],
                           [0],
                           [0],
                           [0],
                           [0],
                           [0]])
                           

Forces = np.dot(A_inverse,constant_matrix)
#print("Forces: ")
#print(Forces)

#Fshank  = abs(Forces[8,0])
Fshank  = -Forces[8,0]

#print("Force acting on shank at ",Rpx," m from L3: ")
#print(Fshank)

#print("Change of Fshank with angle of rotation of crank:")

n = np.array([])
m = np.array([])


t2 = x

R12x = (L2/2)*cos(radians(t2)+pi)
R12y = (L2/2)*sin(radians(t2)+pi)

R32x = (L2/2)*cos(radians(t2))
R32y = (L2/2)*sin(radians(t2))

R23x = (L3/2)*cos(radians(t3))
R23y = (L3/2)*sin(radians(t3))

R43x = (L3/2)*cos(radians(t3)+pi)
R43y = (L3/2)*sin(radians(t3)+pi)

R34x = (L4/2)*cos(radians(t4))
R34y = (L4/2)*sin(radians(t4))

R14x = (L4/2)*cos(radians(t4)+pi)
R14y = (L4/2)*sin(radians(t4)+pi)

A = np.array([[1,0,1,0,0,0,0,0,0],
            [0,1,0,1,0,0,0,0,0],
            [-R12y,R12x,-R32y,R32x,0,0,0,0,0],
            [0,0,-1,0,1,0,0,0,cos(radians(t))],
            [0,0,0,-1,0,1,0,0,sin(radians(t))],
            [0,0,R23y,-R23x,-R43y,R43x,0,0,C],
            [0,0,0,0,-1,0,1,0,0],
            [0,0,0,0,0,-1,0,1,0],
            [0,0,0,0,R34y,-R34x,-R14y,R14x,0]])
A_inverse = np.linalg.inv(A) #inverse
constant_matrix = np.array([[0],
                        [0],
                        [T_input],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0],
                        [0]])
Forces = np.dot(A_inverse,constant_matrix)
Fshank  = abs(Forces[8,0])

    
torque_shank = Fshank*0.17
n = np.append(n,x)
m = np.append(m,torque_shank)
print("x: ",x,"--Fshank: ",Fshank)
#if torque_shank > T_input:
 #   break
    
print('At '+str(x)+', deg the Output Shank Torque is: '+str(torque_shank)+' Nm with Input crank Torque: '+str(T_input)+' Nm')    


#print(n)
#print(m)

#plt.plot(n,m, 'r--')
#plt.show()
