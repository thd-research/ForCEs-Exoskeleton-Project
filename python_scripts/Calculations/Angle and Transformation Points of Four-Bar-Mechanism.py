""" Transformation Points of Four-Bar-Mechanism and Knee EXOSKELETON """

""" Refer to 'four bar linkage and freudenstein.pdf' or 'four bar linkage and freudenstein.dwg' to understand variable nomination"""
#start from P1 [0,0] to P6 end point
#points described in 2D

import numpy as np
from math import *

np.set_printoptions(suppress = True)

#link values:

L1 = 17.45
L2 = 35.75
L4 = 46.15
L3 = 41.89

#shank length
L5 = 30.00

t1 = 40.23 
t2 = 20
a = 103.15 
b = 180. - a
t4 = 100.83

trig_t1 = -t1
trig_t2 = -t2

global tshank
global tshank_hor
global tshank_ver


#function to determine the rotation angle theta3 (of coupler)
def shank_angle(t2):
    k1 = L1/L2
    k2 = L1/L3
    k3 = ((L4*L4)-(L1*L1)-(L2*L2)-(L3*L3))/(2*L2*L3)
    E = -2*sin(radians(t2))
    D = k2*cos(radians(t2)) + cos(radians(t2)) + k3 - k1
    F = k2*cos(radians(t2)) - cos(radians(t2)) + k3 + k1
    #angle between coupler link and link 1 (L1 and L3)
    t = 2 * degrees(atan((-E + sqrt((E*E)-(4*D*F)))/(2*D)))
  
    
    if t < 0:
        t = t + 360

    

    return t

#to determine basic translation of vectors
#arguments: start point: s and translation vector: vx, vy
#returns end vector
def trans(s,vx,vy):
    tv = np.array([[vx],
                   [vy]])
    ev = np.add(s,tv)

    return ev
    

#function to determine the end coordinates from the trigonometric pythagorean principles
#arguments: l: hypotenuse, t: angle at adjacent, p: start coordinates, i: inverse
#for inversing x and y axes, set i to 1, else i to 0
def trig_pyth(p,l,t,i):
    a = -1*i*pi/2
    p_t = np.array([[l * cos(radians(t))], #x
                    [l * sin(radians(t))]])#y
    rot = np.array([[cos(a),-sin(a)],
               [sin(a),cos(a)]])
    
    p_t = rot.dot(p_t)
    
    p_end = np.add(p,p_t)

    return p_end

#arguments: start point s, end point e of line
#equation is generated through cross section in the form: ax + by + c = 0
#param arguments return these:
def line(s,e):
    s = np.append(s,[[1]],axis=0)
    e = np.append(e,[[1]],axis=0)
    eq = np.cross(s,e,axis=0)

    return eq
""" 
    eq = np.array([[a],
                     [b],
                    [c]]) """

#determine the intersection points of two lines denoted by four points:
#arguments: s1:start first, e1:end first, s2:start second, e2:end second
def intersect(s1,e1,s2,e2):
    eq1 = line(s1,e1)
    eq2 = line(s2,e2)

    prod = np.cross(eq1,eq2,axis=0)
    intersection = np.array([[prod[0,0]/prod[2,0]],
                              [prod[1,0]/prod[2,0]]])
    #as long as links not parallel:                       
    if prod[2,0]!=0:
        return intersection

#extra calculations from t:
t = shank_angle(t2)

tcoupler = - (t + abs(t1))

tcoupler_hor = t + t1
tshank_hor = tcoupler_hor - b

tcoupler_ver = t + t1 - 90
tshank_ver = tcoupler_ver - b

if t < 0:
    tshank_ver = tshank_ver + 360
    
#point P1: (Base coordinates in actuator)
p1 = np.array([[0],
              [0]])

#points p2, p3, p4, p5 and p6:
p2 = trig_pyth(p1,L1,trig_t1,0)
p4 = trig_pyth(p1,L2,trig_t1+trig_t2,0)
p3 = trig_pyth(p4,L3,tcoupler,0)
p5 = np.add(p3,p4)/2
#p6 = trans(p5,0,-L5)
p6 = trig_pyth(p5, L5, -tshank_ver,1)

print('input angle:')
print(t2)
print('shank output angle to horizontal:')
print(tshank_hor)
               
print('p1:')
print(p1)
print('p2:')
print(p2)
print('p4:')
print(p4)
print('p3:')
print(p3)
print('p5:')
print(p5)
print('p6:')
print(p6)


#ICR is the intersection of links L4 and L2:

icr = intersect(p1,p4,p2,p3)
print('icr:')
print(icr)


