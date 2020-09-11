#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist
import sys
import signal
from std_msgs.msg import Empty
import numpy as np
import serial
from std_msgs.msg import Empty
import os
import math


log = open("recorded_path.csv", 'w')
#wpts = open("rect1_0_to_5_0_to_5_6_to_0_6.csv", 'r')
ser = serial.Serial("/dev/ttyUSB0", 9600)

anchorSigmas = [0.000803, 0.000502, 0.000639]
anchorOffsets = [-0.61, -0.591, -0.703]


line = []
posArray = []
theta = 0
wptline = []
position = []


"""
    PID Controller Constants
"""
Yconsts = {}
Xconsts = {}

Yconsts["h"] = 1.0
Yconsts["kp"] = 0.01
Yconsts["Ti"] = 1.0
Yconsts["Td"] = 1.0
Yconsts["u0"] = 0.0
Yconsts["e0"] = 0.0

Xconsts["h"] = 1.0
Xconsts["kp"] = 0.01
Xconsts["Ti"] = 1.0
Xconsts["Td"] = 1.0
Xconsts["u0"] = 0.0
Xconsts["e0"] = 0.0

anchor1Pos = (0.0, 0.0)
anchor2Pos = (0.0, 6.0)
anchor3Pos = (6.0, 6.0)

ax = [min(anchor1Pos[0], anchor2Pos[0], anchor3Pos[0])-2.0, max(anchor1Pos[0], anchor2Pos[0], anchor3Pos[0])+2.0,
      min(anchor1Pos[1], anchor2Pos[1], anchor3Pos[1])-2.0, max(anchor1Pos[1], anchor2Pos[1], anchor3Pos[1])+2.0]

# Assume origin is p1


def getPos(p1, p2, p3, r1, r2, r3):

    A = -2*p1[0] + 2*p2[0]
    B = -2*p1[1] + 2*p2[1]
    C = r1**2 - r2**2 - p1[0]**2 + p2[0]**2 - p1[1]**2 + p2[1]**2

    D = -2*p2[0] + 2*p3[0]
    E = -2*p2[1] + 2*p3[1]
    F = r2**2 - r3**2 - p2[0]**2 + p3[0]**2 - p2[1]**2 + p3[1]**2

    pos = (round((C*E - F*B)/(E*A - B*D), 2),
           round((C*D - A*F)/(B*D - A*E), 2))
    return pos


vel = Twist()


def signalHandler(a, b):
    if a == 2:
        print "terminated"
        emp = Empty()
        # os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        # os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        pub_land.publish(emp)
        print "landed"
        sys.exit(0)


def node_init():
    rospy.init_node("bebop_node")
    readdat()


def set_para(lx, ly, lz, ax, ay, az):
    vel.linear.x = lx
    vel.linear.y = ly
    vel.linear.z = lz
    vel.angular.x = ax
    vel.angular.y = ay
    vel.angular.z = az


def hover():
    set_para(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    pub.publish(vel)


"""
Pid output calculating function
    e - error between measured and desired poistions
    k - current time
    Constants :- 
        h - time step 
        kp - proporitional error
        Ti - integration time constant
        Td - derivative time constant
        u0 - initial integration state
        e0 - initial error
"""


def calcPid(e, e_prev, ui_prev, k, consts):

    h = consts["h"]
    kp = consts["kp"]
    Ti = consts["Ti"]
    Td = consts["Td"]
    u0 = consts["u0"]
    e0 = consts["e0"]

    ui = ui_prev + 1/Ti * h*e
    ud = 1/Td * (e-e_prev)/h
    u = kp * (e + ui + ud)

    return u, e, ui


rangeNumSamples = 15
alpha = 2.0/(rangeNumSamples + 1.0)


def readdat():
    k = 0
    while(True):
        signal.signal(signal.SIGINT, signalHandler)
        try:
            #wptline = wpts.readline()
            #position = [float(i) for i in wptline.split(',')]
            k = 0
            mag = 5
            position = [5.0, 1.0]
            #print(position)

            line = ser.readline()
            radialPos = [float(i) for i in line.split(',')]
            prevRadialPos = radialPos
            xMax, yMax = getPos(anchor1Pos, anchor2Pos, anchor3Pos,
                                radialPos[0]+anchorOffsets[0], radialPos[1]+anchorOffsets[1], radialPos[2]+anchorOffsets[2])
            eX_prev = position[0] - xMax
            eY_prev = position[1] - yMax
            uX_prev = 0.0
            uY_prev = 0.0
            while(mag > 0.5):
                try:
                    line = ser.readline()
                    radialPos = [float(i) for i in line.split(',')]
                    radialPos = [radialPos[z]*alpha +
                                 prevRadialPos[z]*(1-alpha) for z in range(0, 3)]
                    prevRadialPos = radialPos
                    datx, daty = getPos(anchor1Pos, anchor2Pos, anchor3Pos,
                                        radialPos[0]+anchorOffsets[0], radialPos[1]+anchorOffsets[1], radialPos[2]+anchorOffsets[2])
                    log.write(str(datx) + ',' + str(daty) + '\n')
                    print("Current Position = " + str(datx) + ',' + str(daty))
                    eX = position[0] - datx
                    eY = position[1] - daty
                    pSteer, eX_prev, uX_prev = calcPid(
                        eX, eX_prev, uX_prev, k, Xconsts)
                    rSteer, eY_prev, uY_prev = calcPid(
                        eY, eY_prev, uY_prev, k, Yconsts)
                    k = k+1
                    # print X,Y
                    pSteer = -pSteer/abs(yMax)
                    rSteer = rSteer/abs(xMax)
                    print("Pitch Steer "+str(pSteer))
                    print("Roll Steer "+str(rSteer))
                    mag = math.sqrt(eX**2 + eY**2)
                    # print("Steer Pitch" + str(pSteer))
                    # print("Steer Roll" + str(rSteer))
                    set_para((pSteer), (rSteer), 0.0, 0.0, 0.0, 0.0)
                    pub.publish(vel)
                except Exception as e:
                    exc_type, exc_obj, exc_tb = sys.exc_info()
                    fname = os.path.split(
                        exc_tb.tb_frame.f_code.co_filename)[1]
                    print(exc_type, fname, exc_tb.tb_lineno)
                    print e
            # emp = Empty()
            # pub_land.publish(emp)
            print("Popped")
        except ValueError:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)


if __name__ == '__main__':
    pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=0)
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=100)
    node_init()
