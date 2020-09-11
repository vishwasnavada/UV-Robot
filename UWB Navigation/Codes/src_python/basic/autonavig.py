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
from scipy.ndimage.filters import gaussian_filter1d

x = 3
y = 6

ser = serial.Serial("/dev/ttyUSB0", 9600)


anchorSigmas = [0.000803, 0.000502, 0.000639]
anchorOffsets = [-0.61, -0.591, -0.703]


horVel = 0.1
rotVel = 0.05
verVel = 0.05


SET_RFLAG = True
SET_PFLAG = True

line = []
posArray = []
theta = 0

xmaxvariance = 0.5
xminvariance = 0.5

ymaxvariance = 0.5
yminvariance = 0.5


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


def roll(datx, fx):

    global SET_RFLAG
    print "rolling"
    if ((datx < x) and (datx not in fx)):
        # if datx > fx:
        set_para(0.0, -1*horVel, 0.0, 0.0, 0.0, 0.0)  # roll-left
        pub.publish(vel)

    elif ((datx > x) and (datx not in fx)):
        set_para(0.0, 1*horVel, 0.0, 0.0, 0.0, 0.0)   # roll-right
        pub.publish(vel)
    else:
        SET_RFLAG = False
        hover()


def pitch(daty, fy):

    global SET_PFLAG

    print "pitching"
    if ((daty < y) and (daty not in fy)):
        set_para(1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0)
        pub.publish(vel)
        print "forward"

    elif ((daty > y) and (daty not in fy)):
        set_para(-1*horVel, 0.0, 0.0, 0.0, 0.0, 0.0)
        pub.publish(vel)
        print "back"

    else:
        SET_PFLAG = False
        hover()
        print "hover state"


def readdat():

    global SET_PFLAG
    global SET_RFLAG

    radialPos = [0.0, 0.0, 0.0]
    while(True):
        signal.signal(signal.SIGINT, signalHandler)
        for points in range(0, 12):
            try:
                line = ser.readline()
                posArr.append([float(i) for i in line.split(',')])
            except Exception as e:
                print(e)
        radialPos = [gaussian_filter1d(posArr[:, i], anchorSigmas[i]) + anchorOffsets[i] for i in range(0, 3)]
        possArr.clear()
        datx, daty = getPos(anchor1Pos, anchor2Pos, anchor3Pos,
                            radialPos[0], radialPos[1], radialPos[2])
        X = datx-x
        Y = daty-y
        mag = math.sqrt(X**2 + Y**2)
        pSteer = X/mag 
        rSteer = Y/mag
        set_para(psteer,rsteer,0.0,0.0,0.0,0.0,0.0)
        if(mag < 0.5):
            print("Landed !!!!!!!!!!!!!!!!!!!!!!!!")
            emp = Empty()
            pub_land.publish(emp)
            sys.exit(0)


if __name__ == '__main__':
    pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=0)
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=100)
    node_init()
