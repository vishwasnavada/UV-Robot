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
from datetime import datetime
from Tkinter import *
import json
waypointFile = "square"
wptsfl = open(waypointFile)
folderName = raw_input("Enter the folder_name  ")
fileName = raw_input("Enter the file_name  ")
if not os.path.exists(folderName):
    os.makedirs(folderName)
log = open(folderName + "/" + fileName, 'w')
ser = serial.Serial("/dev/ttyACM0", 576000)
anchorSigmas = [0.000803, 0.000502, 0.000487]
anchorOffsets = [-0.61,-0.58,-0.50] #MODE_SHORT_DATA_FAST_ACCURACY [-0.591, -0.572, -0.72] 

line = []
posArray = []
theta = 0
wptline = []
#position = [5.0,0.0]

#position[0] = float(raw_input("Enter x  "))
#position[1] = float(raw_input("Enter y  "))
master = Tk()

master.title('PID Tuning')
# You can set the geometry attribute to change the root windows size
master.geometry("500x500")  # You want the size of the app to be 500x500
master.resizable(0, 0)  # Don't allow resizing in the x or y direction


rollFrame = LabelFrame(master)
rollFrame.pack(fill="both", expand="yes")

pitchFrame = LabelFrame(master)
pitchFrame.pack(fill="both", expand="yes")


pitchConsts = {}
rollConsts = {}

pitchConsts["h"] = 1.0
pitchConsts["kp"] = 0.065
pitchConsts["ki"] = 0
pitchConsts["kd"] = 0.8 
pitchConsts["u0"] = 0.0
pitchConsts["e0"] = 0.0

rollConsts["h"] = 1.0
rollConsts["kp"] = 0.049
rollConsts["ki"] = 0
rollConsts["kd"] = 0.8 
rollConsts["u0"] = 0.0
rollConsts["e0"] = 0.0

constantsFile = open(folderName + '/constants','w')
constants = {}

def getRollProp(event):
    rollConsts["kp"] = RollProp.get()

def getRollInt(event):
    rollConsts["ki"] = RollInt.get()

def getRollDer(event):
    rollConsts["kd"] = RollDer.get()

def getPitchProp(event):
    pitchConsts["kp"] = PitchProp.get()

def getPitchInt(event):
    pitchConsts["ki"] = PitchInt.get()

def getPitchDer(event):
    pitchConsts["kd"] = PitchDer.get()


RollProp = Scale(rollFrame, from_=0, to=1, orient=HORIZONTAL,
                 command=getRollProp, label="Roll Proportional", resolution=0.001, length=500)
RollProp.set(rollConsts["kp"])
RollProp.pack()

RollInt = Scale(rollFrame, from_=0, to=0.01, orient=HORIZONTAL,
                command=getRollInt, label="Roll Integration", resolution=0.0001, length=1000)
RollInt.set(rollConsts["ki"])
RollInt.pack()

RollDer = Scale(rollFrame, from_=0, to=2, orient=HORIZONTAL,
                command=getRollDer, label="Roll Deriative", resolution=0.001, length=500)
RollDer.set(rollConsts["kd"])
RollDer.pack()


PitchProp = Scale(pitchFrame, from_=0, to=1, orient=HORIZONTAL,
                  command=getPitchProp, label="Pitch Proportional", resolution=0.001, length=500)
PitchProp.set(pitchConsts["kp"])
PitchProp.pack()

PitchInt = Scale(pitchFrame, from_=0, to=0.01, orient=HORIZONTAL,
                 command=getPitchInt, label="Pitch Integration", resolution=0.0001, length=1000)
PitchInt.set(pitchConsts["ki"])
PitchInt.pack()

PitchDer = Scale(pitchFrame, from_=0, to=2, orient=HORIZONTAL,
                 command=getPitchDer, label="Pitch Deriative", resolution=0.001, length=500)
PitchDer.set(pitchConsts["kd"])
PitchDer.pack()


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
    global contants
    global pitchConsts
    global rollConsts
    if a == 2:
        print "terminated"
        emp = Empty()
        # os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        # os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        pub_land.publish(emp)
        print "landed"

        constants["pitchConstants"] =  pitchConsts
        constants["rollConstants"] = rollConsts 
        jsn = json.dumps(constants)
	constantsFile.write(jsn)
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
        ki - integration time constant
        kd - derivative time constant
        u0 - initial integration state
        e0 - initial error
"""


def calcPid(e, e_prev, ui_prev, deltaTime, consts):

    h = consts["h"]
    kp = consts["kp"]
    ki = consts["ki"]
    kd = consts["kd"]
    u0 = consts["u0"]
    e0 = consts["e0"]

    ui = ui_prev + deltaTime*e
    ud = (e-e_prev)/deltaTime
    u = kp*e + ki*ui + kd*ud

    return u, e, ui


rangeNumSamples = 15
alpha = 2.0/(rangeNumSamples + 1.0)


def readdat():
    global position
    k = 0
    now = 0.0
    deltaTime = 1
    t_old = 0.0
    converged = 0 
    print("Starting")
    raw_input("Press any key to start")
    emp = Empty()
    pub_takeoff.publish(emp)
    startTime = time.time()
    while(True):
        signal.signal(signal.SIGINT, signalHandler)
        try:
            wptline = wptsfl.readline() 
            position = [float(i) for i in wptline.split(',')]
            k = 0
            mag = 5
            print(position)
            line = ser.readline()
            line = ser.readline()

            line = ser.readline()
            radialPos = [float(i) for i in line.split(',')]
            prevRadialPos = radialPos
            xMax, yMax = getPos(anchor1Pos, anchor2Pos, anchor3Pos,
                                radialPos[0]+anchorOffsets[0], radialPos[1]+anchorOffsets[1], radialPos[2]+anchorOffsets[2])
            xMax = abs(xMax)
            yMax = abs(yMax)

            eX_prev = 1 
            eY_prev = 1

            eX_max = abs(position[0] - xMax)
            eY_max = abs(position[1] - yMax)

	    print(" exMax = " + str(eX_max) + "eYMax = " + str(eY_max))
            uX_prev = 0
            uY_prev = 0
            master.update()

            while(converged < 200):
                try:
                    master.update()
                    line = ser.readline()
                    radialPos = [float(i) for i in line.split(',')]
                    radialPos = [radialPos[z]*alpha +
                                 prevRadialPos[z]*(1-alpha) for z in range(0, 3)]
                    prevRadialPos = radialPos
                    datx, daty = getPos(anchor1Pos, anchor2Pos, anchor3Pos,
                                        radialPos[0]+anchorOffsets[0], radialPos[1]+anchorOffsets[1], radialPos[2]+anchorOffsets[2])
                    print("Current Position = " + str(datx) + ',' + str(daty))
                    eX = (position[0] - datx)/eX_max  #Distance
                    eY = (position[1] - daty)/eY_max
		    print("Error in x  " + str(eX)  + "Error in y  "  + str(eY))
#                    now = datetime.now().microsecond
#                    deltaTime = now - t_old
#                    t_old = now
#
                    log.write( str(time.time()-startTime) + ',' + str(datx) + ',' + str(daty) + '\n')

                    rSteer, eX_prev, uX_prev = calcPid(
                        eX, eX_prev, uX_prev, deltaTime, rollConsts)
                    pSteer, eY_prev, uY_prev = calcPid(
                        eY, eY_prev, uY_prev, deltaTime, pitchConsts)

                    eX_prev = eX
                    eY_prev = eY

                    print("Pitch Steer "+str(pSteer))
                    print("Roll Steer "+str(rSteer))
                    mag = math.sqrt(eX**2 + eY**2)
                    if mag < 0.15 :
                        converged += 1
                    #Motion constraints
                    if(pSteer > 0.5):
                        pSteer = 0.5
                    if(pSteer < -0.5):
                        pSteer = -0.5
                    if(rSteer > 0.5):
                        rSteer = 0.5
                    if(pSteer < -0.5):
                        rSteer = -0.5
		    print("X = " + str(position[0]))
		    print("Y = " + str(position[1]))
                    # print("Steer Pitch" + str(pSteer))
                    # print("Steer Roll" + str(rSteer))
                    set_para((pSteer), (-rSteer), 0.0, 0.0, 0.0, 0.0)
                    pub.publish(vel)
                except Exception as e:
                    exc_type, exc_obj, exc_tb = sys.exc_info()
                    fname = os.path.split(
                        exc_tb.tb_frame.f_code.co_filename)[1]
                    print(exc_type, fname, exc_tb.tb_lineno)
                    print e

	    converged = 0
            print("Popped")
        except ValueError:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)


if __name__ == '__main__':

    pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=0)
    pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=100)
    pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=100)
    node_init()
