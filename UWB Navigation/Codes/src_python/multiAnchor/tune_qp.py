#/usr/bin/env python
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

x_limit = y_limit = z_limit = 5.0
home_x = home_y = home_z = 0
kp = 1.0
kb = 1000.0

constantsFile = open(folderName + '/constants','w')
constants = {}


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


def quadprog_solve_qp(H, h, A=None, b=None, C=None, d=None):

    qp_H = .5 * (H + H.T)  # make sure H is symmetric
    qp_h = -h
    if C is not None:
        qp_C = -numpy.vstack([C, A]).T
        qp_d = -numpy.hstack([d, b])
        meq = C.shape[0]
    else:  # no equality constraint
        qp_C = -A.T
        qp_d = -b
        meq = 0

    return quadprog.solve_qp(qp_H, qp_h, qp_C, qp_d, meq)[0]



def qp_q_dot_des(q_act, q_des, q_origin, q_limit, q_kp, q_kb):
    # q_des = 0.

    # cost function matrix is given here   e.g. u^T H u
    H = array([[1000., 0.], [0., 1.]])
    h = array([0., 0.])  # cost function vector    e.g. h^T u

    # stability constraints
    kp = q_kp
    Va = q_act - q_des
    Vb = -kp * (q_act - q_des) * (q_act - q_des)

    # # safety constraints
    limit = q_limit  # in kms for position and radians for angles - very high
    q_rel = q_act - q_origin
    Ba = - 2. * q_rel  # derivative of angle_limit - x^2
    Bb = -q_kb * (limit * limit - q_rel * q_rel)  # - (angle_limit - x^2)

    # inequality constraints are given here Au \leq b
    A = array([[-1, Va], [0, -Ba]])
    b = array([Vb, -Bb])
    # A = array([[-1, Va]])
    # b = array([Vb])

    u_in = quadprog_solve_qp(H, h, A, b)

    return array([u_in[1]])




def readdat():

    global position
    global kp, kb

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
            k = 0
            mag = 5
            print(position)
            line = ser.readline()
            line = ser.readline()

            line = ser.readline()
            radialPos = [float(i) for i in line.split(',')]
            prevRadialPos = radialPos
            home_x, home_y = getPos(anchor1Pos, anchor2Pos, anchor3Pos,
                                radialPos[0]+anchorOffsets[0], radialPos[1]+anchorOffsets[1], radialPos[2]+anchorOffsets[2])
            home_x = abs(home_x)
            home_y = abs(home_y)

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
                print("Error in x  " + str(eX)  + "Error in y  "  + str(eY))
#                    now = datetime.now().microsecond
#                    deltaTime = now - t_old
#                    t_old = now
#
                log.write( str(time.time()-startTime) + ',' + str(datx) + ',' + str(daty) + '\n')

                current_time = rospy.get_rostime()
                rSteer = qp_q_dot_des(datx, position[0], home_x, x_limit, kp, kb)
                pSteer = qp_q_dot_des(daty, position[1], home_y, y_limit, kp, kb)

                print("Pitch Steer "+str(pSteer))
                print("Roll Steer "+str(rSteer))
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
