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
from nav_msgs.msg import Odometry




"""
    PID Controller Constants
"""


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



class Navig:
    def __init__(self):
        self.vel = Twist()
        self.pos = [0., 0.]
        rospy.init_node("bebop_node")
        self.pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=0)
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=100)
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=100)
        rospy.Subscriber("/uwbOdom", Odometry, self.odomCallback, queue_size=1)

    def set_para(self, lx, ly, lz, ax, ay, az):
        self.vel.linear.x = lx
        self.vel.linear.y = ly
        self.vel.linear.z = lz
        self.vel.angular.x = ax
        self.vel.angular.y = ay
        self.vel.angular.z = az
        return self.vel


    def hover(self):
        self.set_para(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.pub.publish(self.vel)


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


    def calcPid(self, e, e_prev, ui_prev, deltaTime, consts):

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

    def takeoff(self):
        emp = Empty()
        self.pub_takeoff.publish(emp)
        print ("Took off")

    def land(self):
        emp = Empty()
        self.pub_land.publish(emp)
        print ("landed")

    def odomCallback(self, msg):
        self.pos = [ msg.pose.pose.position.x,msg.pose.pose.position.y ]

    def move(self, dstPos):
        self.maxPos = dstPos
        eX_prev = 1 
        eY_prev = 1
        eX_max = abs(dstPos[0])
        eY_max = abs(dstPos[1])
        uX_prev = 0
        uY_prev = 0
        deltaTime = 1
        mag = 0.
        converged = 0.
        self.takeoff()
        while(True):
            eX = (dstPos[0] - self.pos[0])/eX_max 
            eY = (dstPos[1] - self.pos[1])/eY_max
            rSteer, eX_prev, uX_prev = self.calcPid(
                eX, eX_prev, uX_prev, deltaTime, rollConsts)
            pSteer, eY_prev, uY_prev = self.calcPid(
                eY, eY_prev, uY_prev, deltaTime, pitchConsts)
            eX_prev = eX
            eY_prev = eY

            mag = math.sqrt(eX**2 + eY**2)
            if mag < 0.3 :
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
            print("Position = " + str(self.pos))
            self.vel = self.set_para((pSteer), (-rSteer), 0.0, 0.0, 0.0, 0.0)
            self.pub.publish(self.vel)

            if(converged > 10):
                self.land()
                exit()



def main():
    navig = Navig()
    wpt = [1.,1.]
    navig.move(wpt)


if __name__ == "__main__":
    main()
