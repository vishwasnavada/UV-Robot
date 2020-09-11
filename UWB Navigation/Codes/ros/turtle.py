#!/usr/bin/python2


import rospy
from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from uwb_node.msg import uwbmsg
import time
import numpy as np
import sys
import os
import signal
from scipy.optimize import optimize
import numpy as np


def sigHandle(sig, frame):
    sys.exit(0)


class bot():
    def __init__(self, ac, offsets, uwbCov):
        self.ac = ac
        self.offsets = offsets
        self.uwbdata = np.array([0., 0., 0., 0.])
        self.gtdata = np.array([0., 0., 0.])
        self.dataRec = False
        self.pos = np.array([0., 0., 0.])
        self.arr = np.array([0., 0., 0.])
        self.cov = uwbCov
        self.cov = tuple(self.cov.ravel().tolist())
        self.uwbPub = rospy.Publisher("/uwbtfd", Odometry, queue_size=10)

    def gt_callback(self, data):
        self.gtdata = np.array(
            [data.pose.position.x, data.pose.position.y, data.pose.position.z])

    def uwb_callback(self, data):
        self.uwbdata = np.array(data.curr_pos) + self.offsets
        self.dataRec = True

    def f_opt(self, pos, radii):
        dist = 0.
        for i in range(0, 4):
            dist += np.square(np.linalg.norm(pos-self.ac[i]) - radii[i])
        return dist

    def convMsg(self, pos):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = self.pos[0]
        msg.pose.pose.position.y = self.pos[1]
        msg.pose.pose.position.z = self.pos[2]
        return msg

    def run(self):
        mag = 0.
        while(True):
            if(self.dataRec == True):
                self.dataRec = False
                self.pos = optimize.fmin_powell(self.f_opt, self.pos, args=(
                    self.uwbdata,), xtol=0.0001, ftol=0.0001, disp=0)
                mag = np.linalg.norm(self.pos - self.gtdata)
                print(str(self.pos) + "\t" + str(self.gtdata) + "\t" + str(mag))
                self.uwbPub.publish(self.convMsg(self.pos))
                #self.arr = np.vstack([self.arr, self.pos])


def main():

    global turtle
    # List of anchors
    ac = np.zeros((4, 3))
    ac[0] = np.array([-2.68, 1.8, 1.81])  # x, height, y
    ac[1] = np.array([-2.7, 1.63, -1.5])
    ac[2] = np.array([2.54, 1.81, -1.3])
    ac[3] = np.array([2.42, 1.58, 1.62])

    offsets = np.array([-0.66171298, -0.76921925, -0.70283083, -0.91615231])

    uwbCov = np.array([[0.00382429, -0.00285907,  0.00308421], [-0.00285907,
                                                                0.00264731, -0.00242654], [0.00308421, -0.00242654,  0.0026886]])
    turtle = bot(ac, offsets, uwbCov)

    rospy.init_node("uwb_parser")
    rospy.Subscriber("/vrpn_client_node/bot/pose",
                     PoseStamped, turtle.gt_callback)
    rospy.Subscriber("/uwb", uwbmsg, turtle.uwb_callback)

    turtle.run()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigHandle)
    main()
