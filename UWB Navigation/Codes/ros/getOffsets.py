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
import sys, os, signal



def sigHandle(sig, frame):
    print("Exiting")
    sys.exit(0)





class bot():

    def __init__(self, ac):
        self.ac = ac
        self.uwbdata = np.array([0.,0.,0.,0.])
        self.gtdata = np.array([0.,0.,0.])
        self.offsets = np.array([0.,0.,0.,0.])
        self.dataRec = False


    def gt_callback(self, data):
        self.gtdata = np.array([data.pose.position.x,data.pose.position.y,data.pose.position.z ])

    def uwb_callback(self, data):
        self.uwbdata = np.array(data.curr_pos)
        self.dataRec = True

    def getOffsets(self):
        num = 0
        while(True):
            if(self.dataRec == True):
                self.dataRec = False
                for i in range (0,4):
                    self.offsets[i] += np.linalg.norm(self.ac[i] - self.gtdata) - self.uwbdata[i]
                num += 1
                print(str(self.offsets) + "\t" + str(num))


def main():

    # List of anchors
    ac = np.zeros((4,3))
    ac[0] = np.array([-2.68,1.81,1.8]) # x, height, y
    ac[1] = np.array([-2.7,1.63,-1.507])
    ac[2] = np.array([2.54,1.82,-1.3])
    ac[3] = np.array([2.424,1.58,1.621])

    turtle = bot(ac)

    rospy.init_node("uwb_parser")
    rospy.Subscriber("/vrpn_client_node/bot/pose", PoseStamped, turtle.gt_callback)
    rospy.Subscriber("/uwb", uwbmsg, turtle.uwb_callback)

    turtle.getOffsets()


    while(True):
        pass





if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigHandle)
    main()
