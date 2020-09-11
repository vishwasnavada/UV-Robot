'''
    UWB + Optical flow based localization
    Requires that the uwb message is compiled and sourced
'''

import rospy
from nav_msgs.msg import Odometry
import math
from uwbsingle.msg import singleUWB
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Empty
from scipy.optimize import optimize
import signal, sys
import matplotlib
import matplotlib.pyplot as plt
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
import scipy.optimize as optimize

np.set_printoptions(suppress=True)



''' Kill signal handler '''
def signalHandler(a, b):
    if a == 2:
        sys.exit(0)


''' Odometry class, performs computation of odometry '''
class Od:
    def __init__(self, anchorPos, xTheta, uwbBufferLen):

        ''' UWB Radio specific variable '''
        self.uwbOffset = -0.7

        ''' Anchor position variables '''
        self.anchorPos = anchorPos
        self.xTheta = xTheta 
        self.yTheta = xTheta + 90

        ''' estimatation buffer variables '''
        self.buffIdx = 0
        self.uwbBufferLen = uwbBufferLen
        self.radiiBuffer = np.zeros((self.uwbBufferLen, 4), dtype=np.float32)


        ''' Temporary Varialbles '''

        ''' Angle between drone and anchor's x axis '''
        self.psi = 0.

        ''' Bebop sensor variables '''
        self.quat = [0., 0., 0., 0.]
        self.euler = 0.

        ''' Bebop Flags '''
        self.odomFlag = False
        self.uwbFlag = False
        self.estPosFlag = False
        self.takeOffFlag = False
        self.landFlag = False



        ''' UWB and  Optical flow variables for estimation'''
        self.delX = 0. 
        self.delY = 0.
        self.delZ = 0.
        self.radius = 0.

        self.gtX = 0.
        self.gtY = 0.

        ''' Height of bebop from ultrasonic '''
        self.Z = 0.

        ''' Position estimate variable '''
        self.position = np.array([0., 0., 0.])

        ''' Time variables '''
        self.dt_odom = 0. 
        self.dt_opt = 0.
        self.dt_uwb = 0.
        self.odomPrevTime = self.odomNowTime = 0.
        self.uwbPrevTime = self.uwbNowTime = 0.

        self.speedNowTime = 0. 
        self.dt_speed = 0.
        self.speedPrevTime = 0.

        ''' Geometry transform matrix '''
        self.rotMat = np.zeros((2,2), dtype=np.float32) 


        ''' Bebop flying state '''
        self.flyingState = False

        ''' Plotting Variables '''
        self.positionXBuffer = []
        self.positionYBuffer = []
        self.startPos = [0,-2]

        ''' Thresholds '''
        self.optThreshold = 0.1


        self.fileHandle = open("log.csv","w+")



        ''' ROS Reated '''
        rospy.init_node("odomnode")
        rospy.Subscriber("/bebop/odom", Odometry, self.odomCallback)
        #rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged, 
        #        self.ardroneSpeedCallback)
        ''' Replace singleUWB message name with something better '''
        rospy.Subscriber("/uwb", singleUWB, self.uwbCallback)
        rospy.Subscriber("/bebop/takeoff", Empty, self.takeoffCallback)
        rospy.Subscriber("/bebop/land", Empty, self.landCallback)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged",
                Ardrone3PilotingStateFlyingStateChanged, self.flyingStateCallback)
        self.uwbPub = rospy.Publisher("/uwbOdom/", Odometry, queue_size=10)

    def flyingStateCallback(self, msg):
        self.flyingState = msg.state


    def ardroneSpeedCallback(self, msg):
        self.speedNowTime = msg.header.stamp.secs + msg.header.stamp.nsecs*math.pow(10,-9) 
        self.dt_speed = self.speedNowTime - self.speedPrevTime
        self.speedPrevTime = self.speedNowTime
        self.delX += self.dt_speed * msg.speedX
        self.delY += self.dt_speed * msg.speedY
        self.delZ += self.dt_speed * msg.speedZ






    ''' store optical flow variables for estimation here '''
    def odomCallback(self, msg):
        self.odomNowTime = msg.header.stamp.secs + msg.header.stamp.nsecs*math.pow(10,-9) 
        self.dt_opt = self.odomNowTime - self.odomPrevTime
        self.odomPrevTime = self.odomNowTime

        self.delX += self.dt_opt * msg.twist.twist.linear.x
        self.delY += self.dt_opt * msg.twist.twist.linear.y
        self.delZ += self.dt_opt * msg.twist.twist.linear.z
        self.Z =  msg.pose.pose.position.z


        self.gtX = msg.pose.pose.position.x + self.startPos[0]
        self.gtY = msg.pose.pose.position.y + self.startPos[1]

        self.quat = msg.pose.pose.orientation
        ''' Yaw angle only '''
        self.euler = euler_from_quaternion([self.quat.x,self.quat.y,self.quat.z,self.quat.w])[2]
        ''' Angle between axes and drone '''
        self.psi = self.xTheta - self.euler
        direction = np.sign(self.psi)
        ''' Fix wrapping over of angle issue '''
        if(abs(self.psi) > 180):
            direction = direction*(-1)
            self.psi = self.psi + direction*360 
        ''' Create Rotation matrix between drone and anchor '''
        c = np.cos(np.deg2rad(self.psi))
        s = np.sin(np.deg2rad(self.psi))
        self.rotMat = np.array([[c,-s], [s,c]])

        ''' State = 3 corresponds to flying '''
        if(self.flyingState == 3):
            self.odomFlag = True


    ''' Get radial distances from UWB here '''
    def uwbCallback(self, msg):
        self.uwbNowTime = msg.header.stamp.secs + msg.header.stamp.nsecs*math.pow(10,-9) 
        self.dt_uwb = self.uwbNowTime - self.uwbPrevTime
        self.uwbPrevTime = self.uwbNowTime
        rad = msg.radius + self.uwbOffset
        ''' Correct for plane of estimation '''
        ''' !!! '''
        self.radius = np.sqrt(rad**2 - (self.anchorPos[2]-self.Z)**2)
        self.uwbFlag = True


    ''' Buffering method, cartesian buffering in this case '''
    def bufferCartesian(self):
        if(self.estPosFlag == False):
            self.radiiBuffer[self.buffIdx,:] = np.array([self.delX, self.delY, self.Z, self.radius])
            ''' Wrap back around '''
            self.buffIdx = (self.buffIdx + 1) % self.uwbBufferLen 
            self.estPosFlag = True
            ''' Set last reading as reference point for subsequent measurements '''
            if(self.buffIdx == 0):
                for i in range (0, self.uwbBufferLen):
                    self.radiiBuffer[i,:-2] -= self.radiiBuffer[self.uwbBufferLen-1,:-2]
                self.radiiBuffer[self.uwbBufferLen-1,:-2] = np.array([0., 0.])
                self.delX = self.delY = self.delZ = 0.


    def bufferPosition(self):
        if(self.estPosFlag == False):
                self.radiiBuffer[self.buffIdx,:] = np.array([self.gtX, self.gtY, self.Z, self.radius])
                self.buffIdx = (self.buffIdx + 1) % self.uwbBufferLen 
                self.estPosFlag = True

    ''' Message conversion method to  publish estimated odometry '''
    def convMsg(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = self.position[2]
        return msg


    def takeoffCallback(self, msg):
        self.takeOffFlag = True
        self.takeoffAngle = self.euler - self.yTheta

    
    def landCallback(self, msg):
        self.landFlag = True


    ''' Least Squares Optimization function using bfgs method '''
    def f_opt(self, position, deltas, radii):
        dist = 0.
        for i in range (0, self.uwbBufferLen):
            if(radii[i] != 0.):
                dist += np.square(np.linalg.norm(position-deltas[i]) - radii[i])
        return dist


    '''
        Kalman Filter and UWB single anchor odometry code
    '''
    def run(self):
        p = np.array([0., 2.5])
        while(True):
            if(self.uwbFlag == True):
                self.uwbFlag = False
                ''' Add any filterning methods for UWB '''
            if(self.odomFlag == True):
                self.odomFlag = False
                self.bufferCartesian()
#                self.bufferPosition()
                ''' Z not estimated because of very little velocity '''
                self.position[2] = self.Z 


                if(self.estPosFlag == True):
                    print("\n\n\n")
                    print( "Inside loop ")
                    print("\n")
                    print("GT X = " + str(self.gtX) + "    GT Y = " + str(self.gtY) )
                    print("Angle    =   " + str(self.psi))
                    print("Delta =   " + str(np.array([self.delX, self.delY])))
                    print("\n")
                    print("Radii Buffer")
                    print(self.radiiBuffer)


                    if(self.delX != 0. and self.delY !=0.):
                        boundX = abs(self.delX)
                        boundY = abs(self.delY)
                    print("\n")
                    print(str(boundX) + "\t" + str(boundY))
                    # Some bounds determined from max velocity

                    p = optimize.least_squares(self.f_opt, p, 
                            args=(self.radiiBuffer[:,:-2], self.radiiBuffer[:,-1],), method='trf', loss='cauchy').x



#                   self.position[:-1] = optimize.fmin_l_bfgs_b(self.f_opt, self.position[:-1], 
#                           args=(self.radiiBuffer[:,:-2], self.radiiBuffer[:,-1],), bounds=None
#                           , approx_grad=True, disp=0)[0]

#                    self.position[:-1] = optimize.least_squares(self.f_opt, self.position[:-1], 
#                            args=(self.radiiBuffer[:,:-2], self.radiiBuffer[:,-1],), bounds=bounds, method='trf', loss='cauchy').x

#                    p = optimize.fmin_bfgs(self.f_opt, self.position[:-1], args=(
#                        self.radiiBuffer[:,:-2],self.radiiBuffer[:,-1]), disp=0)

#                    self.position[:-1] = optimize.fmin_powell(self.f_opt, self.position[:-1], args=(
#                        self.radiiBuffer[:,:-2],self.radiiBuffer[:,-1]), xtol=1e-3, ftol=1e-2, disp=0)
#

                    ''' Add relative position offset to each measurements '''
                    p -= np.array([self.delX, self.delY ])
                    ''' Rotate to anchors frame of reference '''
                    self.position[:-1] = np.matmul(self.rotMat, -p)

                    print("Position After =   " + str(self.position))
                    print("\n\n\n")


                    self.uwbPub.publish(self.convMsg())
                    ''' Plot '''
                    plt.scatter(self.position[0], self.position[1], s=30, facecolors='none', edgecolors = 'r' )
                    plt.scatter(self.gtX, self.gtY, s=30, facecolors='none', edgecolors = 'b' )
                    plt.pause(0.0000001)
                    #self.fileHandle.write(str(self.position[0]) + "," + str(self.position[1]) + "\n")

                    self.estPosFlag = False



def main():
    uwbBufferLen = 9 
    anchorPos = np.array([0., 0., 1.73])
    posXAxis = 0.05
    axis = [-10.,10, -10, 10]
    plt.xlim(axis[0],axis[1])  
    plt.ylim(axis[2],axis[3])  
    plt.xlabel("X distance in meters")
    plt.ylabel("Y distance in meters")
    od = Od(anchorPos, posXAxis, uwbBufferLen)
    od.run()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signalHandler)
    main()
