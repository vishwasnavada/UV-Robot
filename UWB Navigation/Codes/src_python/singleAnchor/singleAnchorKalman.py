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
import matplotlib.patches as patches
import time


from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import UnscentedKalmanFilter as UKF
import threading
from multiprocessing import Queue
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.stats import plot_covariance_ellipse


np.set_printoptions(suppress=True)


''' Kill signal handler '''
def signalHandler(a, b):
    if a == 2:
        sys.exit(0)


''' Odometry class, performs computation of odometry '''
class Od:
    def __init__(self, anchorPos, xTheta, exploreLSEThreshold,startPosition, uwbBufferLen, skip, axis):

        ''' UWB Radio specific variable '''
        self.uwbOffset = -0.7

        ''' Anchor position variables '''
        self.anchorPos = anchorPos
        self.xTheta = xTheta 
        self.yTheta = xTheta + 90

        ''' estimatation buffer variables '''
        self.buffIdx = 0
        self.uwbBufferLen = uwbBufferLen
        self.exploreLSEThreshold = exploreLSEThreshold
        self.radiiBuffer = np.zeros((self.uwbBufferLen, 4), dtype=np.float32)
        self.skip = skip


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
        self.gt = np.array([0., 0.])
        self.prevGt = np.array([0., 0.])

        ''' Height of bebop from ultrasonic '''
        self.Z = 0.

        ''' Position estimate variable '''
        self.position = np.array([0., 0., 0.])
        ''' Estimated anchor position '''
        self.p = np.array([0., 0.])
        self.prevP = np.array([0., 0.])
        ''' State, x,vx,y,vy,theta '''


        '''  Velocity tracking variables '''
        self.velX = self.velY = self.velZ = 0.

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

        ''' Thresholds '''
        self.optThreshold = 0.1


        ''' Variances '''
        ''' Optical Flow variance, assuming std of 5cm '''
        self.var_opt = 0.0025
        ''' UWB variance, assuming std of 10cm '''
        self.var_uwb = 0.01 




        self.fileHandle = open("log.csv","w+")


        self.dt_opt = 1/5
        self.dt_uwb = 1/50
        points = MerweScaledSigmaPoints(n=4, alpha=0.1, beta=2., kappa=0.)
        self.kf = UKF(4, 2, self.dt_uwb, fx=self.f_bot, hx=self.h_bot, points=points)

        ''' Influence of UWB on this needs to be investigated '''
        self.kf.Q = self.var_opt * np.array([ [1,0.,self.dt_opt,0.], 
                                              [0.,1,0,self.dt_opt],
                                              [0.,0.,1,0.],
                                              [0.,0.,0.,1] ])



#        self.kf.Q = self.var_opt * np.array([ [1.,0.,0.,0.], 
#                                              [0.,1.,0.,0.],
#                                              [0.,0.,1.,0.],
#                                              [0.,0.,0.,1.] ])

        self.kf.R = np.diag([self.var_uwb+self.var_opt, self.var_uwb+self.var_opt])
        # Initial point initialized to starting point of trajectory
        self.kf.x = np.array([0.,0.,0.,0.])
        self.kf.x[:2] = startPosition 
        ''' Really bad inital guess '''
        self.kf.P = np.diag([ 1, 1, 1, 1,])


        
        ''' Plot variables '''
        self.axis = axis
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim(self.axis[0],self.axis[1])
        self.ax.set_ylim(self.axis[2],self.axis[3])

        self.ellipse = patches.Ellipse(xy=self.kf.x[:-1],width=1, height=1, angle=self.euler,lw=2, alpha=.4)  
        self.ax.add_patch(self.ellipse)



        ''' ROS Reated '''
        rospy.init_node("odomnode")
        rospy.Subscriber("/bebop/odom", Odometry, self.odomCallback, queue_size=1)
        ''' Replace singleUWB message name with something better '''
        rospy.Subscriber("/uwb", singleUWB, self.uwbCallback, queue_size=1)
        rospy.Subscriber("/bebop/takeoff", Empty, self.takeoffCallback, queue_size=1)
        rospy.Subscriber("/bebop/land", Empty, self.landCallback, queue_size=1)
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged",
                Ardrone3PilotingStateFlyingStateChanged, self.flyingStateCallback, queue_size=1)
        self.uwbPub = rospy.Publisher("/uwbOdom/", Odometry, queue_size=1)

    def flyingStateCallback(self, msg):
        self.flyingState = msg.state
        ''' State = 3 corresponds to flying '''





    ''' store optical flow variables for estimation here '''
    def odomCallback(self, msg):
        self.odomNowTime = msg.header.stamp.secs + msg.header.stamp.nsecs*math.pow(10,-9) 
        self.dt_opt = self.odomNowTime - self.odomPrevTime
        self.odomPrevTime = self.odomNowTime



        self.velX =  msg.twist.twist.linear.x
        self.velY =  msg.twist.twist.linear.y
        self.velZ =  msg.twist.twist.linear.z

#        self.delX += self.dt_opt * self.velX
#        self.delY += self.dt_opt * self.velY
#        self.delZ += self.dt_opt * self.velZ
        self.Z =  msg.pose.pose.position.z


        self.gtx = msg.pose.pose.position.x 
        self.gty = msg.pose.pose.position.y 

        self.gt = np.array([self.gtx, self.gty])

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

        self.prevState = self.kf.x



    ''' Get radial distances from UWB here '''
    def uwbCallback(self, msg):
        self.uwbNowTime = msg.header.stamp.secs + msg.header.stamp.nsecs*math.pow(10,-9) 
        self.dt_uwb = self.uwbNowTime - self.uwbPrevTime
        self.uwbPrevTime = self.uwbNowTime
        rad = msg.radius + self.uwbOffset
        ''' Correct for plane of estimation '''
        ''' !!! '''
        self.radius = np.sqrt(rad**2 - (self.anchorPos[2]-self.Z)**2)
        if(self.flyingState == 3):
            self.kf.predict()
            self.delX += self.kf.x[2]*self.dt_uwb
            self.delY += self.kf.x[3]*self.dt_uwb
            self.bufferDelta()
            self.uwbFlag = True
        else:
            self.uwbFlag = False


    ''' Buffering method, cartesian buffering in this case '''
#    def bufferDelta(self):
#        if(self.estPosFlag == False):
#            self.radiiBuffer[self.buffIdx,:] = np.array([self.delX, self.delY, self.Z, self.radius])
#            ''' Wrap back around '''
#            self.buffIdx = (self.buffIdx + 1) % self.uwbBufferLen 
#            self.estPosFlag = True
#            ''' Set last reading as reference point for subsequent measurements '''
#            if(self.buffIdx == 0):
#                for i in range (0, self.uwbBufferLen):
#                    self.radiiBuffer[i,:-2] -= self.radiiBuffer[self.uwbBufferLen-1,:-2]
#                self.radiiBuffer[self.uwbBufferLen-1,:-2] = np.array([0., 0.])
#                self.delX = self.delY = self.delZ = 0.

    ''' Buffering method, cartesian buffering in this case '''
    def bufferDelta(self):
        self.radiiBuffer[self.buffIdx,:] = np.array([self.delX, self.delY, self.Z, self.radius])
        ''' Wrap back around '''
        self.buffIdx = (self.buffIdx + 1) % self.uwbBufferLen 
        self.estPosFlag = True
        ''' Set last reading as reference point for subsequent measurements '''
        if(self.buffIdx == 0):
            for i in range (0, self.uwbBufferLen):
                self.radiiBuffer[i,:-2] -= self.radiiBuffer[self.uwbBufferLen-1,:-2]
            self.radiiBuffer[self.uwbBufferLen-1,:-2] = np.array([0., 0.])
            if(self.flyingState == 3):
                self.odomFlag = True
            else:
                self.delX = self.delY = 0.
                self.odomFlag = False



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




    def covariance_ellipse(self, P, deviations=1):
        """
        Returns a tuple defining the ellipse representing the 2 dimensional
        covariance matrix P.
        Parameters
        ----------
        P : nd.array shape (2,2)
           covariance matrix
        deviations : int (optional, default = 1)
           # of standard deviations. Default is 1.
        Returns (angle_radians, width_radius, height_radius)
        """
        U, s, _ = np.linalg.svd(P)
        orientation = math.atan2(U[1, 0], U[0, 0])
        width = deviations * math.sqrt(s[0])
        height = deviations * math.sqrt(s[1])

        if height > width:
            raise ValueError('width must be greater than height')

        return (orientation, width, height)


    ''' Least Squares Optimization function  '''
    def f_opt(self, position, deltas, radii, skip):
        dist = 0.
        skip += 1
        for i in range (0, self.uwbBufferLen/skip):
            if(radii[i] != 0.):
                dist += np.square(np.linalg.norm(position-deltas[i*skip]) - radii[i*skip])
        return dist



    # State transition function
    def f_bot(self, state, dt):
        state[2] = self.velX
        state[3] = self.velY
        state[:2] += np.matmul(self.rotMat, self.dt_uwb*state[2:4])
        return state


    # Measurement function
    def h_bot(self, state):
        return state[:2]


    '''Finds Least Squares error between given point and entries in the radiiBuffer '''
    def findLSE(self, radiiBuffer, p):
        lse = 0.
        for i in range (0,self.uwbBufferLen):
            lse += np.square( np.linalg.norm(p-self.radiiBuffer[i][:2]) - self.radiiBuffer[i][-1])
        lse = np.sqrt(lse)/self.uwbBufferLen
        return lse

    ''' Perform exploration step
        Possibly could include any of the following 
            a) Tangential and normal movements w.r.t UWB 
            b) Rotation manoeuver
        In brief, exploration - 
            Buffers uwb readings till full
            Estimates the anchor location once full
            Checks satisfaction of the estimate using LSs
            Exits
    '''
    def explore(self):
        print("Exploration started")
        while(True):
            if(self.odomFlag==True):
                self.odomFlag = False
                try:
                    p = optimize.least_squares(self.f_opt, np.array([0.,0.]), 
                            args=(self.radiiBuffer[:,:-2], self.radiiBuffer[:,-1],self.skip,), 
                            method='trf', loss='cauchy').x 
                    lse = self.findLSE(self.radiiBuffer,p)
                    print("LSE = ")
                    print(lse)
                    print(p)
                    if(lse < self.exploreLSEThreshold):
                        return p
                    else:
                        lse = 0.
                except Exception as e:
                    print(e)

            

        pass

    '''
        Kalman Filter and UWB single anchor odometry code
    '''
    def run(self):
        p = np.array([0., 2.5]) 

        #p = self.explore()
        self.kf.x[:2] = self.anchorPos[:2] - p + np.array([self.delX, self.delY])

        ''' Start main thread '''
        while(True):
            if(self.flyingState == 3):
                if(self.uwbFlag == True):
                    #self.uwbFlag = False
                    #print("\n\n\n")
                    #print("\n")
                    #print("Position")
                    #print(self.position)
                    #print("State ")
                    #print(self.kf.x)
                    #print("GT X = " + str(self.gtX) + "    GT Y = " + str(self.gtY) )
                    #print("Angle    =   " + str(self.psi))
                    #print("Delta =   " + str(np.array([self.delX, self.delY])))
                    #print("Velocity =   " + str(np.array([self.velX, self.velY])))
                    #print("dt_opt = " + str(self.dt_opt))
                    #print("\n")
                    #print("Radii Buffer")
                    #print(self.radiiBuffer)
                    #print("---------------------------------\n")
                    #print(self.kf.P)
#                    ''' Plot '''
                    try:
                        p = optimize.least_squares(self.f_opt, -self.kf.x[:2]+self.anchorPos[:2], 
                                args=(self.radiiBuffer[:,:-2], self.radiiBuffer[:,-1],self.skip,), 
                                method='trf', loss='cauchy').x 
                    except Exception as e:
                        print(e)

                    ''' Z not estimated because of very little velocity '''
                    self.position[2] = self.Z 
                    ''' Plot '''

                    ''' 
                        -p          Drone pos wrt anchor assuming anchor is at 0,0
                        +np.matmul  Adding incremental position wrt optimization step
                        -anchorPos  Shifting origin to global origin
                    '''
                    self.kf.update(-p+ np.matmul(self.rotMat, np.array([self.delX, self.delY]) + self.anchorPos[:2]))

                    self.position[:2] = self.kf.x[:2] 
                    self.delX = self.delY = 0.
                    self.uwbPub.publish(self.convMsg())

                    self.uwbFlag = False


                    plt.scatter(self.position[0], self.position[1], s=30, facecolors='none', edgecolors = 'r' )
                    self.ax.scatter(self.kf.x[0], self.kf.x[1], s=30, facecolors='none', edgecolors = 'g' )
                    self.ax.scatter(self.gtx, self.gty, s=30, facecolors='none', edgecolors = 'b' )



                   # Covariance ellipse
                    orientation, width, height = self.covariance_ellipse(self.kf.P)
                    self.ellipse.xy = self.kf.x[:-1]
                    self.ellipse.width = width*2
                    self.ellipse.height = height*2
                    self.ellipse.angle = np.degrees(orientation)
                    self.ax.add_patch(patches.Ellipse(xy=self.kf.x[:-1],width=width, height=height, angle=self.ellipse.angle,lw=2, alpha=.4))




                    plt.pause(0.0000001)
#    

               # print("Position ")
               # print(str(self.position[0]) + "," + str(self.position[1]) + "\n")


               # self.estPosFlag = False



def main():
    uwbBufferLen =  30
    exploreLSEThreshold = .05
    skip = 0
    #anchorPos = np.array([5., 0., 1.73])
    anchorPos = np.array([0., 0., 1.73])
    startPosition = np.array([0.,0.])
    #startPosition = np.array([5.,-2.5])
    angXAxis = .05
    axis = [-10.,10, -10, 10]
    od = Od(anchorPos, angXAxis, exploreLSEThreshold, startPosition, uwbBufferLen, skip, axis)
    od.run()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signalHandler)
    main()
