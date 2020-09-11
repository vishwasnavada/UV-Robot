#!/usr/bin/python2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import sys
import signal
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import numpy as np

def sigHandle(sig, frame):
    print("Closing")
    sys.exit(0)


class bot():

    def __init__(self,uwbCov):
        self.uwbdata = np.array([0., 0., 0.]) # UWB odometry placeholder
        self.gtdata = np.array([0., 0., 0.]) # Mocap ground truth
        self.pos = np.array([0., 0., 0.])
        self.cov = uwbCov
        self.ax = self.ay = self.az = 0.  # Accelerometer accelerations
        self.yaw = 0.  # 2D rotation angle wrt turtle's front
        self.wx = self.wy = self.wz = 0  # Angular velocities about the axes

        self.uwbRec = False # Received UWB measurement flag
        self.stateRec = False
        self.gtRec = False

    # Sets IMU data here
    def imu_subscriber(self, state):
        self.ax = state.linear_acceleration.x
        self.ay = state.linear_acceleration.y
        self.az = state.linear_acceleration.z
        self.wx = state.angular_velocity.x
        self.wy = state.angular_velocity.y
        self.wz = state.angular_velocity.z
        quaternion = [state.orientation.x, state.orientation.y, state.orientation.z,
                      state.orientation.w]  # 3 Axis Compass gives quaternion output
        self.yaw = euler_from_quaternion(quaternion)[2] # Convert to Euler angle, and get yaw value
        self.stateSet = True

    # Sets Mocap ground truth here
    def gt_callback(self, data):
        self.gtdata = np.array( [data.pose.position.x, data.pose.position.y, 
                        data.pose.position.z]) # Y is the height
        self.gtRec = True

    # Sets UWB odometry here
    def uwb_callback(self, data):
        self.uwbdata[0] = data.pose.pose.position.x
        self.uwbdata[1] = data.pose.pose.position.y # Y is the height
        self.uwbdata[2] = data.pose.pose.position.z
        self.uwbRec = True
        print(str(self.uwbdata) + "\t" + str(self.gtdata))

    # Run loop
    def run(self):
        while(True):
            # Perform the particle filter here upn checking self.dataRec
            pass


def main():

    global turtle
    # UWB XYZ covariance obtained from experiments
    uwbCov = np.array([[0.00382429,-0.00285907,0.00308421], [-0.00285907,0.00264731,-0.00242654], 
                        [0.00308421, -0.00242654,  0.0026886]])
    turtle = bot(uwbCov)

    rospy.init_node("turtle")
    rospy.Subscriber("/vrpn_client_node/bot/pose",
                     PoseStamped, turtle.gt_callback) # Motion capture ground truth
    rospy.Subscriber("/uwbtfd", Odometry, turtle.uwb_callback) # UWB coordinate transformed message
    rospy.Subscriber("/imu", Imu, turtle.imu_subscriber)

    turtle.run()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigHandle)
    main()
