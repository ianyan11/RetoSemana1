#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler 

class Odom():

    def __init__(self, rate):
        self.kr = .8
        self.kl = .8
        self.wl = 0
        self.wr = 0
        self.rate =rate
        self.l = .08
        self.r = .05
        self.pose = np.empty((3,1))
        self.sigmak = np.empty((3,3))

        rospy.Subscriber('/wl', Float32, self.update_wl)
        rospy.Subscriber('/wr', Float32, self.update_wr)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odomConstants = self.fill_odomerty()

    def update_wl(self, wl: Float32) -> None:
        self.wl = wl.data

    def update_wr(self, wr: Float32) -> None:
        self.wr = wr.data

    def calculate_speeds(self) -> None:
        self.v = self.r * (self.wr + self.wl) / 2 
        self.w = self.r * (self.wr - self.wl) / self.l

    def calculate_dead_reckoning(self) -> None:
        angle = self.pose[2].item() 
        mat = np.array([[self.v * math.cos(angle)], 
                        [self.v * math.sin(angle)],
                        [self.w]])/self.rate 
        self.pose += mat
        
    def calculate_odometry(self) -> None:
        self.calculate_speeds()
        self.calculate_dead_reckoning()
        self.calculcate_covariance()

        self.odomConstants.pose.pose.position = Point(self.pose[0].item() , self.pose[1].item() , self.r)
        q = quaternion_from_euler(0, 0, self.pose[2].item())
        self.odomConstants.header.stamp = rospy.Time.now() #time stamp
        self.odomConstants.pose.pose.orientation.x = q[0]
        self.odomConstants.pose.pose.orientation.y = q[1]
        self.odomConstants.pose.pose.orientation.z = q[2]
        self.odomConstants.pose.pose.orientation.w = q[3]
        self.odomConstants.twist.twist.linear.x = self.v
        self.odomConstants.twist.twist.angular.z = self.w
        self.odom.publish(self.odomConstants)
    
    def get_hk(self) -> np.ndarray:
        angle = self.pose[2].item()
        val1 = -self.v * math.sin(angle) / self.rate
        val2 = self.v * math.cos(angle) / self.rate
        return np.array([[1, 0, val1],
                         [0, 1, val2],
                         [0, 0, 1]])

    def get_qk(self) -> np.ndarray:
        angle = self.pose[2].item()
        dwk = (self.r/(2*self.rate))*np.array([[math.cos(angle), math.cos(angle)],
                       [math.sin(angle), math.sin(angle)],
                       [2/self.l, -2/self.l]])
        sdk = np.array([[self.kr*abs(self.wr), 0],
                        [0, self.kl*abs(self.wl)]])
        qk = np.dot(np.dot(dwk,sdk),dwk.T)
        return qk
    
    def calculcate_covariance(self) -> None:
        hk = self.get_hk()
        Qk =  self.get_qk()
        self.sigmak = np.dot(np.dot(hk, self.sigmak), hk.T) + Qk
        self.odomConstants.pose.covariance[0] = self.sigmak[0][0] 
        self.odomConstants.pose.covariance[1] = self.sigmak[0][1]
        self.odomConstants.pose.covariance[5] = self.sigmak[0][2]
        self.odomConstants.pose.covariance[6] = self.sigmak[1][0]
        self.odomConstants.pose.covariance[7] = self.sigmak[1][1]
        self.odomConstants.pose.covariance[11] = self.sigmak[1][2]
        self.odomConstants.pose.covariance[30] = self.sigmak[2][0]
        self.odomConstants.pose.covariance[31] = self.sigmak[2][1]
        self.odomConstants.pose.covariance[35] = self.sigmak[2][2]
        
    def fill_odomerty(self)-> Odometry:
        odometry = Odometry()
        odometry.header.stamp = rospy.Time.now() #time stamp
        odometry.header.frame_id = "world" #parent frame (joint)
        odometry.child_frame_id = "rviz_puzzlebot/base_link" #child frame
        odometry.pose.pose.position.x = 0.0 #position of the robot “x” w.r.t “parent frame”
        odometry.pose.pose.position.y = 0.0 # position of the robot “x” w.r.t “parent frame”
        odometry.pose.pose.position.z = (self.r) #position of the robot “x” w.r.t “parent frame” 
        odometry.pose.pose.orientation.x = 0.0 #Orientation quaternion “x” w.r.t “parent frame”
        odometry.pose.pose.orientation.y = 0.0 #Orientation quaternion “y” w.r.t “parent frame”
        odometry.pose.pose.orientation.z = 0.0 #Orientation quaternion “z” w.r.t “parent frame”s
        odometry.pose.pose.orientation.w = 0.0 #Orientation quaternion “w” w.r.t “parent frame”
        odometry.pose.covariance = [0]*36 #Position Covariance 6x6 matrix (empty for now)

        odometry.twist.twist.linear.x = 0.0 #Linear velocity “x”
        odometry.twist.twist.linear.y = 0.0 #Linear velocity “y”
        odometry.twist.twist.linear.z = 0.0 #Linear velocity “z”
        odometry.twist.twist.angular.x = 0.0 #Angular velocity around x axis (roll)
        odometry.twist.twist.angular.y = 0.0 #Angular velocity around x axis (pitch)
        odometry.twist.twist.angular.z = 0.0 #Angular velocity around x axis (yaw)
        odometry.twist.covariance = [0]*36 #Velocity Covariance 6x6 matrix (empty for now)
        return odometry
    
    def run(self) -> None:
        self.calculate_odometry()

def main():
    rospy.init_node('localisation', anonymous=True)
    hz = 100
    rate = rospy.Rate(hz)
    model = Odom(hz)
    while not rospy.is_shutdown():
        model.run()
        rate.sleep()

if (__name__== "__main__") :
    main()
