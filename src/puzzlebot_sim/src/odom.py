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
        self.kr = 300
        self.kl = 300
        self.wl = 0
        self.wr = 0
        self.rate =rate
        self.wheel_radius = .05
        self.wheel_distance = .08
        self.pose = np.array([0, 0, 0], dtype=np.float64)
        rospy.Subscriber('/wl', Float32, self.update_wl)
        rospy.Subscriber('/wr', Float32, self.update_wr)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odomConstants = self.fill_odomerty()
        self.Qk = np.array([
            [0.5, 0.01, 0.01],
            [0.01, 0.5, 0.01],
            [0.01, 0.01, 0.2]
        ] )
        self.Hk = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        self.sigmak = np.array([
            [0, 0, 0.0],
            [0, 0, 0.0],
            [0, 0, 0.0]
        ])

    def update_wl(self, wl: Float32) -> None:
        self.wl = wl.data

    def update_wr(self, wr: Float32) -> None:
        self.wr = wr.data

    def calculate_odometry(self) -> None:
        l = self.wheel_distance
        mat = np.array([[.5, .5], [1/l, -1/l]])
        deltaD, deltaTheta = np.dot(mat, np.array([self.wr, self.wl])*self.wheel_radius)
        self.calculcate_covariance(deltaD)
        self.pose += np.array([deltaD * math.cos(self.pose[2]), deltaD * math.sin(self.pose[2]), deltaTheta])/self.rate

        self.odomConstants.pose.pose.position = Point(self.pose[0], self.pose[1], self.wheel_radius)
        q = quaternion_from_euler(0, 0, self.pose[2])
        self.odomConstants.header.stamp = rospy.Time.now() #time stamp
        self.odomConstants.pose.pose.orientation.x = q[0]
        self.odomConstants.pose.pose.orientation.y = q[1]
        self.odomConstants.pose.pose.orientation.z = q[2]
        self.odomConstants.pose.pose.orientation.w = q[3]
        self.odomConstants.twist.twist.linear.x = deltaD
        self.odomConstants.twist.twist.angular.z = deltaTheta
        self.odom.publish(self.odomConstants)

    def calculcate_covariance(self, deltaD) -> None:

        self.Hk[0][2] = -deltaD * math.sin(self.pose[2])/self.rate
        self.Hk[1][2] = deltaD * math.cos(self.pose[2])/self.rate
        wk = self.wheel_radius/(2*self.rate)*np.array([[math.cos(self.pose[2]),math.cos(self.pose[2])],
                                           [math.sin(self.pose[2]),math.sin(self.pose[2])],
                                           [2/self.wheel_distance, -2/self.wheel_distance]])
        Edk = np.array([[self.kr*abs(self.wr), 0],
                       [0, self.kl*abs(self.wl)]])
      
        self.Qk = np.dot(np.dot(wk,Edk),np.transpose(wk))
        self.sigmak = np.dot(np.dot(self.Hk, self.sigmak), np.transpose(self.Hk))+self.Qk
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
        odometry.pose.pose.position.z = (self.wheel_radius) #position of the robot “x” w.r.t “parent frame” 
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
    hz = 60
    rate = rospy.Rate(hz)
    model = Odom(hz)
    while not rospy.is_shutdown():
        model.run()
        rate.sleep()

if (__name__== "__main__") :
    main()
