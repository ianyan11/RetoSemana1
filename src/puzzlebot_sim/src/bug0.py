#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from puzzlebot_sim.srv import SetGoal
from tf.transformations import  euler_from_quaternion
from sensor_msgs.msg import LaserScan
import math
import numpy as np
class Bug0:
    def __init__(self):

        self.pubRviz = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pubGazebo = rospy.Publisher('gazebo_puzzlebot/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.service = rospy.Service('/puzzlebot_setGoal', SetGoal, self.set_goal)
        self.goal = Point()
        self.odom = Odometry()
        self.laser = LaserScan()

        self.rate = rospy.Rate(10)

        self.d2goal_threshold = 0.2
        self.a2goal_threshold = 0.1
        self.d2obstacle = .8

        self.state = 'RUNNING'

    def odom_callback(self, data):
        self.odom = data

    def laser_callback(self, data):
        self.laser = data

    def set_robot_velocity(self, cmd_vel):
        self.pubRviz.publish(cmd_vel)
        self.pubGazebo.publish(cmd_vel)

    def set_goal(self, req):
        self.goal = req.goal

        self.run()

    def range_index(self, angle):
        min_angle = self.laser.angle_min
        max_angle = self.laser.angle_max
        size = len(self.laser.ranges)
        
        if angle > max_angle:
            return size -1
        elif angle < min_angle:
            return 0
        
        degrees = np.linspace(max_angle, min_angle, size)
        index = (np.abs(degrees-angle)).argmin()
        return index

    def calc_dist(self, angle1, angle2):
        angle1 = np.radians(angle1)
        angle2 = np.radians(angle2)
        index1 = self.range_index(angle1)
        index2 = self.range_index(angle2)

        sector_ranges = self.laser.ranges[index1:index2]
        return np.mean(sector_ranges)

    def wall_direction(self, hypotenuse, adjacent):
        hyp = self.calc_dist(hypotenuse - 1, hypotenuse)
        adj = self.calc_dist(adjacent - 1, adjacent)

        alpha = np.arctan2(hyp*np.sin(np.deg2rad(hypotenuse))-adj, hyp*np.cos(np.deg2rad(hypotenuse))) 

        return alpha

    def follow_wall(self):
        w_max = 1
        v_max = 1
        w = 0
        v = 0.1

        d2right = self.calc_dist(88, 91)
        alpha = self.wall_direction(70, 90)
        DrightAng = 0                           #desired right angle
        DrightDist = 0.30                       #desired right distance

        rightDist = d2right * np.cos(alpha)

        angErr = DrightAng - alpha              #angle error
        distErr = DrightDist - rightDist        #distance error
        
        kp_alpha = 0.9
        kp_dist = 1
        if self.laser.ranges[573] < 0.8:        #wall distance
            v = 0.1
            w = 0.7
        else:
            v = 0.2
            w = (kp_alpha * angErr) + (kp_dist * distErr)
        
            
        if w > w_max:
            w = w_max
        elif w < -w_max:
            w = -w_max
                        
        cmd_vel = Twist()
        cmd_vel.angular.z = w
        cmd_vel.linear.x = v
        self.set_robot_velocity(cmd_vel)

    

    def run(self):
        while (True):
            rospy.loginfo('State: {}'.format(self.state))
            self.rate.sleep()

            #calculate robot position
            x, y, _ = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z
            _, _, yaw = euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])

            if self.state == 'RUNNING':
                #calculate distance to goal
                d2goal = math.sqrt((self.goal.x - x)**2 + (self.goal.y - y)**2)

                if d2goal < self.d2goal_threshold:
                    self.state = 'REACHED'
                else:
                    cmd_vel = Twist()

                    #adjust angle to goal
                    if abs(yaw - math.atan2(self.goal.y - y, self.goal.x - x)) > self.a2goal_threshold:
                        cmd_vel.angular.z = 0.5 if yaw < math.atan2(self.goal.y - y, self.goal.x - x) else -0.3
                    
                    #follow goal
                    else:
                        cmd_vel.linear.x = 0.3

                    self.set_robot_velocity(cmd_vel)

                    if self.laser.ranges[573] < self.d2obstacle:
                        # Obstacle detected, switch to BUG mode
                        self.state = 'BUG'

                    

            elif self.state == 'BUG':
                # Follow the obstacle contour
                self.follow_wall()

                if (self.calc_dist(-70,70) >= (self.laser.range_max - 1) ): #and (turn to goal) <= np.deg2rad(10)
                    # Obstacle cleared, switch back to RUNNING mode
                    self.state = 'RUNNING'

            elif self.state == 'REACHED':   
                cmd_vel = Twist()
                cmd_vel.linear.x = 0
                cmd_vel.angular.z = 0             
                self.set_robot_velocity(cmd_vel)
                rospy.loginfo('Reached goal!')
                return True

if __name__ == '__main__':
    rospy.init_node('bug0', anonymous=True)
    bug0 = Bug0()
    rospy.spin()