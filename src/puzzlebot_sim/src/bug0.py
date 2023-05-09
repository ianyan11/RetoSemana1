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

    def run(self):
        while (True):
            rospy.loginfo('State: {}'.format(self.state))
            self.rate.sleep()

            x, y, _ = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z
            _, _, yaw = euler_from_quaternion([self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y, self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w])

            if self.state == 'RUNNING':
                d2goal = math.sqrt((self.goal.x - x)**2 + (self.goal.y - y)**2)

                if d2goal < self.d2goal_threshold:
                    self.state = 'REACHED'
                else:
                    cmd_vel = Twist()

                    #Que rangos se utilizarian para el lidar?????????? aqui puse si encuentra algo directamente de frente
                    if self.laser.ranges[0] < self.d2obstacle:
                        # Obstacle detected, switch to BUG mode
                        self.state = 'BUG'

                    if abs(yaw - math.atan2(self.goal.y - y, self.goal.x - x)) > self.a2goal_threshold:
                        cmd_vel.angular.z = 0.5 if yaw < math.atan2(self.goal.y - y, self.goal.x - x) else -0.3
                    else:
                        cmd_vel.linear.x = 0.3

                    self.set_robot_velocity(cmd_vel)

            elif self.state == 'BUG':
                # Follow the obstacle contour
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.2
                cmd_vel.angular.z = -0.5
                self.set_robot_velocity(cmd_vel)

                #cuando ya no tenga nada entre su frente y su izquierda
                if self.laser.ranges[0] > self.d2obstacle:
                    # Obstacle cleared, switch back to RUNNING mode
                    self.state = 'RUNNING'

            elif self.state == 'REACHED':                
                self.set_robot_velocity(0)
                rospy.loginfo('Reached goal!')
                return True

if __name__ == '__main__':
    rospy.init_node('bug0', anonymous=True)
    bug0 = Bug0()
    rospy.spin()