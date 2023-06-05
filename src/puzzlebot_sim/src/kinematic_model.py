#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Pose, TransformStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import PoseStamped
from tf.transformations import quaternion_from_euler 
from tf2_ros import TransformBroadcaster 
 
class kinematic_model:
    def __init__(self):
        self.linear_vel = 0
        self.angular_vel = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        self.pub = rospy.Publisher('/pose_sim', PoseStamped, queue_size=10)
        self.wl = rospy.Publisher('/wl', Float32, queue_size=10)
        self.wr = rospy.Publisher('/wr', Float32, queue_size=10)
        self.odom = rospy.Publisher('/odom', Odometry, queue_size=10)

        self.radius = .05
        self.wheel_distance = .09
        self.frequency = 100
        rospy.Subscriber('/cmd_vel', Twist, self.update_values)
        rospy.Service('/restart', Empty, self.restart)

    def calculate_pose(self) -> None:
        self.x += self.linear_vel *math.cos(self.theta) / self.frequency 
        self.y += self.linear_vel *math.sin(self.theta) / self.frequency 
        #self.theta += math.radians(self.angular_vel)
        self.theta += self.angular_vel / self.frequency
        self.publish_stamp()

    def calculate_wheels(self) -> None:
        self.wl.publish((self.linear_vel*2 - self.angular_vel*self.wheel_distance)/(2*self.radius))
        self.wr.publish((self.linear_vel*2 + self.angular_vel*self.wheel_distance)/(2*self.radius))

    def restart(self, empty: Empty) -> None:
        self.x = 0
        self.y = 0
        self.theta = 0
        return []



    def update_values(self, twist: Twist) -> None:
        self.linear_vel = twist.linear.x
        self.angular_vel = twist.angular.z

    def handle_pose(self, pose: Pose) -> None:
        self.x = pose.position.x
        self.y = pose.position.y
        self.theta = pose.orientation.z

    #Send transformation using broadcaster
    def broadcast_transform(self, orientation: Quaternion):
        #Initialize broadcaster
        br = TransformBroadcaster()
        t = TransformStamped()
        #Fill the transform with the position and orientations
        t.header.stamp = rospy.Time.now()
        #Frame names
        t.header.frame_id = "world"
        t.child_frame_id = "kinematic_model/pose"
        t.transform.translation = Vector3(self.x, self.y, 0)
        t.transform.rotation = orientation
        #Send transform
        br.sendTransform(t)

    def publish_stamp(self) -> None:
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = self.x
        poseStamped.pose.position.y = self.y
        #angle to quaternion using tf2
        q= quaternion_from_euler(0, 0, self.theta)

        poseStamped.pose.orientation.x = q[0]
        poseStamped.pose.orientation.y = q[1]
        poseStamped.pose.orientation.z = q[2]
        poseStamped.pose.orientation.w = q[3]

        poseStamped.header.frame_id = "world"
        self.broadcast_transform(poseStamped.pose.orientation)
        self.pub.publish(poseStamped)
            
    def run(self) -> None:
        self.calculate_pose()
        self.calculate_wheels()

def main():
    rospy.init_node('puzzlebot_sim', anonymous=True)
    model = kinematic_model()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        model.run()
        rate.sleep()

if (__name__== "__main__") :
    main()
