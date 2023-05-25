#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
def point_cloud_callback(msg : LaserScan):
    try:
        msg.header.frame_id = 'rviz_puzzlebot/lidar'
        transformed_cloud_pub.publish(msg)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)

if __name__ == '__main__':
    rospy.init_node('lidar_transform')

    transformed_cloud_pub = rospy.Publisher('/transformed_scan', LaserScan, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, point_cloud_callback)

    rospy.spin()
