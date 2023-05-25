#!/usr/bin/env python3
import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker

def marker_array_callback(msg : MarkerArray):
    try:
        ret = MarkerArray()
        while len(msg.markers) > 0:
            marker = Marker()
            marker = msg.markers.pop()
            marker.header.frame_id= 'rviz_puzzlebot/camera'
            ret.markers.append(marker)

        transformed_marker_array.publish(ret)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)

if __name__ == '__main__':
    rospy.init_node('marker_transform')

    transformed_marker_array = rospy.Publisher('/rviz_puzzlebot/markers_show', MarkerArray, queue_size=10)
    rospy.Subscriber('/gazebo_puzzlebot/markers_show', MarkerArray, marker_array_callback)

    rospy.spin()
