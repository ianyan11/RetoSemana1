#!/usr/bin/env python3
import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from gazebo_msgs.msg import ModelStates


class StateGatherer():
	def __init__(self):
		self.br = tf2_ros.TransformBroadcaster()
		#get argument from launch file. the argument is the node name

		rospy.init_node("pose_tf")
		self.name = rospy.get_param("~robot_name", "puzzlebot")

		rospy.logwarn("Node " + self.name.__add__("_pose_tf") + " started")
		rospy.wait_for_service('/gazebo/set_link_state')
		rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)


	def main(self):
		rospy.spin()


	def callback(self, data):
		#rospy.logwarn(self.name)
		t = TransformStamped()
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "world"
		t.child_frame_id = self.name.__add__("/base_link")
		try: 
			aux_idx = data.name.index(self.name)
			#rospy.logwarn(data)
			t.transform.translation = data.pose[aux_idx].position
			t.transform.rotation = data.pose[aux_idx].orientation
		except:
			t.transform.translation.x = 0
			t.transform.translation.y = 0
			t.transform.translation.z = 0
			t.transform.rotation.x = 0
			t.transform.rotation.y = 0
			t.transform.rotation.z = 0
			t.transform.rotation.w = 1
			pass
		self.br.sendTransform(t)


if __name__ == '__main__':
    aux = StateGatherer()
    aux.main()
