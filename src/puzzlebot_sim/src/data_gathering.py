#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion 
from gazebo_msgs.msg import ModelStates, ModelState
import csv

rviz_pose = PoseStamped()
gazebo_pose = ModelState()
restart_rviz = rospy.Publisher('/restart', Empty, queue_size=10)
restart_gazebo = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
def handle_rviz_pose(pose: PoseStamped) -> None:
    global rviz_pose
    rviz_pose = pose

def handle_gazebo_pose(pose: ModelStates) -> None:
    global gazebo_pose
    index = pose.name.index('puzzlebot')
    gazebo_pose.pose = pose.pose[index]

def save_data(file: csv.writer) -> None:
    global rviz_pose, gazebo_pose


    angles_rviz = euler_from_quaternion([rviz_pose.pose.orientation.x, rviz_pose.pose.orientation.y, rviz_pose.pose.orientation.z, rviz_pose.pose.orientation.w])
    angles_gazebo = euler_from_quaternion([gazebo_pose.pose.orientation.x, gazebo_pose.pose.orientation.y, gazebo_pose.pose.orientation.z, gazebo_pose.pose.orientation.w])

    #get index where gazebo_pose model_name is equal to puzzlebot
    rospy.loginfo(gazebo_pose)
    rospy.loginfo("x: " + str(rviz_pose.pose.position.x) + " y: " + str(rviz_pose.pose.position.y) + " z: " + str(rviz_pose.pose.orientation.z) + " theta: " + str(angles_rviz[2]) +
                 " gazebo_x: " + str(gazebo_pose.pose.position.x) + " gazebo_y: " + str(gazebo_pose.pose.position.y) + " gazebo_z: " + str(gazebo_pose.pose.orientation.z) + " gazebo_theta: " + str(angles_gazebo[2]))
    file.writerow([rviz_pose.pose.position.x, rviz_pose.pose.position.y, rviz_pose.pose.orientation.z, angles_rviz[2], gazebo_pose.pose.position.x, gazebo_pose.pose.position.y, gazebo_pose.pose.orientation.z, angles_gazebo[2]])
    

def restart_position() -> None:
    global restart_rviz
    restart_rviz.publish(Empty())
    state = ModelState()
    state.model_name = 'puzzlebot'
    state.pose.position.x = 0
    state.pose.position.y = 0
    state.pose.position.z = .05
    restart_gazebo.publish(state)

def main():
    rospy.init_node('data_gathering', anonymous=True)    
    cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/pose_sim', PoseStamped, handle_rviz_pose)
    #gazebo pose subscriber
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_gazebo_pose)

    with open('data.csv', 'w', encoding='UTF8') as f:
        file = csv.writer(f)
        file.writerow(['rviz_x', 'rviz_y', 'rviz_z', 'rviz_theta','gazebo_x', 'gazebo_y', 'gazebo_z', 'gazebo_theta'])
        restart_position()
        for i in range(10):
            cmd_vel.publish(Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0)))
            #wait for 1 second
            rospy.sleep(1)
            #stop
            cmd_vel.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

            #save data
            save_data(file)

            #restart position
            restart_position()
        for i in range(10):
            cmd_vel.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3)))
            #wait for 1 second
            rospy.sleep(1)
            #stop
            cmd_vel.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

            #save data
            save_data(file)

            #restart position
            restart_position()            

    f.close()


                



if (__name__== "__main__") :
    main()
