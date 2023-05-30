#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist, Vector3
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion 
from gazebo_msgs.msg import ModelStates, ModelState
import csv

class data_gathering:
    def __init__(self):
        self.rviz_pose = PoseStamped()
        self.gazebo_pose = ModelState()
        self.restart_rviz = rospy.ServiceProxy('/restart', Empty)
        self.restart_gazebo = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        rospy.Subscriber('/pose_sim', PoseStamped, self.handle_rviz_pose)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.handle_gazebo_pose)
        self.gazebo_cmd_vel = rospy.Publisher('/gazebo_puzzlebot/cmd_vel', Twist, queue_size=10)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def set_speed(self, speed: Twist) -> None:
        self.gazebo_cmd_vel.publish(speed)
        self.cmd_vel.publish(speed)

    def handle_rviz_pose(self, pose: PoseStamped) -> None:
        self.rviz_pose = pose

    def handle_gazebo_pose(self, pose: ModelStates) -> None:
        index = pose.name.index('gazebo_puzzlebot')
        self.gazebo_pose.pose = pose.pose[index]
        print(self.gazebo_pose.pose)

    def save_data(self, file: csv.writer) -> None:
        rviz_pose = self.rviz_pose
        gazebo_pose = self.gazebo_pose


        angles_rviz = euler_from_quaternion([rviz_pose.pose.orientation.x, rviz_pose.pose.orientation.y, rviz_pose.pose.orientation.z, rviz_pose.pose.orientation.w])
        angles_gazebo = euler_from_quaternion([gazebo_pose.pose.orientation.x, gazebo_pose.pose.orientation.y, gazebo_pose.pose.orientation.z, gazebo_pose.pose.orientation.w])

        #get index where gazebo_pose model_name is equal to puzzlebot
        rospy.loginfo(gazebo_pose)
        rospy.loginfo("rviz_x: " + str(rviz_pose.pose.position.x) + " rviz_y: " + str(rviz_pose.pose.position.y) + " rviz_theta: " + str(angles_rviz[2]) +
                    " gazebo_x: " + str(gazebo_pose.pose.position.x) + " gazebo_y: " + str(gazebo_pose.pose.position.y) + " gazebo_theta: " + str(angles_gazebo[2]))
        file.writerow([rviz_pose.pose.position.x, rviz_pose.pose.position.y, angles_rviz[2], gazebo_pose.pose.position.x, gazebo_pose.pose.position.y, angles_gazebo[2]])
        

    def restart_position(self) -> None:
        rospy.wait_for_service("/restart")
        self.restart_rviz()
        state = ModelState()
        state.model_name = 'gazebo_puzzlebot'
        state.pose.position.x = 0
        state.pose.position.y = 0
        state.pose.position.z = .045
        self.restart_gazebo.publish(state)

    def main(self):
        rospy.init_node('data_gathering', anonymous=True)    
        #gazebo pose subscriber
        self.restart_position()
        self.set_speed(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
        with open('data.csv', 'w', encoding='UTF8') as f:
            file = csv.writer(f)
            file.writerow(['rviz_x', 'rviz_y', 'rviz_theta','gazebo_x', 'gazebo_y', 'gazebo_theta'])
            for i in range(11):
                self.set_speed(Twist(Vector3(0.3, 0, 0), Vector3(0, 0, 0)))
                #wait for 1 second
                rospy.sleep(2)
                #stop
                self.set_speed(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1)

                #save data
                self.save_data(file)
                #restart position
                self.restart_position()

            for i in range(10):
                self.set_speed(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.3)))
                #wait for 1 second
                rospy.sleep(2)
                #stop
                self.set_speed(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                rospy.sleep(1)

                #save data
                self.save_data(file)
                #restart position
                self.restart_position()            

        f.close()


                



if (__name__== "__main__") :
    dg = data_gathering()
    dg.main()
