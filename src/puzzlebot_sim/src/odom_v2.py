#!/usr/bin/env python3
import rospy
import numpy as np
from math import cos, sin, sqrt, atan2, pi
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from gazebo_msgs.srv import GetModelState, GetModelStateResponse, GetModelStateRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt32MultiArray
from tf.transformations import quaternion_from_euler
from aruco_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformBroadcaster, TransformStamped
import tf
class Odom():
    def __init__(self):
        self.kr = .6
        self.kl = .6
        self.sigma_r = .001
        self.sigma_phi = .001
        self.kl = .6
        self.wl = 0
        self.wr = 0
        self.l = .09
        self.r = .05
        self.robot_name = rospy.get_param('robot_name', '')
        self.__s = np.array([[0],[0],[0]]) # State vector
        self.__sigma = np.array([[0,0,0],[0,0,0],[0,0,0]]) # Covariance matrix
        self.__previous_time = rospy.Time.now()
        rospy.Subscriber('/wl', Float32, self.__update_wl)
        rospy.Subscriber('/wr', Float32, self.__update_wr)
        rospy.Subscriber("/aruco_marker_publisher/markers", MarkerArray, self.__update_markers)
        rospy.Subscriber("/aruco_marker_publisher/markers_list", UInt32MultiArray, self.__update_markers_list)

        self.markers = MarkerArray()
        self.marker_list = UInt32MultiArray()

        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.robot_pose_publisher = rospy.Publisher('/pose_sim', PoseStamped, queue_size=10)
        self.odom_message = self.__fill_odomerty()
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.listener = tf.TransformListener()
    
    def __update_markers_list(self, marker_list: UInt32MultiArray) -> None:
        """Updates the marker list"""
        self.marker_list = marker_list

    def __update_markers(self, markers: MarkerArray) -> None:
        """Updates the markers array"""
        self.markers = markers

    def __update_wl(self, wl: Float32) -> None:
        """Updates the right wheel speed"""
        self.wl = wl.data

    def __update_wr(self, wr: Float32) -> None:
        """Updates the right wheel speed"""
        self.wr = wr.data

    def get_u_vector(self) -> np.ndarray:
        """ Return the input vector u where u = [v, w]

        Attributes:
            v: linear velocity
            w: angular velocitys

        :return: u
        """
        v = self.r * (self.wr + self.wl) / 2
        w = self.r * (self.wr - self.wl) / self.l
        return np.array([v, w]).T

    def __calulate_delta_t(self) -> float:
        """Returns the change in time delta_t

        :return: delta_t
        """
        current_time = rospy.Time.now()
        delta_t = (current_time.to_sec() - self.__previous_time.to_sec())
        self.__previous_time = current_time
        return delta_t

    def update_s_vector(self, delta_u: np.ndarray, s: np.ndarray) -> np.ndarray:
        """Returns the updated state vector s where s = [x, y, theta].
        This function is also called h(s, u) in Manchester literature.
        Attributes:
            theta: angle of the robot
            delta_d: change in the linear distance 
            delta_theta: change in the angle
        :param u: input vector
        :param s: state vector

        :return: s
        """
        theta = s[2][0]  # If you don add the [0] it will be a 2d array and you will get an error. s[2].item() also works.
        delta_d = delta_u[0]
        delta_theta = delta_u[1]
        s = np.array([[delta_d*cos(theta) + s[0][0]],
                      [delta_d*sin(theta) + s[1][0]],
                      [delta_theta + theta]])
        return s

    def get_Hs_matrix(self, s: np.ndarray, delta_u: np.ndarray) -> np.ndarray:
        """Returns the Hs matrix where Hs is the jacobian of h(s, u) with respect to s

        Attributes:
            theta: angle of the robot
            delta_d: change in the linear distance 

        :param s: state vector
        :param delta_u: displacement vector

        :return: Hs
        """
        theta = s[2][0]
        delta_d = delta_u[0]
        Hs = np.array([[1, 0, -delta_d*sin(theta)],
                       [0, 1, delta_d*cos(theta)],
                       [0, 0, 1]])
        return Hs

    def get_matrix_Q(self, s: np.ndarray, delta_t:float) -> np.ndarray:
        """Returns the nondeterministisc error matrix Q.

        :param s: state vector

        :return: Q
        """
        Sigma_Delta = self.calculate_noise_matrix()
        Delta_Omega = self.get_nabla_omega(s, delta_t)
        return Delta_Omega @ Sigma_Delta @ Delta_Omega.T

    def calculate_noise_matrix(self) -> None:
        """Returns the noise matrix

        :return: noise matrix
        """
        return np.array([[self.kr*abs(self.wr), 0],
                         [0, self.kl*abs(self.wl)]])

    def get_nabla_omega(self, s : np.ndarray, delta_t : float) -> np.ndarray:
        """Returns the nabla_omega matrix

        :param s: state vector

        :return: nabla_omega matrix
        """
        theta = s[2][0]
        nabla_omega = 0.5 * self.r * delta_t *np.array([
            [cos(theta), cos(theta)],
            [sin(theta), sin(theta)],
            [2/self.l, 2/self.l]])
        return nabla_omega

    def priori_covariance(self, sigma : np.ndarray, u : np.ndarray) -> np.ndarray:
        """ Returns the covariance matrix before the measurement update

        Attributes:
            delta_t: change in time
            delta_u: displacement vector
            s: state vector
            Hs: jacobian of h(s, u) with respect to s
            Qs: nondeterministic noise matrix

        :param sigma: covariance matrix
        :param u: input vector

        :return: covariance matrix
        """
        delta_t = self.__calulate_delta_t() # Calculate the change in time
        delta_u = u * delta_t # Calculate the displacement vector
        s = self.update_s_vector(delta_u, self.__s) # get update the state vector
        Hs = self.get_Hs_matrix(s, delta_u) # Get the Hs matrix
        Qs = self.get_matrix_Q(s, delta_t) # Get the Qs matrix
        sigma = Hs @ self.__sigma @ Hs.T + Qs # Calculate the covariance matrix
        self.__s = s # save the state vector
        return sigma # return the covariance matrix

    def posteriori_covariance(self, sigma : np.ndarray) -> np.ndarray:
        """ Returns the covariance matrix after the measurement update

        Attributes:
            delta_t: change in time
            delta_u: displacement vector
            s: state vector
            Hs: jacobian of h(s, u) with respect to s
            Qs: nondeterministic noise matrix
            
        :param sigma: covariance matrix
        :param u: input vector
        
        :return: covariance matrix
        """
        marker : Marker
        if len(self.marker_list.data) > 0:
            for marker in self.markers.markers:
                
                marker_id = marker.id
                marker_position = self.get_marker_position(marker_id)
                real_marker_position = self.get_real_marker_position(marker_id)
                s =  self.__s # get the state vector
                print()
                delta_x = marker_position[0][0] + .1 # calculate the change in x
                delta_y = marker_position[1][0] # calculate the change in y
                p = delta_x**2 + delta_y**2 # calculate the change in distance
                z = self.calculate_observation_matrix(delta_x, delta_y, p, s) # calculate the observation matrix
                G = self.linearised_observation_matrix(delta_x, delta_y, p) # calculate the linearised observation matrix
                Z = G @ sigma @ G.T + np.array([[0.1, 0],[0, 0.02]]) # calculate the Z matrix
                K = sigma @ G.T @ np.linalg.inv(Z) # calculate the K matrix
                delta_real = np.array([[real_marker_position[0][0]] - s[0][0], [real_marker_position[1][0]] - s[1][0]])
                p2 = delta_real[0][0]**2 + delta_real[1][0]**2
                z2 = self.calculate_real_observation_matrix(delta_real[0][0], delta_real[1][0], p2, s)
                #rospy.loginfo(f"\nreal marker : {z2.T},\ndetected marker : {z.T},\nrobot_position : {s.T},\nmarker_position : {real_marker_position.T}")
                s = s + K @ (z-z2) # calculate the state vector
                sigma = (np.identity(3) - K @ G) @ sigma # calculate the covariance matrix
                self.__s = s # save the state vector

        return sigma
    
    def broadcast_transform(self, pose: Pose):
        #Initialize broadcaster
        br = TransformBroadcaster()
        t = TransformStamped()
        #Fill the transform with the position and orientations
        t.header.stamp = rospy.Time.now()
        #Frame names
        t.header.frame_id = "rviz_puzzlebot/base_link"
        t.child_frame_id = "rviz_puzzlebot/chassis"
        t.transform.translation = Vector3(pose.position.x, pose.position.y, 0)
        t.transform.rotation= pose.orientation
        #Send transform
        br.sendTransform(t)

    def publish_stamp(self, s) -> None:
        poseStamped = PoseStamped()
        poseStamped.pose.position.x = s[0][0]
        poseStamped.pose.position.y = s[1][0]
        #angle to quaternion using tf2
        q= quaternion_from_euler(0, 0, s[2][0])

        poseStamped.pose.orientation.x = q[0]
        poseStamped.pose.orientation.y = q[1]
        poseStamped.pose.orientation.z = q[2]
        poseStamped.pose.orientation.w = q[3]

        poseStamped.header.frame_id = "rviz_puzzlebot/base_link"
        self.broadcast_transform(poseStamped.pose)
        self.robot_pose_publisher.publish(poseStamped)

    def get_matrix_R(self, observation_matrix : np.ndarray) -> np.ndarray:
        """Returns the matrix R

        :return: R
        """
        sigma_r = self.sigma_r * observation_matrix[0][0]
        sigma_phi = self.sigma_phi * observation_matrix[1][0]
        return np.array([[sigma_r**2, 0],
                         [0, sigma_phi**2]])
    
    def linearised_observation_matrix(self, delta_x : float, delta_y : float, p :float) -> np.ndarray:
        """ Returns the linearised observation matrix

        :param delta_x: change in x
        :param delta_y: change in y
        :param p: change in distance

        :return: linearised observation matrix
        """
        return np.array([[-delta_x/sqrt(p), -delta_y/sqrt(p), 0],
                         [delta_y/p, -delta_x/p, -1]])

    def calculate_real_observation_matrix(self, delta_x : float, delta_y : float, p : float, s : np.ndarray) -> np.ndarray:
        """ Returns the observation matrix

        :param delta_x: change in x
        :param delta_y: change in y
        :param p: change in distance
        :param s: state vector

        :return: observation matrix z
        """
        theta = s[2][0]
        error_angle = atan2(delta_y, delta_x)-theta
        #set the angle between -pi and pi
        while error_angle > pi:
            if error_angle > pi:
                error_angle -= 2*pi
            elif error_angle < -pi:
                error_angle += 2*pi

        return np.array([[sqrt(p)],
                         [error_angle]])
        
    def calculate_observation_matrix(self, delta_x : float, delta_y : float, p : float, s : np.ndarray) -> np.ndarray:
        """ Returns the observation matrix

        :param delta_x: change in x
        :param delta_y: change in y
        :param p: change in distance
        :param s: state vector

        :return: observation matrix z
        """
        return np.array([[sqrt(p)],
                         [atan2(delta_y, delta_x)]])
    
    def get_real_marker_position(self, marker_id : int) -> np.ndarray:
        """ Returns the real position of the marker

        :param marker_id: id of the marker

        :return: position of the marker
        """
        #rosservice call /gazebo/get_model_state '{model_name: "marker_{marker_id}}"}'
        markerState : GetModelStateResponse
        modelRequest = GetModelStateRequest()
        modelRequest.model_name = f'Aruco tag{marker_id}'
        markerState = self.get_model_state(modelRequest)   
        return np.array([[markerState.pose.position.x],
                            [markerState.pose.position.y]]) 
        
    def get_marker_position(self, marker_id : int) -> np.ndarray:
        """ Returns the position of the marker

        :param marker_id: id of the marker

        :return: position of the marker
        """
        marker : Marker
        for marker in self.markers.markers:
            if marker.id == marker_id:
                # x = marker.pose.pose.position.z +.1
                # y = -marker.pose.pose.position.x
                # pose : PoseStamped
                # pose = self.transform_pose(marker.pose.pose, "rviz_puzzlebot/chassis", "rviz_puzzlebot/base_link")
                return np.array([[marker.pose.pose.position.z], [-marker.pose.pose.position.x]])
        
        return np.array([[marker.pose.pose.position.z], [-marker.pose.pose.position.x]])
    
    def transform_pose(self, input_pose : Pose, from_frame : str, to_frame : str) -> PoseStamped:
        # **Assuming /tf2 topic is being broadcasted

        pose_stamped = PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time(0)
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        try:
            # Give the listener some time to receive the transform
            self.listener.waitForTransform(to_frame, from_frame, rospy.Time(0), rospy.Duration(4.0))
            # Now transform the pose
            transformed_pose = self.listener.transformPose(to_frame, pose_stamped)
            return transformed_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.loginfo(e)
            return None

    def calculate_odometry(self) -> None:
        u = self.get_u_vector() # Get the input vectors
        #prediction_step
        sigma = self.priori_covariance(self.__sigma, u)
        #correction_step
        sigma = self.posteriori_covariance(sigma)

        self.odom_message.pose.pose.position = Point(
            self.__s[0][0], self.__s[1][0], self.r)
        q = quaternion_from_euler(0, 0, self.__s[2][0])
        self.odom_message.header.stamp = rospy.Time.now()  # time stamp
        self.odom_message.pose.pose.orientation.x = q[0]
        self.odom_message.pose.pose.orientation.y = q[1]
        self.odom_message.pose.pose.orientation.z = q[2]
        self.odom_message.pose.pose.orientation.w = q[3]
        self.odom_message.pose.covariance[0] = sigma[0][0]
        self.odom_message.pose.covariance[1] = sigma[0][1]
        self.odom_message.pose.covariance[5] = sigma[0][2]
        self.odom_message.pose.covariance[6] = sigma[1][0]
        self.odom_message.pose.covariance[7] = sigma[1][1]
        self.odom_message.pose.covariance[11] = sigma[1][2]
        self.odom_message.pose.covariance[30] = sigma[2][0]
        self.odom_message.pose.covariance[31] = sigma[2][1]
        self.odom_message.pose.covariance[35] = sigma[2][2]
        self.odom_message.twist.twist.linear.x = u[0]
        self.odom_message.twist.twist.angular.z = u[1]
        self.publish_stamp(self.__s)
        self.__sigma = sigma
        self.odom_publisher.publish(self.odom_message)

    def __fill_odomerty(self) -> Odometry:
        """Returns the odometry message. THis is called in the init function

        :return: odometry message
        """
        odometry = Odometry()
        odometry.header.stamp = rospy.Time.now()  # time stamp
        odometry.header.frame_id = "world"  # parent frame (joint)
        odometry.child_frame_id = "rviz_puzzlebot/base_link"  # child frame
        # position of the robot “x” w.r.t “parent frame”
        odometry.pose.pose.position.x = 0.0
        # position of the robot “x” w.r.t “parent frame”
        odometry.pose.pose.position.y = 0.0
        # position of the robot “x” w.r.t “parent frame”
        odometry.pose.pose.position.z = self.r
        # Orientation quaternion “x” w.r.t “parent frame”
        odometry.pose.pose.orientation.x = 0.0
        # Orientation quaternion “y” w.r.t “parent frame”
        odometry.pose.pose.orientation.y = 0.0
        # Orientation quaternion “z” w.r.t “parent frame”s
        odometry.pose.pose.orientation.z = 0.0
        # Orientation quaternion “w” w.r.t “parent frame”
        odometry.pose.pose.orientation.w = 0.0
        # Position Covariance 6x6 matrix (empty for now)
        odometry.pose.covariance = [0]*36

        odometry.twist.twist.linear.x = 0.0  # Linear velocity “x”
        odometry.twist.twist.linear.y = 0.0  # Linear velocity “y”
        odometry.twist.twist.linear.z = 0.0  # Linear velocity “z”
        # Angular velocity around x axis (roll)
        odometry.twist.twist.angular.x = 0.0
        # Angular velocity around x axis (pitch)
        odometry.twist.twist.angular.y = 0.0
        # Angular velocity around x axis (yaw)
        odometry.twist.twist.angular.z = 0.0
        # Velocity Covariance 6x6 matrix (empty for now)
        odometry.twist.covariance = [0]*36
        return odometry

    def run(self) -> None:
        self.calculate_odometry()


def main():
    rospy.init_node('localisation', anonymous=True)
    rate = rospy.Rate(30)
    model = Odom()
    while not rospy.is_shutdown():
        model.run()
        rate.sleep()


if (__name__ == "__main__"):
    main()
