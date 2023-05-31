#!/usr/bin/env python3
import rospy
import tf2_ros
from filterpy.kalman import ExtendedKalmanFilter 
import numpy as np
from math import cos, sin, pi
from typing import Tuple
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
class PuzzlebotKalmanFilter:
    """This class represents a Kalman filter for a Puzzlebot.

    Attributes:
        kf (ExtendedKalmanFilter): The underlying Kalman filter.
        r (float): The radius.
        l (float): The wheel distance.
        x (numpy.array): The state vector (x, y, theta).
        priori_x (numpy.array): The priori state vector (x, y, theta).
    """

    def __init__(self, radius, wheel_distance):
        """Initializes the PuzzlebotKalmanFilter.

        Args:
            radius (float): The radius.
            wheel_distance (float): The wheel distance.
        """
        self.robot_name = "rviz_puzzlebot"
        self.kf = self.initialize_kalman_filter()
        self.r = radius
        self.l = wheel_distance
        self.x = np.array([0, 0, 0]).T # x, y, theta
        self.kr, self.kl = 0.6, 0.6 #error constants
        rospy.init_node('puzzlebot_kalman_filter', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def initialize_kalman_filter(self):
        """Initializes the Kalman filter.

        Returns:
            ExtendedKalmanFilter: The initialized Kalman filter.
        """
        kf = ExtendedKalmanFilter(dim_x=3, dim_z=3)

    def set_displacement_increment(self, omegaR: float, omegaL: float, dt: float) -> Tuple[float, float]:
        """Computes the displacement increment for right and left wheel.

        Args:
            omegaR (float): The angular velocity of the right wheel.
            omegaL (float): The angular velocity of the left wheel.
            dt (float): The time step.

        Returns:
            Tuple[float, float]: The displacement increments for the right and left wheel.
        """
        delta_sr = omegaR * dt # displacement of right wheel
        delta_sl = omegaL * dt # displacement of left wheel
        return delta_sr, delta_sl

    def get_displacement_increment(self, delta_sr : float , delta_sl : float) -> float:
        """Computes the mean displacement increment.

        Args:
            delta_sr (float): The displacement increment of the right wheel.
            delta_sl (float): The displacement increment of the left wheel.

        Returns:
            float: The mean displacement increment.
        """
        return (delta_sr, delta_sl)/2
    
    def get_angular_increment(self, delta_sr : float, delta_sl : float) -> float:
        """Computes the angular increment.

        Args:
            delta_sr (float): The displacement increment of the right wheel.
            delta_sl (float): The displacement increment of the left wheel.

        Returns:
            float: The angular increment.
        """
        return (delta_sr - delta_sl)/self.l
    

    def predict_x(self, delta_s: float, delta_theta : float) -> np.ndarray:
        """Predicts the next state vector.

        Args:
            omegaR (int): The angular velocity of the right wheel.
            omegaL (int): The angular velocity of the left wheel.
            dt (float): The time step.

        Returns:
            numpy.array: The predicted state vector.
        """
        theta = self.x[2]
        predict = self.x + np.array([delta_s*cos(theta + delta_theta/2), delta_s*sin(theta + delta_theta/2), delta_theta]).T
        return predict
    
    def calculate_covariance_matrix(self, delta_sr: float, delta_sl : float) -> np.ndarray:
        return np.array([[self.kr*abs(delta_sr), 0], [0, self.kl*abs(delta_sl)]])

    def calculate_matrix_A(self, delta_s : float, delta_theta : float) -> np.ndarray:
        theta = self.x[2]
        return np.array([[1, 0, -delta_s*sin(theta + delta_theta/2)], [0, 1, delta_s*cos(theta + delta_theta/2)], [0, 0, 1]])
    
    def calculate_matrix_W(self, delta_s : float, delta_theta : float) -> np.ndarray:
        theta = self.x[2]
        return np.array(
            [[1/2*cos(theta + delta_theta/2) - delta_s/(2*self.l)*sin(theta+delta_theta/2), 1/2*cos(theta + delta_theta/2) + delta_s/(2*self.l)*sin(theta+delta_theta/2)], 
            [1/2*sin(theta + delta_theta/2) + delta_s/(2*self.l)*cos(theta+delta_theta/2), 1/2*sin(theta + delta_theta/2) - delta_s/(2*self.l)*cos(theta+delta_theta/2)],
            [1/self.l, -1/self.l]])
    
    def get_camera_to_robot_frame_transform(self)-> TransformStamped:
        try:
            transform = self.tfBuffer.lookup_transform(self.robot_name.__add__('/base_link'), self.robot_name.__add__('/camera_link'), rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error in getting transform")
            return None
          
    def get_robot_to_global_frame_transform(self)-> TransformStamped:
        try:
            transform = self.tfBuffer.lookup_transform(self.robot_name.__add__('/base_link'), 'world', rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error in getting transform")
            return None
        
    def get_marker_in_global_frame_transform(self, id)-> TransformStamped:
        try:
            transform = self.tfBuffer.lookup_transform('world', 'marker'.__add__(id), rospy.Time())
            return transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("Error in getting transform")
            return None
    
    def camera_coordinate_frame_to_robot_coordinate_frame(self, priori : np.ndarray) -> np.ndarray:
        """Transforms the measurement vector from the camera coordinate frame to the robot coordinate frame.
        Args:
            z_k (numpy.array): The measurement vector in the camera coordinate frame.
        Returns:
            numpy.array: The measurement vector in the robot coordinate frame.
        """
        r_transform = self.get_camera_to_robot_frame_transform()
        x_r = r_transform.transform.translation.x
        y_r = r_transform.transform.translation.y
        theta_r = euler_from_quaternion([r_transform.transform.rotation.x, r_transform.transform.rotation.y, r_transform.transform.rotation.z, r_transform.transform.rotation.w])[2]
        x_g = priori[0] + x_r*cos(priori[2]) - y_r*sin(priori[2])
        y_g = priori[1] + x_r*sin(priori[2]) + y_r*cos(priori[2])
        theta_g = priori[2] + theta_r

        return np.array([x_g, y_g, theta_g]).T

    def offset_robot_and_marker_in_global_frame(self, z_G : np.ndarray, z_m :  np.ndarray) -> np.ndarray:
        return z_m-z_G
    
    def deviation_values_in_global_frame(self, delta_z_G : np.ndarray,  z_G_theta : float) -> np.ndarray:
        return np.array([delta_z_G[0]*cos(z_G_theta) + delta_z_G[1]*sin(z_G_theta), delta_z_G[0]*sin(z_G_theta) - delta_z_G[1]*cos(z_G_theta), delta_z_G[2]]).T
    
    def epoch(self, omegaR: float, omegaL: float, dt: float, tvec: np.ndarray, rvec: np.ndarray, id : int):
        #Prediction phase
        delta_sr, delta_sl = self.set_displacement_increment(omegaR, omegaL, dt)
        delta_s = self.get_displacement_increment(delta_sr, delta_sl)
        delta_theta = self.get_angular_increment(delta_sr, delta_sl)
        priori_x = self.predict_x(delta_s, delta_theta, dt)
        Qk = self.calculate_covariance_matrix(delta_sr, delta_sl, dt)
        Ak = self.calculate_matrix_A(delta_s, delta_theta, dt)
        Wk = self.calculate_matrix_W(delta_s, delta_theta, dt)
        #Correction phase
        x_c = tvec[0]
        y_c = tvec[1]
        theta_c = rvec[2]
        z_k = np.array([x_c, y_c, theta_c]).T
        z_G = self.camera_coordinate_frame_to_robot_coordinate_frame(priori_x)
        
        z_m = self.get_marker_in_global_frame_transform(id)
        delta_z_G = self.offset_robot_and_marker_in_global_frame(z_G, z_m)
        delta_z_C = self.deviation_values_in_global_frame(delta_z_G, z_G[2])








    




        

