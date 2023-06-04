#!/usr/bin/env python3
import rospy
import numpy as np
from math import cos, sin
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


class Odom():

    def __init__(self):
        self.kr = .6
        self.kl = .6
        self.wl = 0
        self.wr = 0
        self.l = .09
        self.r = .05
        self.__s = np.empty((3, 1)) # State vector
        self.__sigma = np.empty((3, 3)) # Covariance matrix
        self.__previous_time = rospy.Time.now()
        rospy.Subscriber('/wl', Float32, self.__update_wl)
        rospy.Subscriber('/wr', Float32, self.__update_wr)
        self.odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.odom_message = self.__fill_odomerty()

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
        s = s + np.array([[delta_d*cos(theta)],
                      [delta_d*sin(theta)],
                      [delta_theta]])
        return s

    def get_Hs_matrix(self, s: np.ndarray, delta_u: np.ndarray, delta_t: float) -> np.ndarray:
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

    def calculate_covariance(self, sigma : np.ndarray, u : np.ndarray) -> np.ndarray:
        """ Returns the covariance matrix

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
        Hs = self.get_Hs_matrix(s, delta_u, delta_t) # Get the Hs matrix
        Qs = self.get_matrix_Q(s, delta_t) # Get the Qs matrix
        sigma = Hs @ self.__sigma @ Hs.T + Qs # Calculate the covariance matrix
        self.__s = s # save the state vector
        return sigma # return the covariance matrix

    def calculate_odometry(self) -> None:
        u = self.get_u_vector() # Get the input vectors
        sigma = self.calculate_covariance(self.__sigma, u)

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
