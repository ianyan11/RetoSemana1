import numpy as np
from math import cos, sin, pi, atan2, sqrt
class Odom():
    def __init__(self):
        self.kr = .6
        self.kl = .6
        self.sigma_r = 10
        self.sigma_phi = 10
        self.kl = .6
        self.wl = 0
        self.wr = 0
        self.l = .09
        self.r = .05
        self.__s = np.array([[0],[0],[0]]) # State vector
        self.__sigma = np.array([[0,0,0],[0,0,0],[0,0,0]]) # Covariance matrix


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

    def get_u_inverse(self, u: np.ndarray):
        """Returns the wheel speed from the input vector u where u = [v, w]

        Attributes:
            wl: left wheel velocity
            wr: right wheel velocity

        :param u: input vector

        :return: u_inverse
        """
        self.wl = (2*u[0] - u[1]*self.l) / (2*self.r)
        self.wr = (2*u[0] + u[1]*self.l) / (2*self.r)
        return self.wl, self.wr

    def __calulate_delta_t(self) -> float:
        """Returns the change in time delta_t

        :return: delta_t
        """
        return .1

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
                      [delta_theta+ theta]])
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

    def get_matrix_Q(self, s: np.ndarray, delta_t:float , u) -> np.ndarray:
        """Returns the nondeterministisc error matrix Q.

        :param s: state vector

        :return: Q
        """
        return np.array([[.5, 0.01, 0.01],[0.01, .5, 0.01],[0.01, 0.01, .2]])

    def calculate_noise_matrix(self) -> None:
        """Returns the noise matrix

        :return: noise matrix
        """
        return np.array([[.5, 0.01, 0.01],[0.01, .5, 0.01],[0.01, 0.01, .2]])

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
        Qs = self.get_matrix_Q(s, delta_t, u) # Get the Qs matrix
        sigma = Hs @ self.__sigma @ Hs.T + Qs # Calculate the covariance matrix
        self.__s = s # save the state vector
        return sigma # return the covariance matrix

    def posteriori_covariance(self, sigma : np.ndarray, delta_x1 : float, delta_y1 : float, z2) -> np.ndarray:
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
        s =  self.__s # get the state vector
        p = delta_x1**2 + delta_y1**2 # calculate the change in distance
        z = self.calculate_real_observation_matrix(delta_x1, delta_y1, p, s) # calculate the observation matrix
        G = self.linearised_observation_matrix(delta_x1, delta_y1, p) # calculate the linearised observation matrix
        Z = G @ sigma @ G.T + np.array([[0.1, 0],[0, 0.02]]) # calculate the Z matrix
        K = sigma @ G.T @ np.linalg.inv(Z) # calculate the K matrix
        #rospy.loginfo(f"\nreal marker : {z2.T},\ndetected marker : {z.T},\nrobot_position : {s.T},\nmarker_position : {real_marker_position.T}")
        s = s + K @ (z2 - z) # calculate the state vector
        sigma = (np.identity(3) - K @ G) @ sigma # calculate the covariance matrix
        self.__s = s # save the state vector

        return sigma

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
        #set the angle between -pi and piss
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
        #theta = s.T[2]
        arr = np.array([[sqrt(p)],
                         [atan2(delta_y, delta_x)]])
        return arr
    
if __name__ == "__main__":
    odometry = Odom()
    sigma = np.empty((3, 3)) # Covariance matrix
    sigma = odometry.priori_covariance(sigma, np.array([1, 1])) # Calculate the covariance matrix
    print(sigma)
    sigma = odometry.posteriori_covariance(sigma, 2.9, 4.0, np.array([[4.87], [0.8]])) # Calculate the covariance matrix
    print(sigma)
    sigma = odometry.priori_covariance(sigma, np.array([1, 1])) # Calculate the covariance matrix
    print(sigma)
    sigma = odometry.posteriori_covariance(sigma, 2.7796, 3.9285, np.array([[4.8124], [0.7184]])) # Calculate the covariance matrix
    print(sigma)
