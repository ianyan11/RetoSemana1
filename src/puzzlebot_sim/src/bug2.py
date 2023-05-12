#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import atan2, sqrt, pi, tan
from tf.transformations import euler_from_quaternion
from puzzlebot_sim.srv import SetGoal, SetGoalResponse

class Bug2Algorithm:
    # Define class-level constants
    OBSTACLE_DISTANCE_THRESHOLD = 0.5 # m
    RATE = 60 # Hz
    DESIRED_WALL_DISTANCE = 0.5 # m
    MAX_TURN_SPEED = 0.5 # rad/s
    KP = 0.75 # Proportional gain for turning
    GOAL_REACHED_THRESHOLD = 0.4  # the goal is considered reached if we are closer than this value

    # Initialize ROS node, subscribers, and publishers
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('bug2_algorithm', anonymous=True)
        # Subscribers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.robot_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_pub_gazebo = rospy.Publisher('/gazebo_puzzlebot/cmd_vel', Twist, queue_size=10)

        # Service
        self.goal_service = rospy.Service('/puzzlebot_setGoal', SetGoal, self.set_goal_service_callback)

        self.rate = rospy.Rate(self.RATE)

        self.laser_scan = []
        self.goal_direction = 0.0 # Direction towards goal in RADIANS in world frame
        self.goal_direction_at_obstacle = 0.0 # Direction towards goal in RADIANS in world frame when obstacle was detected
        self.goal_distance = 0.0 # Distance to goal in METERSs
        self.goal = None 
        self.robot_position = Pose() 
        self.current_speed = 0.0
        self.robot_orientation = 0.0 # Robot orientation in RADIANS in world frame
        # Calculate the maximum allowed speed change based on the acceleration limit
        MAX_ACCELERATION = 0.3 # m/s^2
        self.MAX_SPEED_CHANGE = MAX_ACCELERATION / self.RATE  # Assumes rate is in Hz
        
    # Callback function to process LaserScan messages    
    def laser_callback(self, msg: LaserScan):
        self.laser_scan = msg.ranges
        # Implement logic for processing LaserScan messages

    # Callback function to process Odometry messages
    def odom_callback(self, msg: Odometry):
        self.robot_position = msg.pose.pose
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_orientation = self.normalize_angle(yaw)  # You'll need to add this new field
        if self.goal is not None:
            self.calculate_goal_direction()
    
    # Function to calculate the direction towards the goal
    def calculate_goal_direction(self):
        if self.goal is None:
            return
        # Calculate direction towards goal
        delta_x = self.goal.x - self.robot_position.position.x
        delta_y = self.goal.y - self.robot_position.position.y
        self.goal_direction = atan2(delta_y, delta_x)
        self.goal_distance = sqrt(delta_x**2 + delta_y**2)

    # Callback function to process goal setting service requests
    def set_goal_service_callback(self, req):
        self.goal = req.goal
        # Start the bug2 algorithm towards the new goal
        completed = self.run()
        return SetGoalResponse(completed)

    # Function to detect if there is an obstacle ahead of the robot
    def detect_obstacle_ahead(self) -> bool:
        if self.laser_scan is not None:
            for distance in self.laser_scan:
                if distance < self.OBSTACLE_DISTANCE_THRESHOLD:
                    self.goal_direction_at_obstacle = self.goal_direction
                    rospy.loginfo("Obstacle detected" + str(self.laser_scan.index(distance)))
                    return True
        return False
    
    # Function to circumnavigate an obstacle
    def circumnavigate_obstacle(self):
        # Remember the point where the robot encountered the obstacle
        encounter_distance = self.goal_distance
        encounter_direction = self.goal_direction
        robot_position_at_encounter = self.robot_position
        # Circumnavigate the obstacle
        while True:
            self.move_around_obstacle()

            # Check if the robot has intersected the goal line
            self.calculate_goal_direction()
            if (self.has_crossed_line(robot_position_at_encounter.position.x, robot_position_at_encounter.position.y, 
                                      self.robot_position.position.x, self.robot_position.position.y, 
                                      encounter_direction) and self.goal_distance < encounter_distance):
                rospy.loginfo("Goal line intersected")
                break
    
    # Function to check if the robot has crossed the line from the obstacle encounter point to the goal
    def has_crossed_line(self, x1, y1, x2, y2, angle):
        m = tan(angle)  # calculate slope of the line
        return (y2 - y1) > m * (x2 - x1)

    # Function to make the robot move around the obstacle while maintaining a constant distance from it
    def move_around_obstacle(self):
        cmd_vel = Twist()
        # Get the indices of the laser scan that cover from 90 degrees to 0 degrees
        left_indices = range(0, len(self.laser_scan) // 4 + 1) # 0 to 90 degrees
        # Get the minimum distance to the wall on the right
        left_distance = min(self.laser_scan[i] for i in left_indices)
        rospy.loginfo("left_distance: " + str(left_distance))
        # Calculate the error
        error = self.DESIRED_WALL_DISTANCE - left_distance
        # Calculate the turn speed using a P controller
        turn_speed = -self.KP * error
        # Ensure the turn speed is within the allowed limits
        turn_speed = max(min(turn_speed, self.MAX_TURN_SPEED), -self.MAX_TURN_SPEED)

         # Check for obstacles ahead
        if self.laser_scan[0] < self.OBSTACLE_DISTANCE_THRESHOLD:
            # If an obstacle is detected ahead, turn left
            cmd_vel.angular.z = -self.MAX_TURN_SPEED
            cmd_vel.linear.x = 0
        else:
            # Otherwise, continue with the calculated turn speed
            cmd_vel.angular.z = turn_speed
            forward_speed = 0.2-abs(turn_speed)  # move forward
            cmd_vel.linear.x = max(forward_speed, 0)  # ensure we're not moving backwards

        self.set_robot_speed(cmd_vel)


    # Function to make the robot move towards the goal
    def move_towards_goal(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1
        cmd_vel.angular.z = self.goal_direction - self.robot_orientation
        self.set_robot_speed(cmd_vel)

    # Function to set the robot's speed
    def set_robot_speed(self, cmd_vel: Twist):
        # Calculate the desired speed change
        speed_change = cmd_vel.linear.x - self.current_speed

        # If the desired speed change exceeds the maximum, limit it
        if abs(speed_change) > self.MAX_SPEED_CHANGE:
            speed_change = self.MAX_SPEED_CHANGE if speed_change > 0 else -self.MAX_SPEED_CHANGE

        # Update the current speed
        self.current_speed += speed_change
        cmd_vel.linear.x = self.current_speed

        self.cmd_pub.publish(cmd_vel)
        self.cmd_pub_gazebo.publish(cmd_vel)

    # Function to normalize an angle to the range [-pi, pi]
    def normalize_angle(self, angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle
    
    # Main function to run the Bug2 algorithm
    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            if self.detect_obstacle_ahead():
                self.circumnavigate_obstacle()
            elif self.goal_distance < self.GOAL_REACHED_THRESHOLD:
                self.set_robot_speed(Twist())
                rospy.loginfo("Goal reached!")
                return True
            else:
                self.move_towards_goal()
            self.rate.sleep()

# Main function to start the Bug2 algorithm    
if __name__ == '__main__':
    bug2 = Bug2Algorithm()
    rospy.spin()