#!/usr/bin/python3

import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import math
from os.path import expanduser

class OdometryCalibrationNode():
    def __init__(self):
        
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        self.ground_truth_x = 0.0
        self.ground_truth_y = 0.0
        self.ground_truth_theta = 0.0

        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2)
        self.fig.suptitle('Calibration plots')
        self.ax1.axis("equal")
        self.ax2.axis("equal")

        self.odometry_topic = rospy.get_param("odometry_topic", "/odometry")
        self.calibration_topic = rospy.get_param("calibration_topic", "/calibrate_odom")
        self.wheel_radius = rospy.get_param("wheel_radius", 0.08)
        self.wheel_dist = rospy.get_param("wheel_dist", 0.43)
        self.fig_name = rospy.get_param("fig_name", "odom_calib")
        self.package_path = rospy.get_param("package_path", expanduser("~"))
        self.left_encoder_list = []
        self.right_encoder_list = []
        self.linear_cf = 1
        self.angle_cf = 1

        self.red = '\u001b[31m'
        self.green = '\u001b[32m'
        self.blue = '\u001b[34m'

    def run(self):
        self.odometry_sub = rospy.Subscriber(self.odometry_topic, Odometry, self.odom_callback)
        self.calibration_sub = rospy.Subscriber(self.calibration_topic, Vector3, self.calibrate_callback)
    
    def calculate_correction_factor(self, pose_x_list, angle_list):
        if len(pose_x_list) == 0 or len(angle_list) == 0:
            print(self.red + "Error: No odometry data")
            return

        self.linear_cf = self.ground_truth_x / pose_x_list[-1]
        self.angle_cf = self.ground_truth_theta / angle_list[-1]
        print(self.green + f"Correction factors:\n Linear: {self.linear_cf} \n Angular: {self.angle_cf}")
        self.velocity_to_pose("_with_cf.png")

    def odom_callback(self, msg):
        self.left_encoder_list.append(msg.twist.twist.angular.x)
        self.right_encoder_list.append(msg.twist.twist.angular.y)

    def calibrate_callback(self, msg):
        self.ground_truth_x = msg.x
        self.ground_truth_y = msg.y
        self.ground_truth_theta = msg.z

        pose_x_list, angle_list = self.velocity_to_pose("_no_cf.png")
        self.calculate_correction_factor(pose_x_list, angle_list)

    def encoder_to_velocity(self, encoder_right, encoder_left):
        # Convert encoder data to linear and angular velocity
        w_r = encoder_right/9.549297
        w_l = encoder_left/9.549297

        # Calculate Linear and Angular velocity for the robot
        v_lin = (self.wheel_radius/2)*(w_r + w_l)*self.linear_cf
        w     = (self.wheel_radius/self.wheel_dist)*(w_r - w_l)*self.angle_cf

        # Calculate x and y component of linear velocity
        v_x = v_lin
        v_y = 0.0

        # Return the linear velocity components(v_x and v_y) and angular velocity(w)
        return v_x, v_y, w

    def velocity_to_pose(self, fig_name_suffix):
        pose_x_list = []
        pose_y_list = []
        angle_list  = []
        count_list  = []
        
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        # Convert angular velocity for angle calibration
        print(self.blue + f"Running with linear CF = {self.linear_cf} and angular CF = {self.angle_cf}")
        for i in range(len(self.left_encoder_list)):
            v_x, v_y, w = self.encoder_to_velocity(self.right_encoder_list[i], self.left_encoder_list[i])
            dt = 0.1
            dtheta = w*dt
            self.pose.theta = (self.pose.theta + dtheta)
            angle_list.append(self.pose.theta*180/math.pi)
            count_list.append(i)
        
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0
        # Convert Encoder data to Pose data for linear calibration
        for i in range(len(self.left_encoder_list)):
            v_x, v_y, w = self.encoder_to_velocity(self.right_encoder_list[i], self.left_encoder_list[i])
            dt = 0.1
            dx = (v_x*math.cos(self.pose.theta) - v_y*math.sin(self.pose.theta))*dt
            dy = (v_x*math.sin(self.pose.theta) + v_y*math.cos(self.pose.theta))*dt
            dtheta = w*dt
            self.pose.x = self.pose.x + dx
            self.pose.y = self.pose.y + dy
            self.pose.theta = (self.pose.theta + dtheta)
            pose_x_list.append(self.pose.x)
            pose_y_list.append(self.pose.y)

        # Plot the Pose values for linear calibration
        self.ax1.clear()
        self.ax2.clear()
        self.ax1.plot(pose_x_list, pose_y_list, 'b-')
        self.ax2.plot(count_list, angle_list, 'r-')
        plt.savefig(self.package_path + "/" +self.fig_name + fig_name_suffix)
        print("Saved plot to: " + self.package_path + "/" + self.fig_name + fig_name_suffix)

        return pose_x_list, angle_list

# Driver function
def main():
    rospy.init_node("odometry_calibration_node")
    odometry_calibration_node = OdometryCalibrationNode()
    odometry_calibration_node.run()
    rospy.spin()

if __name__ == "__main__":
    main()