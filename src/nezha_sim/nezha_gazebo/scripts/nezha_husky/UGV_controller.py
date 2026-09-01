#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
"""
Robot Controllers Module

This module provides separate controller classes for UGV, UUV, and UAV operations.
Each controller can be used independently or combined in a main control script.
"""

import rospy
import math
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header


class UGVController:
    def __init__(self, namespace="/nezha_husky"):
        """
        Initialize UGV controller

        Args:
            namespace (str): Robot namespace for topics
        """
        topic_name = f"{namespace}/cmd_vel"
        self.cmd_pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        self.linear_speed = 5  # m/s
        self.angular_speed = 0.0  # rad/s

        # Wait for the publisher to be ready
        rospy.sleep(0.5)

        # Verify topic connection
        rospy.loginfo(f"UGV Controller initialized, publishing to: {topic_name}")
        rospy.loginfo(f"Current subscribers: {self.cmd_pub.get_num_connections()}")

    def drive_forward(self, speed=None):
        """
        Drive forward at specified speed

        Args:
            speed (float): Linear speed in m/s (uses default if None)
        """
        if speed is None:
            speed = self.linear_speed

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        # Debug log
        rospy.logdebug(f"Published cmd_vel: linear.x={speed}, angular.z=0.0")

    def turn(self, angular_speed):
        """
        Turn at specified angular speed

        Args:
            angular_speed (float): Angular speed in rad/s
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

    def drive_and_turn(self, linear_speed, angular_speed):
        """
        Drive and turn simultaneously

        Args:
            linear_speed (float): Linear speed in m/s
            angular_speed (float): Angular speed in rad/s
        """
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

    def stop(self):
        """Stop all movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)