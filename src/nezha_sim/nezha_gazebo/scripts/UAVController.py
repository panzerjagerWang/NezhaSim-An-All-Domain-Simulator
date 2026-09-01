#!/usr/bin/env python
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Pose, PoseStamped, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Float32, Header
from mav_msgs.msg import Actuators
from sensor_msgs.msg import Imu


def euler_from_quaternion(quat):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw).

    Args:
        quat (iterable): A list or tuple of four elements (x, y, z, w)

    Returns:
        tuple: A tuple (roll, pitch, yaw) with the Euler angles in radians.
    """
    x, y, z, w = quat

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class UAVController:
    """
    UAV controller providing waypoint navigation, altitude hold, takeoff, attitude change, and landing.
    """

    def __init__(self, debug=False):
        self.debug = debug

        # Subscribe to the UAV pose (message type Pose, published on /nezha/ground_truth/pose)
        rospy.Subscriber("/nezha_mini/ground_truth/pose", Pose, self.pose_callback)
        self.throttle_pub = rospy.Publisher('/nezha_mini/command/throttle_percent', Float32, queue_size=1)
        # Subscribe to motor status information
        rospy.Subscriber("/nezha_mini/command/motor_speed", Actuators, self.motor_speed_callback)

        # Publish control commands
        self.pose_cmd_pub = rospy.Publisher("/nezha_mini/command/pose", PoseStamped, queue_size=10)
        self.trajectory_pub = rospy.Publisher("/nezha_mini/command/trajectory", MultiDOFJointTrajectory, queue_size=10)

        # Initialize pose and attitude
        self.current_pose = None
        self.current_z = None
        self.current_yaw = None
        self.current_motor_speed = None

        # Target setpoints
        self.target_position = None
        self.target_height = 10.0  # Target altitude (unit: meters)
        self.target_yaw = 0.0  # Target attitude (unit: radians)

        # Wait for subscribers and publishers to be ready
        rospy.sleep(1.0)

        # Check ROS connections in debug mode
        if self.debug:
            self._check_connections()

        rospy.loginfo("UAVController initialization complete.")

    def _check_connections(self):
        """Debug helper: check the connection status of ROS topics"""
        topics = ['/nezha_mini/ground_truth/pose', '/nezha_mini/command/pose', '/nezha_mini/command/trajectory']

        rospy.loginfo("Checking ROS topic connections...")
        for topic in topics:
            info = rospy.get_published_topics(topic)
            if info:
                rospy.loginfo(f"Topic {topic} available: {info}")
            else:
                rospy.logwarn(f"Topic {topic} is not publishing any data")

    def pose_callback(self, msg):
        """Handle /nezha_mini/ground_truth/pose messages, updating the current z and yaw."""
        self.current_pose = msg
        self.current_z = msg.position.z
        # Quaternion to Euler angles (only yaw is of interest)
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        (_, _, yaw) = euler_from_quaternion(quat)
        self.current_yaw = yaw

        if self.debug and self.current_pose is not None:
            rospy.loginfo(f"Current position: x={self.current_pose.position.x:.2f}, "
                          f"y={self.current_pose.position.y:.2f}, z={self.current_z:.2f}, "
                          f"yaw={math.degrees(self.current_yaw):.2f}°")

    def motor_speed_callback(self, msg):
        """Handle motor status information"""
        self.current_motor_speed = msg.angular_velocities
        if self.debug:
            rospy.loginfo(f"Motor status: {self.current_motor_speed}")

    def waypoint_navigation(self, target_position):
        """
        Navigate horizontally to the specified target position.
        """
        if self.current_pose is None:
            rospy.logwarn("Cannot navigate: current pose is unknown")
            return False

        # Compute the error between the target position and the current position
        waypoint_error = self.compute_waypoint_error(target_position)

        if waypoint_error > 0.1:  # Apply control when the target error exceeds the threshold
            self.move_to_waypoint(target_position)
            rospy.loginfo("Navigating to target waypoint: target error=%.2f m", waypoint_error)
            return False
        else:
            rospy.loginfo("Reached target waypoint, target error=%.2f m", waypoint_error)
            return True

    def move_to_waypoint(self, target_position, duration=2.0, rate=10):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = target_position.x
        pose_msg.pose.position.y = target_position.y
        pose_msg.pose.position.z = target_position.z

        if self.current_pose:
            pose_msg.pose.orientation = self.current_pose.orientation
        else:
            pose_msg.pose.orientation.w = 1.0

        rate_obj = rospy.Rate(rate)
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration and not rospy.is_shutdown():
            pose_msg.header.stamp = rospy.Time.now()
            self.pose_cmd_pub.publish(pose_msg)
            rate_obj.sleep()

    def set_target_height(self, height):
        """
        Set the target altitude and command the UAV to hold it.
        """
        self.target_height = height
        rospy.loginfo("Setting target altitude: %.2f m", self.target_height)

        if self.current_pose is not None:
            # Send the altitude control command
            target_position = Pose()
            target_position.position.x = self.current_pose.position.x
            target_position.position.y = self.current_pose.position.y
            target_position.position.z = height
            return self.move_to_waypoint(target_position)
        else:
            rospy.logwarn("Cannot set altitude: current position is unknown")
            return False

    def maintain_height(self):
        """
        Altitude hold: command the UAV to maintain the target altitude.
        """
        if self.current_z is None:
            rospy.logwarn("Cannot maintain altitude: current altitude is unknown")
            return False

        height_error = self.target_height - self.current_z
        if abs(height_error) > 0.1:
            # Use position control to adjust the altitude
            if self.current_pose:
                target_position = Pose()
                target_position.position.x = self.current_pose.position.x
                target_position.position.y = self.current_pose.position.y
                target_position.position.z = self.target_height
                self.move_to_waypoint(target_position)
                rospy.loginfo("Adjusting altitude, current altitude: %.2f m, target altitude: %.2f m", self.current_z, self.target_height)
            return False
        else:
            if self.debug:
                rospy.loginfo("Target altitude reached %.2f m", self.current_z)
            return True

    def takeoff(self):
        """
        Takeoff: command the UAV to take off to the target altitude.
        """
        rospy.loginfo("Starting takeoff...")

        # Wait until the current position is received
        timeout = 10  # 10-second timeout
        start_time = rospy.Time.now().to_sec()
        while self.current_pose is None:
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logwarn("Timed out waiting for position data")
                return False
            rospy.loginfo("Waiting for position data...")
            rospy.sleep(0.5)

        # Set the takeoff target position (same as current position, but at a higher altitude)
        takeoff_target = Pose()
        takeoff_target.position.x = self.current_pose.position.x
        takeoff_target.position.y = self.current_pose.position.y
        takeoff_target.position.z = self.target_height

        rospy.loginfo(f"Current altitude: {self.current_z:.2f}m, ascending to target altitude {self.target_height:.2f}m...")

        # Send the takeoff command
        success = self.move_to_waypoint(takeoff_target)
        if not success:
            return False

        # Wait until the target altitude is reached
        timeout = 30  # allow 30 seconds to complete takeoff
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < timeout:
            if self.maintain_height():
                rospy.loginfo("Takeoff complete, target altitude reached: %.2f m", self.current_z)
                return True
            rospy.sleep(0.5)

        rospy.logwarn("Takeoff timed out, target altitude not reached")
        return False

    def land(self):
        """
        Landing: command the UAV to descend slowly and land.
        """
        rospy.loginfo("Starting landing...")

        if self.current_pose is None:
            rospy.logwarn("Cannot land: current position is unknown")
            return False

        # Create the landing target position (same as current position, but altitude 0)
        landing_target = Pose()
        landing_target.position.x = self.current_pose.position.x
        landing_target.position.y = self.current_pose.position.y
        landing_target.position.z = 0.0

        # Save the previous target altitude
        previous_target_height = self.target_height
        # Set the target altitude to 0
        self.target_height = 0.0

        # Send the landing command
        success = self.move_to_waypoint(landing_target)
        if not success:
            self.target_height = previous_target_height  # restore the previous target altitude
            return False

        # Wait until the ground is reached
        timeout = 30  # allow 30 seconds to complete landing
        start_time = rospy.Time.now().to_sec()
        while self.current_z is not None and self.current_z > 0.1:
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logwarn("Landing timed out")
                self.target_height = previous_target_height  # restore the previous target altitude
                return False
            rospy.loginfo("Landing in progress, current altitude: %.2f m", self.current_z)
            rospy.sleep(0.5)

        rospy.loginfo("Landing complete")
        self.target_height = previous_target_height  # restore the previous target altitude
        return True

    def change_attitude(self, target_yaw):
        """
        Change attitude: adjust the UAV's yaw angle.
        """
        self.target_yaw = target_yaw
        # Send the attitude control command
        trajectory_msg = MultiDOFJointTrajectory()
        trajectory_msg.header.stamp = rospy.Time.now()
        trajectory_msg.header.frame_id = "world"

        # Add a joint name
        trajectory_msg.joint_names = ["base_link"]

        # Create a trajectory point
        traj_point = MultiDOFJointTrajectoryPoint()

        # Create a Transform object
        transform = Transform()
        # Keep the current position
        if self.current_pose:
            transform.translation.x = self.current_pose.position.x
            transform.translation.y = self.current_pose.position.y
            transform.translation.z = self.current_pose.position.z

        # Set the target rotation (yaw only)
        transform.rotation.w = math.cos(self.target_yaw / 2)
        transform.rotation.z = math.sin(self.target_yaw / 2)

        # Add the transform
        traj_point.transforms.append(transform)

        # Add the timestamp
        traj_point.time_from_start = rospy.Duration(1.0)

        # Add the trajectory point to the trajectory message
        trajectory_msg.points.append(traj_point)

        # Publish the trajectory message
        self.trajectory_pub.publish(trajectory_msg)

        rospy.loginfo("Changing attitude, target yaw: %.2f°", math.degrees(self.target_yaw))
        return True
    def set_throttle(self, percent):
        """
        Set the throttle percentage (0~100); published to /nezha/command/throttle_percent
        """
        self.throttle_pub.publish(Float32(percent))
        rospy.loginfo(f"[UAVController] Throttle set to {percent}%")
    def compute_waypoint_error(self, target_position):
        """
        Compute the error between the current pose and the target waypoint.
        """
        if self.current_pose is None:
            return float('inf')

        x_error = target_position.x - self.current_pose.position.x
        y_error = target_position.y - self.current_pose.position.y
        z_error = target_position.z - self.current_pose.position.z

        return math.sqrt(x_error ** 2 + y_error ** 2 + z_error ** 2)


def main():
    """
    Main function: demonstrate the UAV control features
    """
    # Initialize the ROS node
    rospy.init_node('uav_controller_demo', anonymous=True)
    rospy.loginfo("UAV control demo program starting...")

    # Create a UAV controller instance with debug mode enabled
    uav = UAVController(debug=True)

    # Wait for the controller to finish initializing
    rospy.sleep(2.0)

    # Check whether the UAV has received position data
    timeout = 20  # 20-second timeout
    start_time = rospy.Time.now().to_sec()
    while uav.current_pose is None:
        if rospy.Time.now().to_sec() - start_time > timeout:
            rospy.logerr("Unable to obtain UAV position data; check whether the topic /nezha/ground_truth/pose is being published normally")
            return
        rospy.loginfo("Waiting to receive UAV position data...")
        rospy.sleep(1.0)

    # Use a low takeoff altitude for testing
    uav.target_height = 2.0

    # Execute takeoff
    if not uav.takeoff():
        rospy.logerr("Takeoff failed")
        return

    rospy.loginfo("Takeoff succeeded, waiting 5 seconds...")
    rospy.sleep(5.0)

    # Create a target position
    target = Pose()
    target.position.x = 1.0
    target.position.y = 0.0
    target.position.z = 2.0  # keep the current altitude

    # Navigate to the target position
    rospy.loginfo("Starting navigation to target waypoint (1.0, 0.0, 2.0)...")

    # Loop until the target is reached or timeout
    timeout = 30  # 30-second timeout
    start_time = rospy.Time.now().to_sec()
    reached = False

    while not reached and not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_time < timeout:
        reached = uav.waypoint_navigation(target.position)
        rospy.sleep(0.5)

    if reached:
        rospy.loginfo("Successfully reached target waypoint!")
    else:
        rospy.logwarn("Navigation timed out, target waypoint not reached")

    # Change attitude (rotate 90 degrees)
    rospy.loginfo("Changing attitude, rotating 90 degrees...")
    uav.change_attitude(math.pi / 2)
    rospy.sleep(5.0)

    # Return to start
    rospy.loginfo("Returning to start (0.0, 0.0, 2.0)...")
    target.position.x = 0.0
    target.position.y = 0.0

    # Loop until the start is reached or timeout
    timeout = 30  # 30-second timeout
    start_time = rospy.Time.now().to_sec()
    reached = False

    while not reached and not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_time < timeout:
        reached = uav.waypoint_navigation(target.position)
        rospy.sleep(0.5)

    # Execute landing
    rospy.loginfo("Starting landing...")
    uav.land()

    rospy.loginfo("Demo complete!")

    # Keep the node running until it is shut down
    rospy.spin()

def main():
    """
    Main function: demonstrate the UAV control features
    """
    # Initialize the ROS node
    rospy.init_node('uav_controller_demo', anonymous=True)
    rospy.loginfo("UAV control demo program starting...")

    uav = UAVController(debug=True)
    rospy.sleep(2.0)

    timeout = 20  # maximum time to wait for the pose topic
    start_time = rospy.Time.now().to_sec()
    while uav.current_pose is None:
        if rospy.Time.now().to_sec() - start_time > timeout:
            rospy.logerr("Unable to obtain UAV position data; check whether the topic /nezha/ground_truth/pose is being published normally")
            return
        rospy.loginfo("Waiting to receive UAV position data...")
        rospy.sleep(1.0)

    uav.target_height = 2.0  # low altitude for testing

    # ---- Takeoff retry mechanism ----
    max_attempts = 0   # 0 means retry indefinitely; set to a positive integer to limit attempts
    attempt = 0
    takeoff_success = False
    while not takeoff_success and (max_attempts == 0 or attempt < max_attempts) and not rospy.is_shutdown():
        attempt += 1
        rospy.loginfo(f"Takeoff attempt #{attempt}...")
        takeoff_success = uav.takeoff()
        if not takeoff_success:
            rospy.logwarn("Takeoff failed, retrying in 3 seconds...")
            rospy.sleep(3.0)
    if not takeoff_success:
        rospy.logerr("Takeoff still unsuccessful after multiple attempts; exiting.")
        return

    rospy.loginfo("Takeoff succeeded, waiting 5 seconds...")
    rospy.sleep(5.0)

    # The flow below matches the original logic...
    target = Pose()
    target.position.x = 1.0
    target.position.y = 0.0
    target.position.z = 2.0
    rospy.loginfo("Starting navigation to target waypoint (1.0, 0.0, 2.0)...")

    timeout = 30
    start_time = rospy.Time.now().to_sec()
    reached = False

    while not reached and not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_time < timeout:
        reached = uav.waypoint_navigation(target.position)
        rospy.sleep(0.5)

    if reached:
        rospy.loginfo("Successfully reached target waypoint!")
    else:
        rospy.logwarn("Navigation timed out, target waypoint not reached")

    rospy.loginfo("Changing attitude, rotating 90 degrees...")
    uav.change_attitude(math.pi / 2)
    rospy.sleep(5.0)

    rospy.loginfo("Returning to start (0.0, 0.0, 2.0)...")
    target.position.x = 0.0
    target.position.y = 0.0

    timeout = 30
    start_time = rospy.Time.now().to_sec()
    reached = False
    while not reached and not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_time < timeout:
        reached = uav.waypoint_navigation(target.position)
        rospy.sleep(0.5)
    rospy.loginfo("Starting landing...")
    uav.land()
    rospy.loginfo("Demo complete!")
    rospy.spin()

if __name__ == "__main__":
    main()
