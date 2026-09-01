#!/usr/bin/env python
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# -*- coding: utf-8 -*-

import rospy
import csv
import numpy as np
import signal
import sys
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry


class PIDController:
    """PID controller"""

    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """Reset the PID state"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time):
        """Compute the PID output"""
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0:
            return 0.0

        # Proportional term
        p_term = self.kp * error

        # Integral term (with anti-windup)
        self.integral += error * dt
        if self.output_limit is not None:
            self.integral = np.clip(
                self.integral,
                -self.output_limit / self.ki,
                self.output_limit / self.ki
            )
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Total output
        output = p_term + i_term + d_term

        # Saturation limiting
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)

        self.prev_error = error
        self.prev_time = current_time

        return output
class GlobalTrajectoryController:
    def __init__(self):
        rospy.init_node('global_trajectory_controller', anonymous=True)

        # Publisher: publish trajectory commands
        self.traj_pub = rospy.Publisher('/nezha_f/command/trajectory',
                                        MultiDOFJointTrajectory,
                                        queue_size=10)

        # Subscriber: obtain the current position information
        self.odom_sub = rospy.Subscriber('/nezha_f/ground_truth/odometry',
                                         Odometry,
                                         self.odom_callback)

        # Current state
        self.current_position = Vector3(0, 0, 0)
        self.current_velocity = Vector3(0, 0, 0)

        # Waypoint list
        self.waypoints = []

        # ** Data recording **
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.velocity_x = []
        self.velocity_y = []
        self.velocity_z = []

        self.max_records = 100
        self.recording = False
        self.record_counter = 0  # Total record counter
        self.target_records = 0  # Target number of records (excluding first and last)
        self.current_epoch = 0

        # Data recording flags
        self.first_waypoint_recorded = False
        self.last_waypoint_recorded = False

        # Odometry frequency measurement
        self.odom_callback_count = 0
        self.odom_start_time = None

        rospy.sleep(1.0)
        rospy.loginfo("Global trajectory controller initialized")

    def odom_callback(self, msg):
        """Odometry callback: update the current position and record data"""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear

        # Frequency measurement
        if self.odom_start_time is not None:
            self.odom_callback_count += 1

        # Improved data recording logic: strictly cap the maximum count
        if self.recording and len(self.position_x) < self.max_records:
            self.record_counter += 1

            # Number of intermediate points recorded so far (excluding the manually recorded first/last points)
            # The first point was recorded manually, so current_recorded = len(self.position_x) - 1
            current_recorded = len(self.position_x) - 1

            # Enforce a strict upper-bound check
            if current_recorded >= self.target_records:
                # Target reached; stop recording (wait for the last point to be recorded manually)
                return

            # Compute the number of points that should theoretically be recorded based on the total callback count
            if self.target_records > 0 and self.total_expected_callbacks > 0:
                expected_records = int(
                    (self.record_counter / float(self.total_expected_callbacks)) * self.target_records)

                # Double check: make sure we never exceed the target count
                if current_recorded < expected_records and current_recorded < self.target_records:
                    self.position_x.append(self.current_position.x)
                    self.position_y.append(self.current_position.y)
                    self.position_z.append(self.current_position.z)

                    self.velocity_x.append(self.current_velocity.x)
                    self.velocity_y.append(self.current_velocity.y)
                    self.velocity_z.append(self.current_velocity.z)

                    if len(self.position_x) % 10 == 0:
                        progress = (self.record_counter / float(self.total_expected_callbacks)) * 100
                        rospy.loginfo(
                            f"Recorded {len(self.position_x)}/{self.max_records} data points (progress: {progress:.1f}%)")

    def record_current_state(self):
        """Manually record the current state (position and velocity)"""
        self.position_x.append(self.current_position.x)
        self.position_y.append(self.current_position.y)
        self.position_z.append(self.current_position.z)

        self.velocity_x.append(self.current_velocity.x)
        self.velocity_y.append(self.current_velocity.y)
        self.velocity_z.append(self.current_velocity.z)

        rospy.loginfo(f"✓ Manually recorded data point {len(self.position_x)}/{self.max_records}")

    def save_to_txt(self, data_list, filename, mode='a'):
        """Save the list data to a txt file"""
        try:
            with open(filename, mode) as f:
                formatted_data = '[' + ','.join([f"{value:.6f}" for value in data_list]) + '],\n'
                f.write(formatted_data)
            rospy.loginfo(f"✓ {filename} - Epoch {self.current_epoch} ({len(data_list)} data points total)")
        except Exception as e:
            rospy.logerr(f"✗ Failed to save file {filename}: {e}")

    def save_recorded_data(self):
        """Save the recorded data to 6 separate txt files"""
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Saving flight data for Epoch {self.current_epoch}...")
        rospy.loginfo("-" * 60)

        mode = 'w' if self.current_epoch == 1 else 'a'

        rospy.loginfo("Position data:")
        self.save_to_txt(self.position_x, 'position_x.txt', mode)
        self.save_to_txt(self.position_y, 'position_y.txt', mode)
        self.save_to_txt(self.position_z, 'position_z.txt', mode)

        rospy.loginfo("-" * 60)

        rospy.loginfo("Velocity data:")
        self.save_to_txt(self.velocity_x, 'velocity_x.txt', mode)
        self.save_to_txt(self.velocity_y, 'velocity_y.txt', mode)
        self.save_to_txt(self.velocity_z, 'velocity_z.txt', mode)

        rospy.loginfo("-" * 60)
        rospy.loginfo(f"Epoch {self.current_epoch} data saved successfully!")
        rospy.loginfo("=" * 60)

    def reset_for_next_epoch(self):
        """Reset the data in preparation for the next epoch"""
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.velocity_x = []
        self.velocity_y = []
        self.velocity_z = []

        self.recording = False
        self.record_counter = 0
        self.first_waypoint_recorded = False
        self.last_waypoint_recorded = False

        rospy.loginfo("Data reset; ready for the next epoch...")

    def ensure_exact_record_count(self):
        """Ensure the number of data points is exactly 100"""
        current_count = len(self.position_x)

        if current_count == self.max_records:
            rospy.loginfo(f"✓ Data point count is correct: {current_count}/{self.max_records}")
            return

        if current_count < self.max_records:
            shortage = self.max_records - current_count
            rospy.logwarn(f"⚠ Not enough data points! Current: {current_count}, target: {self.max_records}, missing: {shortage}")
            rospy.loginfo("Collecting additional samples...")

            rate = rospy.Rate(10)
            fill_count = 0

            while len(self.position_x) < self.max_records and not rospy.is_shutdown():
                # Record the current state directly
                self.position_x.append(self.current_position.x)
                self.position_y.append(self.current_position.y)
                self.position_z.append(self.current_position.z)
                self.velocity_x.append(self.current_velocity.x)
                self.velocity_y.append(self.current_velocity.y)
                self.velocity_z.append(self.current_velocity.z)

                rate.sleep()
                fill_count += 1
                if fill_count % 10 == 0:
                    rospy.loginfo(f"Collecting additional samples... {len(self.position_x)}/{self.max_records}")

            rospy.loginfo(f"✓ Additional sampling complete: {len(self.position_x)}/{self.max_records}")

        elif current_count > self.max_records:
            excess = current_count - self.max_records
            rospy.logwarn(f"⚠ Too many data points! Current: {current_count}, target: {self.max_records}, excess: {excess}")
            rospy.loginfo("Trimming excess data...")

            self.position_x = self.position_x[:self.max_records]
            self.position_y = self.position_y[:self.max_records]
            self.position_z = self.position_z[:self.max_records]
            self.velocity_x = self.velocity_x[:self.max_records]
            self.velocity_y = self.velocity_y[:self.max_records]
            self.velocity_z = self.velocity_z[:self.max_records]

            rospy.loginfo(f"✓ Trimming complete: {len(self.position_x)}/{self.max_records}")

    def return_to_start_with_pid(self, timeout=20.0, position_threshold=0.5):
        """
        Use PID control to return precisely to the first waypoint position

        Parameters:
            timeout: timeout (seconds) - increased to 60 seconds
            position_threshold: position error threshold (meters) - relaxed to 0.3 meters
        """
        if len(self.waypoints) == 0:
            rospy.logerr("No waypoint information available; cannot return to start")
            return False

        start_waypoint = self.waypoints[0]
        target_position = Vector3(
            start_waypoint['x'],
            start_waypoint['y'],
            start_waypoint['z']
        )

        rospy.loginfo("=" * 60)
        rospy.loginfo("🏠 Returning to start using PID control...")
        rospy.loginfo(f"  Target position: ({target_position.x:.3f}, "
                      f"{target_position.y:.3f}, {target_position.z:.3f})")
        rospy.loginfo(f"  Current position: ({self.current_position.x:.3f}, "
                      f"{self.current_position.y:.3f}, {self.current_position.z:.3f})")

        # Compute the initial distance
        dx = target_position.x - self.current_position.x
        dy = target_position.y - self.current_position.y
        dz = target_position.z - self.current_position.z
        initial_distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        rospy.loginfo(f"  Initial distance: {initial_distance:.3f} m")
        rospy.loginfo(f"  Position threshold: {position_threshold} m")
        rospy.loginfo(f"  Timeout: {timeout} s")
        rospy.loginfo("=" * 60)

        # If already close to the start, return immediately
        if initial_distance < position_threshold:
            rospy.loginfo("✓ Already near the start; no movement needed")
            return True

        # Initialize the PID controllers - tuned for greater stability
        # Position PID: lower gains to reduce oscillation
        pid_x = PIDController(kp=1.5, ki=0.05, kd=0.8, output_limit=3.0)
        pid_y = PIDController(kp=1.5, ki=0.05, kd=0.8, output_limit=3.0)
        pid_z = PIDController(kp=2.0, ki=0.08, kd=1.0, output_limit=3.0)

        # Velocity PID: more conservative parameters
        pid_vx = PIDController(kp=0.8, ki=0.02, kd=0.3, output_limit=1.5)
        pid_vy = PIDController(kp=0.8, ki=0.02, kd=0.3, output_limit=1.5)
        pid_vz = PIDController(kp=1.0, ki=0.03, kd=0.4, output_limit=1.5)

        rospy.loginfo("🚁 Starting PID-controlled return...")

        rate = rospy.Rate(50)  # 50 Hz control rate
        start_time = rospy.Time.now()
        last_log_time = 0
        stable_counter = 0
        required_stable_count = 100  # Requires 2 seconds of stability (100 cycles @ 50 Hz)

        max_position_error = initial_distance
        min_position_error = initial_distance

        # Phase control
        phase = "approaching"  # approaching, stabilizing, stable

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()

            # Timeout check
            if elapsed > timeout:
                current_distance = np.sqrt(
                    (target_position.x - self.current_position.x) ** 2 +
                    (target_position.y - self.current_position.y) ** 2 +
                    (target_position.z - self.current_position.z) ** 2
                )

                # If the distance is within an acceptable range, still consider it a success
                if current_distance < position_threshold * 2:  # relaxed to 0.6 m
                    rospy.logwarn(f"⚠ Timed out but position is acceptable (error: {current_distance:.3f}m < {position_threshold * 2}m)")
                    rospy.loginfo("=" * 60)
                    rospy.loginfo(f"✓ Return to start complete (timed out but position acceptable)")
                    rospy.loginfo(f"  Final position: ({self.current_position.x:.3f}, "
                                  f"{self.current_position.y:.3f}, "
                                  f"{self.current_position.z:.3f})")
                    rospy.loginfo(f"  Position error: {current_distance:.4f} m")
                    rospy.loginfo(f"  Elapsed time: {elapsed:.1f} s")
                    rospy.loginfo("=" * 60)
                    return True
                else:
                    rospy.logerr(f"✗ Return to start timed out and the position error is too large")
                    rospy.logerr(f"  Current distance from start: {current_distance:.3f} m (threshold: {position_threshold * 2}m)")
                    return False

            # Compute the position error
            pos_error_x = target_position.x - self.current_position.x
            pos_error_y = target_position.y - self.current_position.y
            pos_error_z = target_position.z - self.current_position.z

            position_error = np.sqrt(pos_error_x ** 2 + pos_error_y ** 2 + pos_error_z ** 2)

            # Update the error statistics
            max_position_error = max(max_position_error, position_error)
            min_position_error = min(min_position_error, position_error)

            # Position PID output (desired velocity)
            desired_vx = pid_x.compute(pos_error_x, current_time)
            desired_vy = pid_y.compute(pos_error_y, current_time)
            desired_vz = pid_z.compute(pos_error_z, current_time)

            # Velocity error
            vel_error_x = desired_vx - self.current_velocity.x
            vel_error_y = desired_vy - self.current_velocity.y
            vel_error_z = desired_vz - self.current_velocity.z

            # Velocity PID output
            ax = pid_vx.compute(vel_error_x, current_time)
            ay = pid_vy.compute(vel_error_y, current_time)
            az = pid_vz.compute(vel_error_z, current_time)

            # Build the control command
            control_velocity = Vector3(desired_vx, desired_vy, desired_vz)

            # Publish the control command
            traj = MultiDOFJointTrajectory()
            traj.header.stamp = current_time
            traj.header.frame_id = "world"
            traj.joint_names = ["base_link"]

            point = MultiDOFJointTrajectoryPoint()

            # Position
            transform = Transform()
            transform.translation = target_position
            transform.rotation = Quaternion(0, 0, 0, 1)
            point.transforms.append(transform)

            # Velocity
            twist = Twist()
            twist.linear = control_velocity
            twist.angular = Vector3(0, 0, 0)
            point.velocities.append(twist)

            point.time_from_start = rospy.Duration(0.02)  # 20 ms

            traj.points.append(point)
            self.traj_pub.publish(traj)

            # Phase determination
            if position_error < position_threshold:
                if phase == "approaching":
                    phase = "stabilizing"
                    rospy.loginfo(f"  >> Entering the stabilizing phase (error: {position_error:.4f}m)")
            else:
                if phase == "stabilizing":
                    phase = "approaching"
                    rospy.loginfo(f"  >> Returning to the approaching phase (error: {position_error:.4f}m)")

            # Log output (once per second)
            if int(elapsed) > last_log_time:
                speed = np.sqrt(
                    self.current_velocity.x ** 2 +
                    self.current_velocity.y ** 2 +
                    self.current_velocity.z ** 2
                )
                rospy.loginfo(
                    f"  [{phase:12s}] {elapsed:4.1f}s | "
                    f"error: {position_error:.4f}m | "
                    f"velocity: {speed:.2f}m/s | "
                    f"stable: {stable_counter}/{required_stable_count}"
                )
                last_log_time = int(elapsed)

            # Arrival check: position error below the threshold
            if position_error < position_threshold:
                stable_counter += 1

                if stable_counter >= required_stable_count:
                    rospy.loginfo("=" * 60)
                    rospy.loginfo(f"✓ Successfully returned to start and stabilized!")
                    rospy.loginfo(f"  Final position: ({self.current_position.x:.3f}, "
                                  f"{self.current_position.y:.3f}, "
                                  f"{self.current_position.z:.3f})")
                    rospy.loginfo(f"  Position error: {position_error:.4f} m")
                    rospy.loginfo(f"  Elapsed time: {elapsed:.1f} s")
                    rospy.loginfo(f"  Max error: {max_position_error:.4f} m")
                    rospy.loginfo(f"  Min error: {min_position_error:.4f} m")
                    rospy.loginfo("=" * 60)
                    return True
            else:
                stable_counter = 0  # Reset the stability counter

            rate.sleep()

        return False

    # Embedded PID controller class (in case the original file does not define one)
    class PIDController:
        """PID controller"""

        def __init__(self, kp, ki, kd, output_limit=None):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.output_limit = output_limit

            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_time = None

        def reset(self):
            """Reset the PID state"""
            self.integral = 0.0
            self.prev_error = 0.0
            self.prev_time = None

        def compute(self, error, current_time):
            """Compute the PID output"""
            if self.prev_time is None:
                self.prev_time = current_time
                self.prev_error = error
                return 0.0

            dt = (current_time - self.prev_time).to_sec()
            if dt <= 0:
                return 0.0

            # Proportional term
            p_term = self.kp * error

            # Integral term (with anti-windup)
            self.integral += error * dt
            if self.output_limit is not None:
                self.integral = np.clip(
                    self.integral,
                    -self.output_limit / self.ki,
                    self.output_limit / self.ki
                )
            i_term = self.ki * self.integral

            # Derivative term
            derivative = (error - self.prev_error) / dt
            d_term = self.kd * derivative

            # Total output
            output = p_term + i_term + d_term

            # Saturation limiting
            if self.output_limit is not None:
                output = np.clip(output, -self.output_limit, self.output_limit)

            self.prev_error = error
            self.prev_time = current_time

            return output

    def measure_odom_frequency(self, duration=2.0):
        """Measure the actual odometry frequency"""
        rospy.loginfo("Measuring odometry frequency...")
        self.odom_callback_count = 0
        self.odom_start_time = rospy.Time.now()

        rospy.sleep(duration)

        elapsed = (rospy.Time.now() - self.odom_start_time).to_sec()
        measured_frequency = self.odom_callback_count / elapsed if elapsed > 0 else 0

        self.odom_start_time = None

        rospy.loginfo(f"Measurement complete: odometry frequency ≈ {measured_frequency:.1f} Hz")
        return measured_frequency

    def load_waypoints_from_csv(self, csv_file):
        """Load waypoints from a CSV file"""
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                waypoints = []
                for row in reader:
                    waypoint = {
                        'index': int(row['index']),
                        'x': float(row['x_m']),
                        'y': float(row['y_m']),
                        'z': float(row['z_m'])
                    }
                    waypoints.append(waypoint)

                self.waypoints = waypoints
                rospy.loginfo("=" * 60)
                rospy.loginfo(f"✓ Successfully loaded {len(self.waypoints)} waypoints")
                rospy.loginfo(f"  Start: ({waypoints[0]['x']:.3f}, {waypoints[0]['y']:.3f}, {waypoints[0]['z']:.3f})")
                rospy.loginfo(f"  End: ({waypoints[-1]['x']:.3f}, {waypoints[-1]['y']:.3f}, {waypoints[-1]['z']:.3f})")
                rospy.loginfo("=" * 60)
                return True

        except FileNotFoundError:
            rospy.logerr(f"✗ File not found: {csv_file}")
            return False
        except Exception as e:
            rospy.logerr(f"✗ Failed to load CSV file: {e}")
            import traceback
            traceback.print_exc()
            return False

    def downsample_waypoints(self, target_count=None, min_distance=None):
        """
        Downsample the waypoints

        Parameters:
            target_count: target number of waypoints
            min_distance: minimum waypoint spacing (meters)
        """
        if len(self.waypoints) == 0:
            return

        original_count = len(self.waypoints)

        # Method 1: downsample by target count
        if target_count is not None and len(self.waypoints) > target_count:
            step = len(self.waypoints) / target_count
            selected_indices = [int(i * step) for i in range(target_count)]

            # Make sure the last point is included
            if selected_indices[-1] != len(self.waypoints) - 1:
                selected_indices[-1] = len(self.waypoints) - 1

            downsampled = [self.waypoints[i] for i in selected_indices]

        # Method 2: downsample by minimum spacing
        elif min_distance is not None:
            downsampled = [self.waypoints[0]]  # Keep the first point

            for i in range(1, len(self.waypoints)):
                last_wp = downsampled[-1]
                current_wp = self.waypoints[i]

                # Compute the distance
                dx = current_wp['x'] - last_wp['x']
                dy = current_wp['y'] - last_wp['y']
                dz = current_wp['z'] - last_wp['z']
                distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                # If the distance exceeds the threshold, add this point
                if distance >= min_distance:
                    downsampled.append(current_wp)

            # Make sure the last point is included
            if downsampled[-1]['index'] != self.waypoints[-1]['index']:
                downsampled.append(self.waypoints[-1])
        else:
            rospy.loginfo("No downsampling parameter specified; keeping the original waypoints")
            return

        # Renumber
        for i, wp in enumerate(downsampled):
            wp['index'] = i

        self.waypoints = downsampled

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✓ Waypoint downsampling complete: {original_count} → {len(self.waypoints)}")
        rospy.loginfo("=" * 60)

    def calculate_segment_time(self, wp1, wp2, max_speed=5.0, min_time=0.3):
        """
        Compute the flight time between two waypoints (accelerated version)

        Parameters:
            wp1, wp2: waypoint dictionaries
            max_speed: maximum speed (m/s) - raised to 5.0
            min_time: minimum time (seconds) - lowered to 0.3
        """
        dx = wp2['x'] - wp1['x']
        dy = wp2['y'] - wp1['y']
        dz = wp2['z'] - wp1['z']
        distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # Compute the time from the distance
        time = max(distance / max_speed, min_time)

        return time

    def create_global_trajectory(self, max_speed=5.0, min_segment_time=0.3):
        """
        Create the global trajectory message (accelerated version)

        Parameters:
            max_speed: maximum flight speed (m/s) - default 5.0
            min_segment_time: minimum time per segment (seconds) - default 0.3
        """
        if len(self.waypoints) == 0:
            rospy.logerr("No waypoints available!")
            return None, 0.0

        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]

        cumulative_time = 0.0

        rospy.loginfo("=" * 60)
        rospy.loginfo("Building the global trajectory (high-speed mode)...")
        rospy.loginfo(f"Maximum speed: {max_speed} m/s")
        rospy.loginfo("=" * 60)

        for i, waypoint in enumerate(self.waypoints):
            point = MultiDOFJointTrajectoryPoint()

            # Set the position
            transform = Transform()
            transform.translation.x = waypoint['x']
            transform.translation.y = waypoint['y']
            transform.translation.z = waypoint['z']
            transform.rotation = Quaternion(0, 0, 0, 1)
            point.transforms.append(transform)

            # Compute the velocity
            velocity = Twist()
            if i < len(self.waypoints) - 1:
                next_wp = self.waypoints[i + 1]

                # Compute the direction vector
                dx = next_wp['x'] - waypoint['x']
                dy = next_wp['y'] - waypoint['y']
                dz = next_wp['z'] - waypoint['z']
                distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                # Compute the time for this segment
                segment_time = self.calculate_segment_time(
                    waypoint, next_wp, max_speed, min_segment_time
                )

                # Compute the velocity components
                if distance > 0:
                    speed = distance / segment_time
                    velocity.linear.x = (dx / distance) * speed
                    velocity.linear.y = (dy / distance) * speed
                    velocity.linear.z = (dz / distance) * speed

                cumulative_time += segment_time
            else:
                # The last point has zero velocity
                velocity.linear = Vector3(0, 0, 0)

            velocity.angular = Vector3(0, 0, 0)
            point.velocities.append(velocity)

            # Set the timestamp
            point.time_from_start = rospy.Duration(cumulative_time)

            traj.points.append(point)

            # Print progress
            if i % 10 == 0 or i == len(self.waypoints) - 1:
                rospy.loginfo(f"  Waypoint {i + 1}/{len(self.waypoints)}: "
                              f"position({waypoint['x']:.2f}, {waypoint['y']:.2f}, {waypoint['z']:.2f}) "
                              f"time: {cumulative_time:.1f}s")

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✓ Global trajectory build complete!")
        rospy.loginfo(f"  Total waypoints: {len(traj.points)}")
        rospy.loginfo(f"  Estimated flight time: {cumulative_time:.1f} s ⚡")
        rospy.loginfo("=" * 60)

        return traj, cumulative_time

    def execute_global_trajectory(self, epoch, max_speed=5.0, min_segment_time=0.3):
        """
        Execute global trajectory tracking (accelerated version)

        Parameters:
            epoch: current loop iteration
            max_speed: maximum flight speed (m/s) - default 5.0
            min_segment_time: minimum time per segment (seconds) - default 0.3
        """
        if len(self.waypoints) == 0:
            rospy.logerr("No waypoints available!")
            return False

        # Create the global trajectory
        traj_msg, total_time = self.create_global_trajectory(max_speed, min_segment_time)

        if traj_msg is None:
            return False

        # ** Configure the data recording parameters **
        # Measure the odometry frequency
        odom_frequency = self.measure_odom_frequency(duration=2.0)

        # Compute the expected total number of callbacks
        self.total_expected_callbacks = int(total_time * odom_frequency)

        # Target number of records (excluding the 2 manually recorded first/last points)
        self.target_records = self.max_records - 2

        # Reset the record counter
        self.record_counter = 0

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Data recording configuration:")
        rospy.loginfo(f"  Odometry frequency: {odom_frequency:.1f} Hz")
        rospy.loginfo(f"  Estimated flight time: {total_time:.1f} s")
        rospy.loginfo(f"  Expected total callbacks: {self.total_expected_callbacks}")
        rospy.loginfo(f"  Target record count (intermediate points): {self.target_records}")
        rospy.loginfo(
            f"  Average sampling interval: record once every {self.total_expected_callbacks / float(self.target_records):.1f} callbacks")
        rospy.loginfo("=" * 60)

        # ** Record the first waypoint data **
        if not self.first_waypoint_recorded:
            rospy.loginfo("Recording the first waypoint data...")
            self.record_current_state()
            self.first_waypoint_recorded = True

        # ** Start recording **
        self.recording = True

        # Publish the trajectory
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo(f"🚁 Epoch {epoch}: starting global trajectory tracking (high-speed mode)...")
        rospy.loginfo("=" * 60)

        self.traj_pub.publish(traj_msg)

        # Monitor the flight progress
        rate = rospy.Rate(5)  # Raised to 5 Hz to better monitor high-speed flight
        start_time = rospy.Time.now()
        last_progress = -1

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(int(elapsed / total_time * 100), 100)

            # Print progress every 10%
            if progress // 10 > last_progress // 10:
                rospy.loginfo(f"Flight progress: {progress}% ({elapsed:.1f}s / {total_time:.1f}s) "
                              f"current position: ({self.current_position.x:.2f}, "
                              f"{self.current_position.y:.2f}, "
                              f"{self.current_position.z:.2f}) "
                              f"velocity: ({self.current_velocity.x:.2f}, "
                              f"{self.current_velocity.y:.2f}, "
                              f"{self.current_velocity.z:.2f}) "
                              f"recorded: {len(self.position_x)}/{self.max_records}")
                last_progress = progress

            # Flight complete
            if elapsed >= total_time + 1.0:  # Reduced the wait time to 1 second
                break

            rate.sleep()

        # ** Stop recording **
        self.recording = False

        # ** Record the last waypoint data **
        if not self.last_waypoint_recorded:
            rospy.loginfo("Recording the last waypoint data...")
            self.record_current_state()
            self.last_waypoint_recorded = True

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo(f"✓ Epoch {epoch}: global trajectory tracking complete!")
        rospy.loginfo(f"  Actual flight time: {elapsed:.1f} s")
        rospy.loginfo(f"  Final position: ({self.current_position.x:.2f}, "
                      f"{self.current_position.y:.2f}, "
                      f"{self.current_position.z:.2f})")
        rospy.loginfo(f"  Final velocity: ({self.current_velocity.x:.2f}, "
                      f"{self.current_velocity.y:.2f}, "
                      f"{self.current_velocity.z:.2f})")
        rospy.loginfo(f"  Recorded data points: {len(self.position_x)}")
        rospy.loginfo(f"  Actual callback count: {self.record_counter}")
        rospy.loginfo("=" * 60)

        return True

    def run_multiple_epochs(self, num_epochs=5, max_speed=5.0, min_segment_time=0.3,
                            wait_between_epochs=3.0, return_speed=2.0):
        """
        Run multiple loop flights
        """
        rospy.loginfo("\n" + "🎯" * 30)
        rospy.loginfo(f"Starting {num_epochs} loop flights")
        rospy.loginfo("🎯" * 30 + "\n")

        for epoch in range(1, num_epochs + 1):
            self.current_epoch = epoch

            rospy.loginfo("\n" + "📍" * 30)
            rospy.loginfo(f"Epoch {epoch}/{num_epochs}")
            rospy.loginfo("📍" * 30 + "\n")

            # Return to start before each loop begins (except the first)
            if epoch > 1:
                rospy.loginfo("🔄 Preparing a new loop; returning to start first...")
                success = self.return_to_start_with_pid()

                if not success:
                    rospy.logerr(f"✗ Epoch {epoch}: failed to return to start!")
                    return False

                rospy.loginfo(f"⏳ Waiting at the start for {wait_between_epochs} seconds before flight...\n")
                rospy.sleep(wait_between_epochs)

            # Execute the trajectory
            success = self.execute_global_trajectory(
                epoch=epoch,
                max_speed=max_speed,
                min_segment_time=min_segment_time
            )

            if not success:
                rospy.logerr(f"✗ Epoch {epoch} execution failed!")
                return False

            # Ensure the data point count is correct
            self.ensure_exact_record_count()

            # Save the data
            self.save_recorded_data()

            # Fix: reset the data after every save (including the last one)
            if epoch < num_epochs:
                rospy.loginfo(f"\n✓ Epoch {epoch} complete; resetting data for the next round...\n")
                self.reset_for_next_epoch()  # Added this line
            else:
                rospy.loginfo(f"\n✓ Epoch {epoch} complete (final round)\n")

        rospy.loginfo("\n" + "🎉" * 30)
        rospy.loginfo(f"All {num_epochs} loop flights complete!")
        rospy.loginfo("🎉" * 30 + "\n")

        return True


def signal_handler(sig, frame):
    """Handle the Ctrl+C signal"""
    rospy.loginfo("\nInterrupt signal received; cleaning up...")
    sys.exit(0)


def main():
    controller = None

    try:
        # 注册信号处理器
        signal.signal(signal.SIGINT, signal_handler)

        controller = GlobalTrajectoryController()

        # 从同目录下加载CSV文件
        csv_file = 'waypoints.csv'

        if not controller.load_waypoints_from_csv(csv_file):
            rospy.logerr("无法加载航点文件，程序退出")
            return

        # 可选：降采样航点（加快速度）
        controller.downsample_waypoints(target_count=20)  # 减少到20个点
        # controller.downsample_waypoints(min_distance=1.0)  # 增加最小间距到1米

        # 等待系统初始化
        rospy.loginfo("等待系统初始化...")
        rospy.sleep(2.0)

        # 执行多次循环飞行
        success = controller.run_multiple_epochs(
            num_epochs=101,  # 🔄 循环100次
            max_speed=2.0,  # 🚀 速度2.0 m/s
            min_segment_time=0.3,  # ⚡ 最小时间0.3秒
            wait_between_epochs=1.0  # ⏳ 每次循环间隔3秒
        )

        if success:
            rospy.loginfo("✅ 所有任务完成！")
        else:
            rospy.logerr("❌ 任务执行失败！")

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
