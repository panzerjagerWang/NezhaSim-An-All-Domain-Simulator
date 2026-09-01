#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#

#!/usr/bin/env python3
"""
UUV trajectory tracking control
Tracks a CSV trajectory file using waypoint navigation
Supports 2D trajectories (x, z coordinates)
New: real-time monitoring of waypoint passage, capturing data the instant a waypoint is passed
New: automatic Gazebo model reset
"""

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w)

    Args:
        roll: roll angle (radians)
        pitch: pitch angle (radians)
        yaw: yaw angle (radians)

    Returns:
        (qx, qy, qz, qw): quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class UUVTrajectoryTracker:
    def __init__(self, trajectory_file):
        rospy.init_node("uuv_trajectory_tracker")

        # ========== Initialize all attributes first so callbacks never access undefined attributes ==========
        self.current_pose = None

        # Data recording
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []
        self.recorded_waypoints = set()

        # Real-time monitoring
        self.enable_realtime_monitoring = False
        self.monitoring_waypoint_indices = []

        # Trajectory tracking parameters
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 1.0
        self.lookahead_distance = 2.0
        self.fixed_y = 0.0
        self.current_epoch = 0

        # Load the trajectory (must happen before subscribing)
        self.trajectory = self.load_trajectory(trajectory_file)
        # ====================================================================

        # Now it is safe to subscribe to topics
        rospy.Subscriber("/nezha_mini/ground_truth/pose", Pose,
                         self._pose_callback, queue_size=1)

        self.pub_pitch = rospy.Publisher("/nezha_mini/thrusters/0/input",
                                         FloatStamped, queue_size=10)
        self.pub_left = rospy.Publisher("/nezha_mini/thrusters/1/input",
                                        FloatStamped, queue_size=10)
        self.pub_right = rospy.Publisher("/nezha_mini/thrusters/2/input",
                                         FloatStamped, queue_size=10)

        from UUVControllerV8 import PIDController

        # PID controllers
        self.pitch_up_pid = PIDController(
            kp=70.0, ki=12.0, kd=100.0,
            output_min=-30.0, output_max=30.0,
            windup_limit=20.0
        )

        self.pitch_down_pid = PIDController(
            kp=60.0, ki=12.0, kd=80.0,
            output_min=-30.0, output_max=30.0,
            windup_limit=20.0
        )

        self.yaw_pid = PIDController(
            kp=40.0, ki=3.0, kd=15.0,
            output_min=-20.0, output_max=20.0, windup_limit=10.0
        )

        self.position_pid = PIDController(
            kp=15.0, ki=2.0, kd=10.0,
            output_min=-30.0, output_max=30.0, windup_limit=10.0
        )

        self.depth_pid = PIDController(
            kp=80.0, ki=15.0, kd=100.0,
            output_min=-45.0, output_max=45.0,
            windup_limit=25.0
        )

        # Wait for the Gazebo service
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        print("Gazebo model reset service connected")

        # Print initialization info
        print(f"\n{'=' * 60}")
        print(f"Trajectory tracker initialized")
        print(f"Total trajectory points: {len(self.trajectory)}")
        print(f"Trajectory range: X[{self.trajectory[:, 0].min():.2f}, {self.trajectory[:, 0].max():.2f}]")
        print(f"          Z[{self.trajectory[:, 1].min():.2f}, {self.trajectory[:, 1].max():.2f}]")
        print(f"Fixed Y coordinate: {self.fixed_y:.2f}")
        print(f"Data recording: real-time waypoint passage monitoring, instantaneous capture")
        print(f"{'=' * 60}\n")

    def reset_model_to_origin(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Reset the UUV model in Gazebo to a specified position and attitude

        Args:
            x: X coordinate (default 0.0)
            y: Y coordinate (default 0.0)
            z: Z coordinate (default -5.0, 5 m underwater)
            roll: roll angle (radians) (default 0.0)
            pitch: pitch angle (radians) (default 0.0)
            yaw: yaw angle (radians) (default 0.0)
        """
        try:
            print(f"\n{'─' * 70}")
            print(f"Resetting Gazebo model to origin...")
            print(f"{'─' * 70}")

            # Create the model state message
            model_state = ModelState()
            model_state.model_name = 'nezha_mini'

            # Set position
            model_state.pose.position = Point(x, y, z)

            # Use the tf library to convert Euler angles to a quaternion
            import tf
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            model_state.pose.orientation = Quaternion(
                quaternion[0],  # x
                quaternion[1],  # y
                quaternion[2],  # z
                quaternion[3]  # w
            )

            # Set velocity to zero
            model_state.twist = Twist()

            # Set the reference frame
            model_state.reference_frame = 'world'

            # Call the service
            response = self.set_model_state(model_state)

            if response.success:
                print(f"Model position reset to: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                print(f"Model attitude reset to: Roll={math.degrees(roll):.2f}°, "
                      f"Pitch={math.degrees(pitch):.2f}°, Yaw={math.degrees(yaw):.2f}°")
                print(f"   {response.status_message}")

                # Wait for the physics engine to settle
                print(f"Waiting for the physics engine to settle...")
                rospy.sleep(3.0)

                # Reset the PID controllers
                self.position_pid.reset()
                self.yaw_pid.reset()
                self.pitch_up_pid.reset()
                self.pitch_down_pid.reset()
                self.depth_pid.reset()

                print(f"PID controllers reset")

                # Verify the reset
                rospy.sleep(0.5)
                if self.current_pose is not None:
                    current_roll, current_pitch, current_yaw = self.get_current_angles()
                    print(f"Current attitude: Roll={math.degrees(current_roll):.2f}°, "
                          f"Pitch={math.degrees(current_pitch):.2f}°, "
                          f"Yaw={math.degrees(current_yaw):.2f}°")

                return True
            else:
                print(f"Model reset failed: {response.status_message}")
                return False

        except Exception as e:
            print(f"Error during reset: {e}")
            import traceback
            traceback.print_exc()
            return False

    def reset_for_next_epoch(self):
        """Reset data in preparation for the next epoch"""
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []

        self.recorded_waypoints = set()
        self.current_waypoint_index = 0

        print(f"  Data reset, preparing for the next epoch...")
        print(f"\n{'=' * 60}")
        print(f"Trajectory tracker initialized")
        print(f"Total trajectory points: {len(self.trajectory)}")
        print(f"Trajectory range: X[{self.trajectory[:, 0].min():.2f}, {self.trajectory[:, 0].max():.2f}]")
        print(f"          Z[{self.trajectory[:, 1].min():.2f}, {self.trajectory[:, 1].max():.2f}]")
        print(f"Fixed Y coordinate: {self.fixed_y:.2f}")
        print(f"Data recording: real-time waypoint passage monitoring, instantaneous capture")
        print(f"{'=' * 60}\n")

    def load_trajectory(self, filename):
        """Load a trajectory file (x, z format)"""
        try:
            data = np.loadtxt(filename, delimiter=',')
            print(f"Trajectory file loaded successfully: {filename}")
            print(f"Trajectory data shape: {data.shape}")
            if data.shape[1] != 2:
                print(f"Warning: expected 2 columns (x, z), but got {data.shape[1]} columns")
            return data
        except Exception as e:
            print(f"Failed to load trajectory file: {e}")
            return None

    def _pose_callback(self, pose):
        """
        Pose callback - includes real-time waypoint monitoring logic
        """
        self.current_pose = pose

        # Real-time monitoring: check whether a waypoint has been passed
        if self.enable_realtime_monitoring and len(self.monitoring_waypoint_indices) > 0:
            self._check_waypoint_passage()

    def _check_waypoint_passage(self):
        """
        Check in real time whether a waypoint has been passed (called from the pose callback)
        """
        if self.current_pose is None:
            return

        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_z = self.current_pose.position.z

        # Check all waypoints currently being monitored
        waypoints_to_remove = []

        for wp_idx in self.monitoring_waypoint_indices:
            # Guard against index out of range
            if wp_idx >= len(self.trajectory):
                continue

            # Check whether it has already been recorded
            if wp_idx in self.recorded_waypoints:
                waypoints_to_remove.append(wp_idx)
                continue

            # Compute the distance to the waypoint
            wp_x, wp_z = self.trajectory[wp_idx]
            dx = wp_x - current_x
            dz = wp_z - current_z
            distance = math.sqrt(dx ** 2 + dz ** 2)

            # Record immediately if within the waypoint tolerance
            if distance < self.waypoint_tolerance:
                self._record_waypoint_data_immediate(wp_idx)
                waypoints_to_remove.append(wp_idx)

        # Remove recorded waypoints (no longer monitored)
        for wp_idx in waypoints_to_remove:
            if wp_idx in self.monitoring_waypoint_indices:
                self.monitoring_waypoint_indices.remove(wp_idx)

    def _record_waypoint_data_immediate(self, waypoint_index):
        """
        Record waypoint data immediately (called from the pose callback)
        """
        if waypoint_index in self.recorded_waypoints:
            return False

        # Record position
        self.position_x.append(self.current_pose.position.x)
        self.position_y.append(self.current_pose.position.y)
        self.position_z.append(self.current_pose.position.z)

        # Record attitude
        from UUVControllerV8 import euler_from_quaternion
        q = [self.current_pose.orientation.x,
             self.current_pose.orientation.y,
             self.current_pose.orientation.z,
             self.current_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        # Store in degrees (originally radians)
        self.roll_data.append(math.degrees(roll))
        self.pitch_data.append(math.degrees(pitch))
        self.yaw_data.append(math.degrees(yaw))

        # Mark as recorded
        self.recorded_waypoints.add(waypoint_index)

        print(f"    [Real-time capture] Waypoint {waypoint_index + 1} [{len(self.recorded_waypoints)}/{len(self.trajectory)}]")
        print(f"       Position: X={self.current_pose.position.x:.3f}, "
              f"Y={self.current_pose.position.y:.3f}, "
              f"Z={self.current_pose.position.z:.3f}")
        print(f"       Attitude: Roll={math.degrees(roll):.2f}°, "
              f"Pitch={math.degrees(pitch):.2f}°, "
              f"Yaw={math.degrees(yaw):.2f}°")

        return True

    def start_monitoring_waypoints(self, waypoint_indices):
        """
        Start monitoring a given list of waypoints

        Args:
            waypoint_indices: list of waypoint indices to monitor
        """
        self.monitoring_waypoint_indices = list(waypoint_indices)
        self.enable_realtime_monitoring = True
        print(
            f"    Started real-time monitoring of {len(self.monitoring_waypoint_indices)} waypoints: {[i + 1 for i in self.monitoring_waypoint_indices]}")

    def stop_monitoring_waypoints(self):
        """Stop monitoring waypoints"""
        self.enable_realtime_monitoring = False
        self.monitoring_waypoint_indices = []

    def save_to_txt(self, data_list, filename, mode='a'):
        """Save list data to a txt file (append mode)"""
        try:
            with open(filename, mode) as f:
                formatted_data = '[' + ','.join([f"{value:.6f}" for value in data_list]) + '],\n'
                f.write(formatted_data)
            print(f"  {filename} - Epoch {self.current_epoch} ({len(data_list)} data points)")
        except Exception as e:
            print(f"  Failed to save file {filename}: {e}")

    def save_trajectory_data(self):
        """Save the current epoch's trajectory data (appended to file)"""
        if len(self.position_x) == 0:
            print("No data to save")
            return False

        print(f"\n{'=' * 60}")
        print(f"Saving trajectory data for Epoch {self.current_epoch}...")
        print(f"Total data points: {len(self.position_x)}")
        print(f"Recorded waypoints: {sorted(self.recorded_waypoints)}")

        # Check for any missing waypoints
        missing_waypoints = []
        for i in range(len(self.trajectory)):
            if i not in self.recorded_waypoints:
                missing_waypoints.append(i + 1)

        if missing_waypoints:
            print(f"Warning: no data recorded for the following waypoints: {missing_waypoints}")

        print("-" * 60)

        try:
            # Use write mode for the first epoch, append mode for later epochs
            mode = 'w' if self.current_epoch == 1 else 'a'

            # Save position data
            print("Position data:")
            self.save_to_txt(self.position_x, 'uuv_position_x.txt', mode)
            self.save_to_txt(self.position_y, 'uuv_position_y.txt', mode)
            self.save_to_txt(self.position_z, 'uuv_position_z.txt', mode)

            print("-" * 60)

            # Save attitude data
            print("Attitude data:")
            self.save_to_txt(self.roll_data, 'uuv_roll.txt', mode)
            self.save_to_txt(self.pitch_data, 'uuv_pitch.txt', mode)
            self.save_to_txt(self.yaw_data, 'uuv_yaw.txt', mode)

            print("-" * 60)
            print(f"Epoch {self.current_epoch} data saved successfully!")
            print(f"{'=' * 60}\n")
            return True

        except Exception as e:
            print(f"Failed to save data: {e}")
            return False

    def _save_list_to_txt(self, data_list, filename):
        """Internal helper: save a list to a txt file (single-line format)"""
        with open(filename, 'w') as f:
            formatted_data = '[' + ','.join([f"{value:.6f}" for value in data_list]) + ']\n'
            f.write(formatted_data)
        print(f"  {filename} ({len(data_list)} data points)")

    def get_current_angles(self):
        if self.current_pose is None:
            return None, None, None

        from UUVControllerV8 import euler_from_quaternion
        q = [self.current_pose.orientation.x,
             self.current_pose.orientation.y,
             self.current_pose.orientation.z,
             self.current_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        return roll, pitch, yaw

    def get_current_position(self):
        if self.current_pose is None:
            return None, None, None
        return (self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z)

    def set_thrusters(self, T_pitch, T_left, T_right):
        T_pitch = max(min(T_pitch, 50.0), -50.0)
        T_r = max(min(T_right, 40.0), -40.0)
        T_l = max(min(T_left, 40.0), -40.0)

        now = rospy.Time.now()
        self.pub_pitch.publish(FloatStamped(Header(stamp=now), T_pitch))
        self.pub_right.publish(FloatStamped(Header(stamp=now), T_r))
        self.pub_left.publish(FloatStamped(Header(stamp=now), T_l))

    def stop_thrusters(self):
        self.set_thrusters(0, 0, 0)

    def wait_for_stable(self, timeout=2.0):
        time.sleep(timeout)

    def calculate_distance_xz(self, x1, z1, x2, z2):
        """Compute distance in the x-z plane"""
        return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

    def calculate_distance_to_waypoint(self, waypoint):
        """Compute distance to a waypoint (x-z plane)"""
        current_x, current_y, current_z = self.get_current_position()
        target_x, target_z = waypoint[0], waypoint[1]

        dx = target_x - current_x
        dz = target_z - current_z
        return math.sqrt(dx ** 2 + dz ** 2)

    def find_lookahead_waypoint(self):
        """
        Lookahead algorithm: find the target point at lookahead_distance ahead (x-z plane)
        """
        current_x, _, current_z = self.get_current_position()

        for i in range(self.current_waypoint_index, len(self.trajectory)):
            wp_x, wp_z = self.trajectory[i]
            dist = self.calculate_distance_xz(current_x, current_z, wp_x, wp_z)

            if dist >= self.lookahead_distance:
                return i, self.trajectory[i]

        return len(self.trajectory) - 1, self.trajectory[-1]

    def navigate_to_waypoint(self, target_x, target_z, target_y=None,
                             distance_tolerance=1.0, timeout=60.0):
        """Navigate to a given waypoint"""

        if target_y is None:
            target_y = self.fixed_y

        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Reset PID controllers
        self.position_pid.reset()
        self.yaw_pid.reset()
        self.pitch_up_pid.reset()
        self.pitch_down_pid.reset()
        self.depth_pid.reset()

        rate = rospy.Rate(50)
        start_time = time.time()
        loop_count = 0

        # Depth control parameters
        DEPTH_TOLERANCE = 0.15
        DEPTH_CRITICAL = 0.35
        MAX_TARGET_PITCH = math.radians(25)
        MAX_SAFE_PITCH = math.radians(30)

        prev_pitch = None
        prev_time = time.time()
        prev_depth = None
        warning_counter = 0
        WARNING_INTERVAL = 200

        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                print(f"  Navigation timed out!")
                return False

            current_x, current_y, current_z = self.get_current_position()

            dx = target_x - current_x
            dy = target_y - current_y
            dz = target_z - current_z

            distance_xz = math.sqrt(dx ** 2 + dz ** 2)
            distance_z = abs(dz)

            # Compute the rate of depth change
            if prev_depth is not None:
                depth_velocity = (current_z - prev_depth) / 0.02
            else:
                depth_velocity = 0.0
            prev_depth = current_z

            # Three-level control mode
            if distance_z > DEPTH_CRITICAL:
                depth_mode = "EMERGENCY"
                horizontal_speed_factor = 0.1
                depth_weight = 0.95
                pitch_weight = 0.05
                depth_gain = 3.0
            elif distance_z > DEPTH_TOLERANCE:
                depth_mode = "PRIORITY"
                horizontal_speed_factor = 0.3
                depth_weight = 0.85
                pitch_weight = 0.15
                depth_gain = 2.5
            else:
                depth_mode = "NORMAL"
                horizontal_speed_factor = 1.0
                depth_weight = 0.5
                pitch_weight = 0.5
                depth_gain = 1.5

            # Arrival check
            if distance_xz < distance_tolerance and distance_z < DEPTH_TOLERANCE:
                if loop_count % 50 == 0:
                    print(f"  Reached waypoint! X-Z:{distance_xz:.2f}m, Z error:{dz:.2f}m")
                break

            roll, pitch, yaw = self.get_current_angles()
            current_time = time.time()

            # Compute pitch rate
            if prev_pitch is not None:
                dt = max(current_time - prev_time, 0.001)
                pitch_velocity = (pitch - prev_pitch) / dt
            else:
                pitch_velocity = 0.0
            prev_pitch = pitch
            prev_time = current_time

            # Attitude control target
            if depth_mode == "EMERGENCY":
                horizontal_base = max(abs(dx), 0.8)
            elif depth_mode == "PRIORITY":
                horizontal_base = max(abs(dx), 1.5)
            else:
                horizontal_base = max(abs(dx), 2.5)

            target_pitch_geometry = math.atan2(dz, horizontal_base)
            target_pitch_geometry = max(min(target_pitch_geometry, MAX_TARGET_PITCH), -MAX_TARGET_PITCH)

            # Safety protection
            if abs(pitch) > MAX_SAFE_PITCH:
                if warning_counter % WARNING_INTERVAL == 0:
                    print(f"  Pitch={math.degrees(pitch):.1f}° exceeds limit")
                target_pitch_geometry = pitch * 0.6

            # Depth control
            depth_error = target_z - current_z
            depth_thrust_pid = self.depth_pid.update(target_z, current_z, current_time)

            # Feedforward control
            if abs(depth_error) > DEPTH_TOLERANCE:
                if abs(depth_velocity) < 0.05:
                    feedforward = depth_error * 30.0
                    depth_thrust_raw = depth_thrust_pid + feedforward
                else:
                    depth_thrust_raw = depth_thrust_pid
            else:
                depth_thrust_raw = depth_thrust_pid

            depth_thrust = depth_thrust_raw * depth_gain
            depth_thrust = max(min(depth_thrust, 45.0), -45.0)

            # Horizontal thrust control
            if depth_mode == "EMERGENCY":
                thrust_magnitude = 1.0
            elif depth_mode == "PRIORITY":
                thrust_magnitude = 2.5
            else:
                thrust_raw = self.position_pid.update(distance_tolerance, -distance_xz, current_time)
                thrust_magnitude = abs(thrust_raw)
                thrust_magnitude = max(min(thrust_magnitude, 25.0), 5.0)

            thrust = thrust_magnitude if dx >= 0 else -thrust_magnitude
            thrust *= horizontal_speed_factor

            # Yaw control
            target_yaw = math.atan2(dy, dx) if abs(dy) > 0.1 else 0.0
            yaw_error = target_yaw - yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
            yaw_correction = self.yaw_pid.update(target_yaw, yaw, current_time) * 0.3

            # Attitude control
            pitch_error = target_pitch_geometry - pitch

            if pitch_error > 0:
                pid_output = -self.pitch_up_pid.update(target_pitch_geometry, pitch, current_time)
            else:
                pid_output = -self.pitch_down_pid.update(target_pitch_geometry, pitch, current_time)

            if depth_mode == "EMERGENCY":
                pid_output *= 0.3
            elif depth_mode == "PRIORITY":
                pid_output *= 0.5
            else:
                pid_output *= 0.7

            damping_gain = 3.0
            T_damping = damping_gain * pitch_velocity

            # Weighted fusion
            pitch_control_component = (pid_output + T_damping) * pitch_weight
            depth_control_component = depth_thrust * depth_weight

            pitch_output = pitch_control_component + depth_control_component
            pitch_output = max(min(pitch_output, 50.0), -50.0)

            # Detailed debug output
            if loop_count % 100 == 0:
                direction = "backward" if dx < 0 else "forward"

                if depth_mode == "EMERGENCY":
                    mode_icon = "EMERGENCY DEPTH"
                elif depth_mode == "PRIORITY":
                    mode_icon = "DEPTH PRIORITY"
                else:
                    mode_icon = "NORMAL MODE"

                dive_status = "diving" if dz < 0 else "ascending"
                pitch_warning = " PITCH TOO LARGE" if abs(pitch) > MAX_SAFE_PITCH else ""
                stall_warning = " DEPTH STALLED" if abs(depth_velocity) < 0.05 and distance_z > DEPTH_TOLERANCE else ""

                print(
                    f"  [{mode_icon}] {dive_status} | X-Z:{distance_xz:.2f}m | Z error:{dz:.2f}m | {direction}{pitch_warning}{stall_warning}")
                print(f"        X:{current_x:.2f}→{target_x:.2f} | Z:{current_z:.2f}→{target_z:.2f}")
                print(
                    f"        Pitch:{math.degrees(pitch):.1f}°→{math.degrees(target_pitch_geometry):.1f}° | Depth velocity:{depth_velocity:.3f}m/s")

            loop_count += 1
            warning_counter += 1

            self.set_thrusters(pitch_output, thrust - yaw_correction, thrust + yaw_correction)
            rate.sleep()

        return True

    def track_trajectory_with_lookahead(self):
        """
        轨迹跟踪主函数 - 🆕 使用实时监控采集数据
        """
        if self.trajectory is None or len(self.trajectory) == 0:
            print("轨迹为空,无法跟踪!")
            return False

        print(f"\n{'#' * 60}")
        print(f"# 开始轨迹跟踪(前瞻算法 + 实时数据采集)")
        print(f"{'#' * 60}\n")

        self.current_waypoint_index = 0
        start_time = time.time()
        last_lookahead_idx = 0

        while not rospy.is_shutdown():
            # 检查是否完成轨迹
            if self.current_waypoint_index >= len(self.trajectory):
                print(f"\n{'=' * 60}")
                print(f"✓✓✓ 轨迹跟踪完成!")
                print(f"总用时: {time.time() - start_time:.1f}秒")
                print(f"{'=' * 60}\n")
                break

            # 使用前瞻算法找到目标点
            lookahead_idx, lookahead_wp = self.find_lookahead_waypoint()
            target_x, target_z = lookahead_wp[0], lookahead_wp[1]

            # 防止索引倒退
            if lookahead_idx < last_lookahead_idx:
                print(f"  ! 检测到索引倒退 ({lookahead_idx} < {last_lookahead_idx})，使用顺序索引")
                lookahead_idx = self.current_waypoint_index
                target_x, target_z = self.trajectory[lookahead_idx]

            last_lookahead_idx = lookahead_idx

            print(f"\n{'─' * 60}")
            print(f"📍 Waypoint {self.current_waypoint_index + 1}/{len(self.trajectory)}")
            print(f"{'─' * 60}")
            print(f"目标: X={target_x:.2f}, Z={target_z:.2f}, Y={self.fixed_y:.2f}")
            print(f"前瞻索引: {lookahead_idx + 1}")

            # 🆕 开始监控本次导航可能经过的所有航点
            waypoints_to_monitor = list(range(self.current_waypoint_index, lookahead_idx + 1))
            self.start_monitoring_waypoints(waypoints_to_monitor)

            # 导航到目标点
            success = self.navigate_to_waypoint(
                target_x, target_z,
                target_y=self.fixed_y,
                distance_tolerance=self.waypoint_tolerance,
                timeout=30.0
            )

            # 🆕 停止监控
            self.stop_monitoring_waypoints()

            if not success:
                print(f"✗ 导航失败")

            # 更新当前航点索引（基于实际记录的航点）
            for i in range(self.current_waypoint_index, len(self.trajectory)):
                if i in self.recorded_waypoints:
                    self.current_waypoint_index = i + 1
                    print(f"  ✓ 通过 Waypoint {i + 1}")
                else:
                    break

            # 短暂停顿
            self.wait_for_stable(0.5)

        # 保存整条轨迹的数据
        self.save_trajectory_data()

        self.stop_thrusters()
        return True

    def track_trajectory_simple(self):
        """
        简单版轨迹跟踪 - 🆕 使用实时监控采集数据
        """
        if self.trajectory is None or len(self.trajectory) == 0:
            print("轨迹为空,无法跟踪!")
            return False

        print(f"\n{'#' * 60}")
        print(f"# 开始轨迹跟踪(简单模式 + 实时数据采集)")
        print(f"{'#' * 60}\n")

        start_time = time.time()
        success_count = 0

        for i, waypoint in enumerate(self.trajectory):
            target_x, target_z = waypoint[0], waypoint[1]

            print(f"\n{'─' * 60}")
            print(f"📍 Waypoint {i + 1}/{len(self.trajectory)}")
            print(f"{'─' * 60}")
            print(f"目标: X={target_x:.2f}, Z={target_z:.2f}, Y={self.fixed_y:.2f}")

            # 🆕 开始监控当前航点
            self.start_monitoring_waypoints([i])

            success = self.navigate_to_waypoint(
                target_x, target_z,
                target_y=self.fixed_y,
                distance_tolerance=self.waypoint_tolerance,
                timeout=30.0
            )

            # 🆕 停止监控
            self.stop_monitoring_waypoints()

            if success:
                success_count += 1
            else:
                print(f"✗ Waypoint {i + 1} 导航失败")

            self.wait_for_stable(0.5)

        print(f"\n{'=' * 60}")
        print(f"轨迹跟踪完成!")
        print(f"成功: {success_count}/{len(self.trajectory)} 个waypoint")
        print(f"总用时: {time.time() - start_time:.1f}秒")
        print(f"{'=' * 60}\n")

        # 保存整条轨迹的数据
        self.save_trajectory_data()

        self.stop_thrusters()
        return True


if __name__ == "__main__":
    try:
        # 定义原点位置
        ORIGIN_X = 0.0
        ORIGIN_Y = 0.0
        ORIGIN_Z = 0.0  # 根据你的实际初始深度调整

        # 重复次数
        NUM_REPEATS = 100

        print(f"\n{'#' * 70}")
        print(f"# 开始 {NUM_REPEATS} 次轨迹跟踪循环")
        print(f"# 原点位置: X={ORIGIN_X:.2f}, Y={ORIGIN_Y:.2f}, Z={ORIGIN_Z:.2f}")
        print(f"# 数据将追加保存到同一文件")
        print(f"# 每次循环后将重置Gazebo模型")
        print(f"{'#' * 70}\n")

        # 🆕 只创建一次tracker实例
        tracker = UUVTrajectoryTracker("Field_data_distance.csv")
        rospy.sleep(2.0)

        for repeat_count in range(1, NUM_REPEATS + 1):
            print(f"\n{'=' * 70}")
            print(f"{'=' * 70}")
            print(f"  🔄 第 {repeat_count}/{NUM_REPEATS} 次循环")
            print(f"{'=' * 70}")
            print(f"{'=' * 70}\n")

            # 🆕 更新epoch计数
            tracker.current_epoch = repeat_count

            # 执行轨迹跟踪
            print(f"\n>>> 开始第 {repeat_count} 次轨迹跟踪...")
            success = tracker.track_trajectory_with_lookahead()

            if success:
                print(f"\n✅ 第 {repeat_count} 次轨迹跟踪完成")
            else:
                print(f"\n⚠️ 第 {repeat_count} 次轨迹跟踪失败")

            # 🆕 保存当前epoch的数据（追加到文件）
            tracker.save_trajectory_data()

            # 如果不是最后一次循环，重置模型和数据
            if repeat_count < NUM_REPEATS:
                # 停止推进器
                tracker.stop_thrusters()
                tracker.wait_for_stable(1.0)

                # 🆕 重置Gazebo模型到原点
                reset_success = tracker.reset_model_to_origin(
                    x=ORIGIN_X,
                    y=ORIGIN_Y,
                    z=ORIGIN_Z
                )

                if not reset_success:
                    print(f"⚠️ 模型重置失败，但继续下一次循环")

                # 等待模型稳定
                tracker.wait_for_stable(2.0)

                # 🆕 重置数据准备下一个epoch
                tracker.reset_for_next_epoch()

                print(f"\n⏳ 准备开始第 {repeat_count + 1} 次循环...\n")

        print(f"\n{'#' * 70}")
        print(f"{'#' * 70}")
        print(f"  🎉 所有 {NUM_REPEATS} 次循环完成！")
        print(f"  📊 数据已保存到:")
        print(f"     - uuv_position_x.txt")
        print(f"     - uuv_position_y.txt")
        print(f"     - uuv_position_z.txt")
        print(f"     - uuv_roll.txt")
        print(f"     - uuv_pitch.txt")
        print(f"     - uuv_yaw.txt")
        print(f"{'#' * 70}")
        print(f"{'#' * 70}\n")

        # 清理
        tracker.stop_thrusters()

    except rospy.ROSInterruptException:
        print("\n⚠️ ROS中断")
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    except Exception as e:
        import traceback

        print(f"\n❌ 发生错误: {e}")
        traceback.print_exc()

