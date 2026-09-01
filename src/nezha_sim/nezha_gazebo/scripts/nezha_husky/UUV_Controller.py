#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
import math
import time
import numpy as np
import rospy
import argparse
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

from geometry_msgs.msg import Pose
from std_msgs.msg import Header
import transformations as tf


# ----------------------- WP state enum ------------------------------
class WPState:
    ALIGN_ATT = 0
    APPROACH = 1
    DECEL = 2
    DONE = 3


# ------------------------- PID Controller ---------------------------
class PIDController:
    def __init__(self, kp, ki, kd, output_min, output_max, windup_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.windup_limit = windup_limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, setpoint, measurement, current_time=None):
        # Use the system time or the provided time
        if current_time is None:
            current_time = time.time()

        # First call
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = setpoint - measurement
            return self.kp * self.prev_error  # Use the proportional term only

        # Compute the error and the time delta
        error = setpoint - measurement
        dt = current_time - self.prev_time

        # Guard against division by zero and abnormal time deltas
        if dt <= 0:
            return self.kp * error

        # Compute the derivative term (with low-pass filtering)
        derivative = (error - self.prev_error) / dt

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, self.windup_limit), -self.windup_limit)

        # Compute the PID output
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)

        # Clamp the output range
        output = max(min(output, self.output_max), self.output_min)

        # Update the state
        self.prev_error = error
        self.prev_time = current_time

        return output


# --------------------------------------------------------------------
def euler_from_quaternion(q):
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(+1.0, sinp)))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ----------------------- Main Class ---------------------------------
class UnderwaterNavigation:
    def __init__(self, args):
        # Subscribers & publishers
        rospy.Subscriber("/nezha_husky/ground_truth/pose", Pose, self._pose_callback, queue_size=1)
        self.pub_pitch = rospy.Publisher("/nezha_husky/thrusters/0/input",
                                         FloatStamped, queue_size=10)
        self.pub_left = rospy.Publisher("/nezha_husky/thrusters/1/input",
                                        FloatStamped, queue_size=10)
        self.pub_right = rospy.Publisher("/nezha_husky/thrusters/2/input",
                                         FloatStamped, queue_size=10)

        # Parameter initialization (loaded from args)
        # PID parameters
        self.pitch_pid_controller = PIDController(
            kp=args.K_pitch,
            ki=args.K_pitch_i,
            kd=args.K_pitch_d,
            output_min=-40,
            output_max=40,
            windup_limit=10.0  # Prevent integral windup
        )
        self.yaw_pid_controller = PIDController(
            kp=args.K_yaw,
            ki=args.K_yaw_i,
            kd=args.K_yaw_d,
            output_min=-10,
            output_max=10,
            windup_limit=5.0  # Prevent integral windup
        )

        self.K_fwd = args.K_fwd
        self.T_fwd_max = args.T_fwd_max

        self.pitch_limit = math.radians(args.pitch_limit_deg)
        self.pitch_guard = math.radians(args.pitch_guard_deg)
        self.yaw_tol = math.radians(args.yaw_tol_deg)
        self.pitch_tol = math.radians(args.pitch_tol_deg)
        self.roll_tol = math.radians(args.roll_tol_deg)
        self.heading_tol = math.radians(args.heading_tol_deg)  # Heading angle limit

        self.final_tol = args.final_tol
        self.decel_radius = args.decel_radius

        # Waypoint parameters
        self.wp_x, self.wp_y, self.wp_z = args.wp_x, args.wp_y, args.wp_z
        self.state = WPState.ALIGN_ATT
        self.yaw_sp = 0.0
        self.pitch_sp = 0.0

        # Timeout control
        self.max_mission_time = args.max_mission_time  # Maximum mission execution time (seconds)
        self.max_align_time = args.max_align_time  # Maximum time for a single attitude adjustment
        self.mission_start_time = time.time()
        self.align_start_time = time.time()

        # Energy optimization parameters
        self.energy_efficient_mode = args.energy_efficient
        self.min_thrust = args.min_thrust  # Minimum thrust threshold; no output below this value

        # State monitoring and diagnostics
        self.current_pose = None
        self.last_pose_time = 0
        self.pose_timeout = 0.5  # Pose timeout threshold (seconds)

        # Debug parameters
        self._t0 = time.time()
        self._print_tick = 0

        rospy.loginfo(f"UnderwaterNavigation ready — target ({self.wp_x}, {self.wp_y}, {self.wp_z})")

    def _pose_callback(self, pose):
        """Store the latest pose data, used to monitor sensor status."""
        self.current_pose = pose
        self.last_pose_time = time.time()

    def check_sensor_health(self):
        """Check whether the pose sensor is working properly."""
        if time.time() - self.last_pose_time > self.pose_timeout:
            rospy.logwarn("⚠ Pose data timed out! Please check the sensor status.")
            return False
        return True

    def check_timeout(self):
        """Check whether any phase has timed out."""
        # Check whether the entire mission has timed out
        if time.time() - self.mission_start_time > self.max_mission_time:
            rospy.logwarn(f"❌ Mission timed out after running {time.time() - self.mission_start_time:.1f} seconds")
            return True

        # Check whether the alignment phase has timed out
        if (self.state == WPState.ALIGN_ATT and
                time.time() - self.align_start_time > self.max_align_time):
            rospy.logwarn(f"⚠ Attitude alignment timed out; forcing transition to the APPROACH phase")
            self.state = WPState.APPROACH
            self.pitch_pid_controller.reset()  # Reset on transition
            self.yaw_pid_controller.reset()  # Reset on transition

        return False

    # ------------------ Core spatial relationship checks -------------------------------
    def is_heading_to_target(self, pose):
        # Current forward direction
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R = tf.quaternion_matrix(q)[:3, :3]
        body_x = R @ np.array([1, 0, 0])
        pos = pose.position
        target_vec = np.array([self.wp_x - pos.x, self.wp_y - pos.y, self.wp_z - pos.z])
        norm = np.linalg.norm(target_vec)
        if norm < 1e-6:  # Already at the point; no check needed
            return True
        target_dir = target_vec / norm
        cos_theta = -float(np.dot(body_x, target_dir))
        angle = math.acos(np.clip(cos_theta, -1, 1))
        # Whether the angle between the nose and the target direction is small enough; True means the nose faces the target
        return angle < self.heading_tol

    # -------------------- Main loop --------------------------------------
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self.state != WPState.DONE:
            # Timeout check
            if self.check_timeout():
                self.emergency_stop("Mission timed out; performing emergency stop")
                break

            # Sensor health check
            if not self.check_sensor_health():
                # Handling for abnormal sensor data
                rospy.logwarn_throttle(1, "Abnormal sensor data; waiting for recovery...")
                self.set_thrusters(0, 0, 0)  # Safety measure: stop the thrusters
                rate.sleep()
                continue

            try:
                pose = rospy.wait_for_message("/nezha_husky/ground_truth/pose", Pose, timeout=0.05)
            except rospy.ROSException:
                rate.sleep()
                continue

            # --------- Current attitude & error ---------------------------------
            pos = pose.position
            roll, pitch, yaw = euler_from_quaternion(
                [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

            dx, dy, dz = (self.wp_x - pos.x, self.wp_y - pos.y, self.wp_z - pos.z)
            dist_xy = math.hypot(dx, dy)
            dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

            if dist_xy > 1e-3:
                self.yaw_sp = math.atan2(dy, dx)
            self.pitch_sp = -math.atan2(dz, max(dist_xy, 0.01))

            self.pitch_sp = self.saturate(self.pitch_sp, -self.pitch_limit, self.pitch_limit)
            print(roll, pitch, yaw)
            print(roll, self.pitch_sp, self.yaw_sp)

            prev_state = self.state

            # --------- State machine ------------------------------------------
            if self.state == WPState.ALIGN_ATT:
                heading_good = self.is_heading_to_target(pose)
                if (abs(self.wrap(self.yaw_sp - yaw)) < self.yaw_tol and
                        abs(self.wrap(self.pitch_sp - pitch)) < self.pitch_tol and
                         heading_good):
                    self.state = WPState.APPROACH
                    self.pitch_pid_controller.reset()  # Reset the PID on state transition
                    self.yaw_pid_controller.reset()
                else:
                    if not heading_good:
                        rospy.logwarn_throttle(2.0, "⚠ Nose is misaligned; prioritizing attitude adjustment toward the target")
                        # A large thrust/PID adjustment may be added here to speed up turning

            elif self.state == WPState.APPROACH:
                if dist_3d < self.decel_radius:
                    self.state = WPState.DECEL

            elif self.state == WPState.DECEL:
                if dist_3d < self.final_tol:
                    self.state = WPState.DONE

            if self.state != prev_state:
                phase_names = {
                    WPState.ALIGN_ATT: "ALIGN_ATT alignment phase",
                    WPState.APPROACH: "APPROACH cruise phase",
                    WPState.DECEL: "DECEL deceleration phase",
                    WPState.DONE: "DONE complete"
                }
                msg = f"✓ {phase_names[prev_state]} → {phase_names[self.state]}"
                rospy.loginfo(msg)
                print(msg, flush=True)

                # Reset the phase timer
                if self.state == WPState.ALIGN_ATT:
                    self.align_start_time = time.time()

            # —— Control dispatch ——
            try:
                if self.state == WPState.ALIGN_ATT:
                    self.align_att_phase(pitch, yaw)
                elif self.state == WPState.APPROACH:
                    self.approach_or_decel_phase(dist_xy, dz, pitch, yaw, decel=False)
                elif self.state == WPState.DECEL:
                    self.approach_or_decel_phase(dist_xy, dz, pitch, yaw, decel=True, dist_3d=dist_3d)
            except Exception as e:
                rospy.logerr(f"Controller execution error: {e}")
                self.set_thrusters(0, 0, 0)  # Safe stop on failure

            # self.debug_print(dist_3d, roll, pitch, yaw)
            rate.sleep()

        self.set_thrusters(0, 0, 0)
        if self.state == WPState.DONE:
            rospy.loginfo("Arrived waypoint ✔")
            print("Arrived waypoint ✔", flush=True)
        else:
            rospy.loginfo("Navigation terminated")
            print("Navigation terminated", flush=True)

    # ------------------------------------------------- Controller phases
    def align_att_phase(self, pitch_now, yaw_now):
        # Correct pitch/yaw error using PID control
        now = time.time()

        # Pitch PID control
        pitch_err = self.wrap(self.pitch_sp - pitch_now)
        T_pitch = self.pitch_pid_controller.update(self.pitch_sp, pitch_now, now)

        # Yaw PID control
        yaw_err = self.wrap(self.yaw_sp - yaw_now)
        delta = self.yaw_pid_controller.update(self.yaw_sp, yaw_now, now)

        # Apply the control output
        self.set_thrusters(T_pitch, delta, -delta)

    def approach_or_decel_phase(self, dist_xy, dz, pitch_now, yaw_now,
                                decel=False, dist_3d=0.0):

        now = time.time()
        pitch_err = self.wrap(self.pitch_sp - pitch_now)
        yaw_err = self.wrap(self.yaw_sp - yaw_now)
        align_ok = abs(pitch_err) < self.pitch_guard

        # If the pitch error is too large, return to the alignment phase
        if not align_ok:
            self.state = WPState.ALIGN_ATT
            self.align_start_time = now  # Reset the alignment timer
            warn = f"⚠ Pitch error too large {math.degrees(pitch_err):.1f}°, returning to ALIGN_ATT"
            rospy.logwarn(warn)
            print(warn, flush=True)
            return

        pitch_factor = math.cos(pitch_err)

        # Compute the forward thrust, accounting for deceleration and energy optimization
        if decel:
            scale = dist_3d / self.decel_radius
            T_fwd = self.T_fwd_max * scale
            # Apply a smooth deceleration curve; a quadratic or custom curve may be chosen
            if self.energy_efficient_mode:
                T_fwd = self.T_fwd_max * (scale ** 2)  # Smoother deceleration curve
        else:
            T_fwd = min(self.K_fwd * dist_xy, self.T_fwd_max)

        # Apply the pitch factor to adjust the forward thrust
        T_fwd *= pitch_factor

        # Energy optimization: no output below the threshold
        if self.energy_efficient_mode and T_fwd < self.min_thrust:
            T_fwd = 0

        # Compute the control output using the PID
        T_pitch = -self.pitch_pid_controller.update(self.pitch_sp, pitch_now, now)
        delta = self.yaw_pid_controller.update(self.yaw_sp, yaw_now, now)

        # Apply the control output
        self.set_thrusters(T_pitch, T_fwd + delta, T_fwd - delta)

    def emergency_stop(self, reason):
        """Emergency-stop all thrusters and log the reason."""
        self.set_thrusters(0, 0, 0)
        rospy.logerr(f"Emergency stop: {reason}")
        print(f"❌ Emergency stop: {reason}", flush=True)

    # --------------------------------- Low-level helper functions
    def set_thrusters(self, T_pitch, T_r, T_l):
        """Set the thruster outputs, applying clamping and energy optimization."""
        T_pitch *= 1.0  # Convention. Change to +1 if the actual direction does not match

        # Energy optimization: no output below the threshold
        if self.energy_efficient_mode:
            T_pitch = 0 if abs(T_pitch) < self.min_thrust else T_pitch
            T_r = 0 if abs(T_r) < self.min_thrust else T_r
            T_l = 0 if abs(T_l) < self.min_thrust else T_l

        now = rospy.Time.now()
        try:
            self.pub_pitch.publish(FloatStamped(Header(stamp=now), T_pitch))
            self.pub_right.publish(FloatStamped(Header(stamp=now), T_r))
            self.pub_left.publish(FloatStamped(Header(stamp=now), T_l))
        except Exception as e:
            rospy.logerr(f"Failed to publish thruster command: {e}")

    def debug_print(self, dist_3d, roll, pitch, yaw):
        if time.time() - self._t0 < 0.2 * self._print_tick:
            return
        self._print_tick += 1
        state_tag = ["ALIGN", "APPRO", "DECEL", "DONE"][self.state]
        msg = (f" d={math.degrees(self.pitch_sp):+.2f} m  "
               f"d={math.degrees(self.yaw_sp):+.2f} m  "
            f"d={dist_3d:+.2f} m  "
               f"R={math.degrees(roll):+.1f}° "
               f"P={math.degrees(pitch):+.1f}° "
               f"Y={math.degrees(yaw):+.1f}°  "
               f"P_err={math.degrees(self.wrap(self.pitch_sp - pitch)):+.1f}° "
               f"Y_err={math.degrees(self.wrap(self.yaw_sp - yaw)):+.1f}°")
        print(msg, flush=True)
        rospy.logdebug(msg)

        # Add PID diagnostic info, logged only at the debug level
        if rospy.get_param("~debug_pid", False):
            pid_msg = (f"PID Pitch: I={self.pitch_pid_controller.integral:.2f} "
                       f"D={self.pitch_pid_controller.prev_error:.2f} "
                       f"Yaw: I={self.yaw_pid_controller.integral:.2f} "
                       f"D={self.yaw_pid_controller.prev_error:.2f}")
            rospy.logdebug(pid_msg)

    @staticmethod
    def wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def saturate(x, lo, hi):
        return max(min(x, hi), lo)


# --------------------- argparse arguments ----------------------
def parse_args():
    parser = argparse.ArgumentParser()
    # Lower the PID gains to reduce thrust
    parser.add_argument('--K_pitch', type=float, default=15.0, help="Pitch P gain")  # Lowered from 15 to 8
    parser.add_argument('--K_pitch_i', type=float, default=0.1, help="Pitch I gain")  # Lowered from 0.1 to 0.05
    parser.add_argument('--K_pitch_d', type=float, default=5.0, help="Pitch D gain")  # Lowered from 5 to 3

    parser.add_argument('--K_yaw', type=float, default=15.0, help="Yaw P gain")  # Lowered from 15 to 8
    parser.add_argument('--K_yaw_i', type=float, default=0.1, help="Yaw I gain")  # Lowered from 0.1 to 0.05
    parser.add_argument('--K_yaw_d', type=float, default=5.0, help="Yaw D gain")  # Lowered from 5 to 3

    parser.add_argument('--K_fwd', type=float, default=10)  # Lowered from 15 to 10
    parser.add_argument('--T_fwd_max', type=float, default=30.0)  # Raised from 25 to 30, but further limited during the mission

    # ... other parameters remain unchanged

    # Attitude limits
    parser.add_argument('--pitch_limit_deg', type=float, default=180)
    parser.add_argument('--pitch_guard_deg', type=float, default=180)
    parser.add_argument('--yaw_tol_deg', type=float, default=180)
    parser.add_argument('--pitch_tol_deg', type=float, default=180)
    parser.add_argument('--roll_tol_deg', type=float, default=180)
    parser.add_argument('--heading_tol_deg', type=float, default=45)  # Heading tolerance angle

    # Navigation parameters
    parser.add_argument('--final_tol', type=float, default=0.50)
    parser.add_argument('--decel_radius', type=float, default=0.60)

    # Waypoint
    parser.add_argument('--wp_x', type=float, default=2.0)
    parser.add_argument('--wp_y', type=float, default=0.0)
    parser.add_argument('--wp_z', type=float, default=-1.0)

    # Timeout parameters
    parser.add_argument('--max_mission_time', type=float, default=3003.0, help="Maximum mission execution time (seconds)")
    parser.add_argument('--max_align_time', type=float, default=3000.0, help="Maximum attitude alignment time (seconds)")

    # Energy optimization
    parser.add_argument('--energy_efficient', action='store_true', help="Enable energy optimization mode")
    parser.add_argument('--min_thrust', type=float, default=1.0, help="Minimum thrust threshold")

    args, unknown = parser.parse_known_args()  # roslaunch compatibility
    return args


# --------------------------------------------------------------------
if __name__ == "__main__":
    try:
        args = parse_args()
        nav = UnderwaterNavigation(args)
        nav.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"❌ Program error: {e}")
        import traceback

        traceback.print_exc()
