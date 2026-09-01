#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
import rospy
import math
import time
import numpy as np
import csv
import os
from datetime import datetime
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseStamped
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header, Bool
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# =====================================================================
# Import the correct service types (used to obtain hydrodynamics and wave data)
# =====================================================================
try:
    from nezha_plugins.srv import HydrodynamicsForces, GetPhaseSample
except ImportError:
    HydrodynamicsForces = None
    GetPhaseSample = None
    print("⚠️ Warning: failed to import nezha_plugins.srv.")

def quaternion_from_euler(roll, pitch, yaw):
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

        self.current_pose = None
        self.prev_thrust_left = 0.0
        self.prev_thrust_right = 0.0
        self.prev_thrust_pitch = 0.0
        self.smoothing_factor = 0.6  
        self.enable_buoyancy_compensation = True  
        self.buoyancy_compensation = 0 

        # Trajectory tracking parameters
        self.current_waypoint_index = 0
        self.waypoint_tolerance = 1.5
        self.lookahead_distance = 4.0
        self.fixed_y = 0.0
        self.current_epoch = 0

        # ========== 🚁 Transmedia state machine parameters ==========
        self.WATER_SURFACE_Z = 0.05
        self.MOTOR_STOP_TIMEOUT = 20.0
        self.motor_stop_time = None

        self.STATE_AERIAL = "AERIAL"
        self.STATE_MOTOR_STOP = "MOTOR_STOP"
        self.STATE_UNDERWATER = "UNDERWATER"
        self.state = self.STATE_AERIAL

        # ============================================================================
        # 📊 Data logger initialization (Service-based)
        # ============================================================================
        self.save_dir = "processed_data_100pts"
        os.makedirs(self.save_dir, exist_ok=True)
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(self.save_dir, f"trajectory_data_with_forces_{timestamp_str}.csv")
        
        self.csv_file = open(self.csv_filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        self.csv_writer.writerow([
            "timestamp", "robot_type", "x", "y", "z",
            "buoyancy_x", "buoyancy_y", "buoyancy_z",
            "damping_x", "damping_y", "damping_z",
            "added_mass_x", "added_mass_y", "added_mass_z",
            "coriolis_x", "coriolis_y", "coriolis_z",
            "wave_x", "wave_y", "wave_z",
            "surface_z", "submersion_ratio"
        ])
        
        self.forces = {
            'buoyancy': [0.0, 0.0, 0.0], 'damping': [0.0, 0.0, 0.0],
            'added_mass': [0.0, 0.0, 0.0], 'coriolis': [0.0, 0.0, 0.0],
            'wave': [0.0, 0.0, 0.0], 'submersion_ratio': 0.0,
            'surface_z': 0.0
        }

        # Connect to services
        self.force_srv = self._connect_service('/nezha_mini/get_hydrodynamics_forces', HydrodynamicsForces)
        self.phase_srv = self._connect_service('/nezha_mini/transmedia/get_phase_sample', GetPhaseSample)

        # Load the trajectory
        self.trajectory = self.load_trajectory(trajectory_file)

        rospy.Subscriber("/nezha_mini/ground_truth/pose", Pose, self._pose_callback, queue_size=1)

        # 🌊 Underwater thruster publishers
        self.pub_pitch = rospy.Publisher("/nezha_mini/thrusters/0/input", FloatStamped, queue_size=10)
        self.pub_left = rospy.Publisher("/nezha_mini/thrusters/1/input", FloatStamped, queue_size=10)
        self.pub_right = rospy.Publisher("/nezha_mini/thrusters/2/input", FloatStamped, queue_size=10)

        # 🚁 Aerial propeller publishers
        self.pub_motor_stop = rospy.Publisher("/nezha_mini/motor_stop_cmd", Bool, queue_size=10)
        self.pose_cmd_pub = rospy.Publisher("/nezha_mini/command/pose", PoseStamped, queue_size=10)

        from UUVControllerV8 import PIDController

        self.pitch_pid = PIDController(kp=90.0, ki=8.0, kd=70.0, output_min=-50.0, output_max=50.0, windup_limit=30.0)
        self.yaw_pid = PIDController(kp=25.0, ki=2.0, kd=10.0, output_min=-15.0, output_max=15.0, windup_limit=8.0)
        self.position_pid = PIDController(kp=18.0, ki=2.0, kd=10.0, output_min=-30.0, output_max=30.0, windup_limit=12.0)
        self.depth_pid = PIDController(kp=50.0, ki=5.0, kd=60.0, output_min=-40.0, output_max=40.0, windup_limit=15.0)

        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        print(f"✓ Gazebo model reset service connected | data will be saved to: {self.csv_filename}")

    # ============================================================================
    # 📊 Data logging service methods
    # ============================================================================
    def _connect_service(self, srv_name, srv_type):
        if srv_type is None:
            return None
        print(f"⏳ Waiting for service {srv_name}...")
        try:
            rospy.wait_for_service(srv_name, timeout=3.0)
            srv = rospy.ServiceProxy(srv_name, srv_type, persistent=True)
            print(f"✅ Service {srv_name} connected successfully!")
            return srv
        except rospy.ROSException:
            print(f"⚠️ Service {srv_name} connection timed out.")
            return None

    def update_forces_from_service(self):
        """Update force data in real time by calling the service"""
        if self.force_srv is not None:
            try:
                resp = self.force_srv()
                self.forces['buoyancy'] = [resp.buoyancy_x, resp.buoyancy_y, resp.buoyancy_z]
                self.forces['damping'] = [resp.damping_x, resp.damping_y, resp.damping_z]
                self.forces['added_mass'] = [resp.added_mass_x, resp.added_mass_y, resp.added_mass_z]
                self.forces['coriolis'] = [resp.coriolis_x, resp.coriolis_y, resp.coriolis_z]
                self.forces['wave'] = [resp.wave_x, resp.wave_y, resp.wave_z]
                self.forces['submersion_ratio'] = resp.submersion_ratio
            except rospy.ServiceException:
                self.force_srv = self._connect_service('/nezha_mini/get_hydrodynamics_forces', HydrodynamicsForces)
            except Exception:
                pass

        if self.phase_srv is not None:
            try:
                resp = self.phase_srv()
                self.forces['surface_z'] = resp.surface_z
            except rospy.ServiceException:
                self.phase_srv = self._connect_service('/nezha_mini/transmedia/get_phase_sample', GetPhaseSample)
            except Exception:
                pass

    def log_current_state(self):
        """Write the current state to the CSV"""
        if self.current_pose is None:
            return
            
        t = rospy.Time.now().to_sec()
        x, y, z = self.get_current_position()
        robot_type = "UAV" if self.state in [self.STATE_AERIAL, self.STATE_MOTOR_STOP] else "UUV"

        try:
            row = [
                t, robot_type, x, y, z,
                *self.forces['buoyancy'],
                *self.forces['damping'],
                *self.forces['added_mass'],
                *self.forces['coriolis'],
                *self.forces['wave'],
                self.forces['surface_z'], 
                self.forces['submersion_ratio']
            ]
            self.csv_writer.writerow(row)
        except Exception:
            pass

    def close_logger(self):
        """关闭日志文件"""
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
            print(f"\n💾 [Data Logger] 数据已保存至: {self.csv_filename}")

    # ============================================================================
    # 基础控制方法 (保持不变)
    # ============================================================================
    def load_trajectory(self, filename):
        trajectory = []
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    trajectory.append([float(row['nezha_x']), float(row['nezha_z'])])
            data = np.array(trajectory)
            print(f"成功加载轨迹文件: {filename}")
            print(f"轨迹数据形状: {data.shape}")
            return data
        except Exception as e:
            print(f"加载轨迹文件失败: {e}")
            return None

    def _pose_callback(self, pose):
        self.current_pose = pose

    def get_current_angles(self):
        if self.current_pose is None:
            return None, None, None
        from UUVControllerV8 import euler_from_quaternion
        q = [self.current_pose.orientation.x, self.current_pose.orientation.y,
             self.current_pose.orientation.z, self.current_pose.orientation.w]
        return euler_from_quaternion(q)

    def get_current_position(self):
        if self.current_pose is None:
            return None, None, None
        return (self.current_pose.position.x, self.current_pose.position.y, self.current_pose.position.z)

    def stop_thrusters(self):
        self.set_thrusters_smooth(0, 0, 0)

    def set_thrusters_smooth(self, T_pitch, T_left, T_right):
        T_pitch = max(min(T_pitch, 50.0), -50.0)
        T_r = max(min(T_right, 40.0), -40.0)
        T_l = max(min(T_left, 40.0), -40.0)

        alpha = self.smoothing_factor
        T_pitch_smooth = alpha * self.prev_thrust_pitch + (1 - alpha) * T_pitch
        T_l_smooth = alpha * self.prev_thrust_left + (1 - alpha) * T_l
        T_r_smooth = alpha * self.prev_thrust_right + (1 - alpha) * T_r

        self.prev_thrust_pitch = T_pitch_smooth
        self.prev_thrust_left = T_l_smooth
        self.prev_thrust_right = T_r_smooth

        now = rospy.Time.now()
        self.pub_pitch.publish(FloatStamped(Header(stamp=now), T_pitch_smooth))
        self.pub_right.publish(FloatStamped(Header(stamp=now), T_r_smooth))
        self.pub_left.publish(FloatStamped(Header(stamp=now), T_l_smooth))

    def _reset_pids(self):
        self.position_pid.reset()
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.depth_pid.reset()

    # ============================================================================
    # 🛑 自由落水逻辑 (加入数据记录)
    # ============================================================================
    def _run_motor_stop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pub_motor_stop.publish(Bool(data=True))
            self.stop_thrusters()

            x, y, z = self.get_current_position()
            elapsed = time.time() - self.motor_stop_time

            # 📝 记录数据
            self.update_forces_from_service()
            self.log_current_state()

            print(f"🛑 MOTOR_STOP | Z:{z:.3f} 目标Z<{self.WATER_SURFACE_Z} elapsed:{elapsed:.1f}s", end='\r')

            if elapsed > self.MOTOR_STOP_TIMEOUT and z > self.WATER_SURFACE_Z:
                if int(elapsed * 10) % 30 == 0:
                    rospy.logwarn(f"\n⚠️ 已等待 {elapsed:.0f}s，Z={z:.3f} 仍未入水！请检查浮力或motor_stop_cmd话题")

            if z < self.WATER_SURFACE_Z:
                rospy.loginfo(f"\n🌊 接触水面确认 Z={z:.3f} (耗时{elapsed:.1f}s)，开启水桨主动下潜！")
                self._reset_pids()
                self.state = self.STATE_UNDERWATER
                break

            rate.sleep()

    def calculate_distance_xz(self, x1, z1, x2, z2):
        return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

    def find_lookahead_waypoint(self):
        current_x, _, current_z = self.get_current_position()
        for i in range(self.current_waypoint_index, len(self.trajectory)):
            wp_x, wp_z = self.trajectory[i]
            dist = self.calculate_distance_xz(current_x, current_z, wp_x, wp_z)
            if dist >= self.lookahead_distance:
                return i, self.trajectory[i]
        return len(self.trajectory) - 1, self.trajectory[-1]

    # ============================================================================
    # 🚁 核心导航逻辑 (加入数据记录)
    # ============================================================================
    def navigate_to_waypoint(self, target_x, target_z, target_y=None, distance_tolerance=1.0, timeout=60.0):
        if target_y is None:
            target_y = self.fixed_y

        self._reset_pids()
        rate = rospy.Rate(50)
        start_time = time.time()
        DEPTH_TOLERANCE = 0.3
        last_print_time = time.time()
        PRINT_INTERVAL = 2.0

        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - start_time > timeout:
                return False

            current_x, current_y, current_z = self.get_current_position()
            roll, pitch, yaw = self.get_current_angles()
            
            # 📝 记录数据 (50Hz)
            self.update_forces_from_service()
            self.log_current_state()

            dx = target_x - current_x
            dy = target_y - current_y
            dz = target_z - current_z

            distance_xz = math.sqrt(dx ** 2 + dz ** 2)
            dist_xy = math.sqrt(dx ** 2 + dy ** 2)
            distance_z = abs(dz)

            if self.state == self.STATE_UNDERWATER:
                if distance_xz < distance_tolerance and distance_z < DEPTH_TOLERANCE:
                    break

            if self.state == self.STATE_AERIAL:
                self.stop_thrusters() 
                self.pub_motor_stop.publish(Bool(data=False)) 

                if current_z < 0.3 and dist_xy < 1.0:
                    rospy.loginfo(f"\n🛑 到达入水点 (dist_xy={dist_xy:.2f}m, Z={current_z:.2f})，关闭飞桨，等待自由落水...")
                    self.motor_stop_time = time.time()
                    self.state = self.STATE_MOTOR_STOP
                    self._run_motor_stop()
                    return True 

                aerial_target_z = 0.2
                dx_a = target_x - current_x
                dy_a = target_y - current_y
                dz_a = aerial_target_z - current_z
                
                dist_3d = math.sqrt(dx_a**2 + dy_a**2 + dz_a**2)
                
                CARROT_DIST = 0.3  
                if dist_3d > CARROT_DIST:
                    carrot_x = current_x + (dx_a / dist_3d) * CARROT_DIST
                    carrot_y = current_y + (dy_a / dist_3d) * CARROT_DIST
                    carrot_z = current_z + (dz_a / dist_3d) * CARROT_DIST
                else:
                    carrot_x = target_x
                    carrot_y = target_y
                    carrot_z = aerial_target_z

                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "world"
                pose_msg.pose.position.x = carrot_x
                pose_msg.pose.position.y = carrot_y
                pose_msg.pose.position.z = carrot_z
                
                if self.current_pose:
                    pose_msg.pose.orientation = self.current_pose.orientation
                else:
                    pose_msg.pose.orientation.w = 1.0
                    
                self.pose_cmd_pub.publish(pose_msg)

                if current_time - last_print_time > PRINT_INTERVAL:
                    print(f"[🚁空中慢飞] X:{current_x:+.2f}→{target_x:+.2f} | Z:{current_z:+.2f}→{aerial_target_z:+.2f} | 水平距离:{dist_xy:.2f}m")
                    last_print_time = current_time

            elif self.state == self.STATE_UNDERWATER:
                self.pub_motor_stop.publish(Bool(data=True)) 

                raw_pid_output = self.depth_pid.update(current_z, target_z, current_time)
                MAX_DEPTH_CORRECTION = 45
                pitch_correction_deg = max(min(raw_pid_output, MAX_DEPTH_CORRECTION), -MAX_DEPTH_CORRECTION)
                if abs(dz) < 0.05:
                    pitch_correction_deg = 0.0
                pitch_correction_rad = math.radians(pitch_correction_deg)

                horizontal_base = max(abs(dx), 0.1)
                target_pitch_geometry_raw = math.atan2(dz, horizontal_base)
                distance_weight = 1.0 if distance_xz > 3.0 else (0.5 if abs(dz) > 0.5 else 0.75) if distance_xz > 1.5 else 0.5
                target_pitch_geometry = target_pitch_geometry_raw * distance_weight

                target_pitch_final = target_pitch_geometry + pitch_correction_rad
                GLOBAL_MAX_PITCH = math.radians(20.0) if abs(dz) > 0.8 else math.radians(10.0)
                target_pitch_final = max(min(target_pitch_final, GLOBAL_MAX_PITCH), -GLOBAL_MAX_PITCH)

                pid_output = self.pitch_pid.update(target_pitch_final, pitch, current_time)
                pitch_output = max(min(-pid_output + (self.buoyancy_compensation if self.enable_buoyancy_compensation else 0.0), 40.0), -40.0)

                thrust_raw = self.position_pid.update(0.0, distance_xz, current_time)
                min_thrust = 4.0 * max(0.0, min(distance_xz / 2.0, 1.0))
                thrust = max(min(abs(thrust_raw), 10.0), min_thrust)

                target_yaw = math.atan2(dy, dx) if abs(dx) > 0.1 or abs(dy) > 0.1 else yaw
                yaw_error = math.atan2(math.sin(target_yaw - yaw), math.cos(target_yaw - yaw))
                yaw_correction = max(min(-yaw_error * 20.0, 15.0), -15.0)

                if current_time - last_print_time > PRINT_INTERVAL:
                    depth_direction = "⬆️上浮" if dz < 0 else "⬇️下潜" if dz > 0 else "➡️平行"
                    print(f"[🌊水下航行] X:{current_x:+.2f}→{target_x:+.2f} | Z:{current_z:+.2f}→{target_z:+.2f} {depth_direction}")
                    last_print_time = current_time

                self.set_thrusters_smooth(pitch_output, thrust + yaw_correction, thrust - yaw_correction)

            rate.sleep()

        return True

    def track_trajectory_with_lookahead(self):
        if self.trajectory is None or len(self.trajectory) == 0:
            print("轨迹为空,无法跟踪!")
            return False

        print(f"\n{'#' * 60}")
        print(f"# 开始跨域轨迹跟踪 (空地协同)")
        print(f"{'#' * 60}\n")

        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        _, _, start_z = self.get_current_position()
        if start_z > self.WATER_SURFACE_Z:
            self.state = self.STATE_AERIAL
            rospy.loginfo(f"检测到初始高度 Z={start_z:.2f} 在空中，启用飞桨控制...")
        else:
            self.state = self.STATE_UNDERWATER
            rospy.loginfo(f"检测到初始高度 Z={start_z:.2f} 在水下，直接启用跨域水桨控制...")

        self.current_waypoint_index = 0
        start_time = time.time()
        last_lookahead_idx = 0

        while not rospy.is_shutdown():
            if self.current_waypoint_index >= len(self.trajectory):
                print(f"\n{'=' * 60}\n✓✓✓ 轨迹跟踪完成! 总用时: {time.time() - start_time:.1f}秒\n{'=' * 60}\n")
                break

            lookahead_idx, lookahead_wp = self.find_lookahead_waypoint()
            target_x, target_z = lookahead_wp[0], lookahead_wp[1]

            if lookahead_idx < last_lookahead_idx:
                lookahead_idx = self.current_waypoint_index
                target_x, target_z = self.trajectory[lookahead_idx]

            last_lookahead_idx = lookahead_idx

            print(f"\n{'─' * 60}\n📍 Waypoint {self.current_waypoint_index + 1}/{len(self.trajectory)}\n{'─' * 60}")
            
            success = self.navigate_to_waypoint(target_x, target_z, target_y=self.fixed_y, distance_tolerance=self.waypoint_tolerance, timeout=30.0)

            if success:
                self.current_waypoint_index = min(self.current_waypoint_index + 5, lookahead_idx + 1, len(self.trajectory))
            else:
                self.current_waypoint_index += 1

        self.stop_thrusters()
        return True

if __name__ == "__main__":
    tracker = None
    try:
        NUM_REPEATS = 1

        print(f"\n{'#' * 70}")
        print(f"# 开始 {NUM_REPEATS} 次轨迹跟踪循环（包含跨域入水）")
        print(f"{'#' * 70}\n")

        tracker = UUVTrajectoryTracker("Field_data_distance.csv")
        rospy.sleep(2.0)

        for repeat_count in range(1, NUM_REPEATS + 1):
            tracker.current_epoch = repeat_count
            success = tracker.track_trajectory_with_lookahead()

            if success:
                print(f"\n✅ 第 {repeat_count} 次轨迹跟踪完成")
            else:
                print(f"\n⚠️ 第 {repeat_count} 次轨迹跟踪失败")

        tracker.stop_thrusters()

    except rospy.ROSInterruptException:
        pass
    finally:
        if tracker is not None:
            tracker.close_logger()

