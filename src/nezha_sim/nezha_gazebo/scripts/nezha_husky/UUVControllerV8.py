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
from tf import transformations as tf

# nezha_mini_structure
# 侧视图（XZ平面）:
#         ↑ T0 (垂直推进器)
#         │
#     ┌───┴───┐
#     │  CoG  │  ← 质心
#     └───────┘
#    T1→     ←T2
#    (水平推进器)
#
# 顶视图（XY平面）:
#         T0:input value is positive: then the heads go up and turn its back to top
#         │
#     ┌───┴───┐
#     │       │
#     └───────┘
#       T2: input value is positive: then the heading turns right T1 input value is positive: then the heading turns right

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
        # 使用系统时间或提供的时间
        if current_time is None:
            current_time = time.time()

        # 首次调用
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = setpoint - measurement
            return self.kp * self.prev_error  # 仅使用比例项

        # 计算误差和时间差
        error = setpoint - measurement
        dt = current_time - self.prev_time

        # 防止除零和异常的时间差
        if dt <= 0:
            return self.kp * error

        # 计算微分项（带低通滤波）
        derivative = (error - self.prev_error) / dt

        # 积分项（带抗积分饱和）
        self.integral += error * dt
        self.integral = max(min(self.integral, self.windup_limit), -self.windup_limit)

        # 计算PID输出
        output = (self.kp * error +
                  self.ki * self.integral +
                  self.kd * derivative)

        # 限制输出范围
        output = max(min(output, self.output_max), self.output_min)

        # 更新状态
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
        rospy.init_node("nezha_underwater_nav_blocking")

        # 订阅 & 发布
        rospy.Subscriber("/nezha_mini/ground_truth/pose", Pose, self._pose_callback, queue_size=1)
        self.pub_pitch = rospy.Publisher("/nezha_mini/thrusters/0/input",
                                         FloatStamped, queue_size=10)
        self.pub_left = rospy.Publisher("/nezha_mini/thrusters/1/input",
                                        FloatStamped, queue_size=10)
        self.pub_right = rospy.Publisher("/nezha_mini/thrusters/2/input",
                                         FloatStamped, queue_size=10)

        # 参数初始化（从 args 读入）
        # PID参数
        self.pitch_pid_controller = PIDController(
            kp=args.K_pitch,
            ki=args.K_pitch_i,
            kd=args.K_pitch_d,
            output_min=-15.0,  # 改小，匹配 T_fwd_max
            output_max=15.0,
            windup_limit=5.0   # 减小积分限幅
        )
        self.yaw_pid_controller = PIDController(
            kp=args.K_yaw,
            ki=args.K_yaw_i,
            kd=args.K_yaw_d,
            output_min=-10,
            output_max=10,
            windup_limit=5.0  # 防止积分饱和
        )

        self.K_fwd = args.K_fwd
        self.T_fwd_max = args.T_fwd_max

        self.pitch_limit = math.radians(args.pitch_limit_deg)
        self.pitch_guard = math.radians(args.pitch_guard_deg)
        self.yaw_tol = math.radians(args.yaw_tol_deg)
        self.pitch_tol = math.radians(args.pitch_tol_deg)
        self.roll_tol = math.radians(args.roll_tol_deg)
        self.heading_tol = math.radians(args.heading_tol_deg)  # 机头夹角限

        self.final_tol = args.final_tol
        self.decel_radius = args.decel_radius

        # 航点参数
        self.wp_x, self.wp_y, self.wp_z = args.wp_x, args.wp_y, args.wp_z
        self.state = WPState.ALIGN_ATT
        self.yaw_sp = 0.0
        self.pitch_sp = 0.0

        # 超时控制
        self.max_mission_time = args.max_mission_time  # 任务最长执行时间（秒）
        self.max_align_time = args.max_align_time  # 单次姿态调整最长时间
        self.mission_start_time = time.time()
        self.align_start_time = time.time()

        # 能量优化参数
        self.energy_efficient_mode = args.energy_efficient
        self.min_thrust = args.min_thrust  # 最小推力阈值，低于此值时不输出

        # 状态监控和诊断
        self.current_pose = None
        self.last_pose_time = 0
        self.pose_timeout = 0.5  # 位姿超时阈值（秒）

        # 调试参数
        self._t0 = time.time()
        self._print_tick = 0

        rospy.loginfo(f"UnderwaterNavigation ready — target ({self.wp_x}, {self.wp_y}, {self.wp_z})")

    def _pose_callback(self, pose):
        """保存最新位姿数据，用于监控传感器状态"""
        self.current_pose = pose
        self.last_pose_time = time.time()

    def check_sensor_health(self):
        """检查位姿传感器是否正常工作"""
        if time.time() - self.last_pose_time > self.pose_timeout:
            rospy.logwarn("⚠ 位姿数据超时！请检查传感器状态")
            return False
        return True

    def check_timeout(self):
        """检查各阶段是否超时"""
        # 检查整个任务是否超时
        if time.time() - self.mission_start_time > self.max_mission_time:
            rospy.logwarn(f"❌ 任务超时，已执行 {time.time() - self.mission_start_time:.1f} 秒")
            return True

        # 检查对齐阶段是否超时
        if (self.state == WPState.ALIGN_ATT and
                time.time() - self.align_start_time > self.max_align_time):
            rospy.logwarn(f"⚠ 姿态对齐超时，尝试强制进入APPROACH阶段")
            self.state = WPState.APPROACH
            self.pitch_pid_controller.reset()  # 添加这行
            self.yaw_pid_controller.reset()  # 添加这行

        return False

    # ------------------核心空间关系判断-------------------------------
    def is_heading_to_target(self, pose):
        # 当前朝前方向
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        R = tf.quaternion_matrix(q)[:3, :3]
        body_x = R @ np.array([1, 0, 0])
        pos = pose.position
        target_vec = np.array([self.wp_x - pos.x, self.wp_y - pos.y, self.wp_z - pos.z])
        norm = np.linalg.norm(target_vec)
        if norm < 1e-6:  # 就在点上，无需判断
            return True
        target_dir = target_vec / norm
        cos_theta = float(np.dot(body_x, target_dir))
        angle = math.acos(np.clip(cos_theta, -1, 1))
        # 机头与目标方向夹角是否足够小，True即头正对目标
        return angle < self.heading_tol

    # --------------------主循环--------------------------------------
    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and self.state != WPState.DONE:
            # 超时检查
            if self.check_timeout():
                self.emergency_stop("任务超时，执行紧急停止")
                break

            # 传感器健康检查
            if not self.check_sensor_health():
                # 传感器数据异常时的处理
                rospy.logwarn_throttle(1, "传感器数据异常，等待恢复...")
                self.set_thrusters(0, 0, 0)  # 安全措施：停止推进器
                rate.sleep()
                continue

            try:
                pose = rospy.wait_for_message("/nezha_mini/ground_truth/pose", Pose, timeout=0.05)
            except rospy.ROSException:
                rate.sleep()
                continue

            # --------- 当前姿态 & 误差 ---------------------------------
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

            prev_state = self.state

            # --------- 状态机 ------------------------------------------
            if self.state == WPState.ALIGN_ATT:
                heading_good = self.is_heading_to_target(pose)
                if (abs(self.wrap(self.yaw_sp - yaw)) < self.yaw_tol and
                        abs(self.wrap(self.pitch_sp - pitch)) < self.pitch_tol and
                         heading_good):
                    self.state = WPState.APPROACH
                    self.pitch_pid_controller.reset()  # 状态转换时重置PID
                    self.yaw_pid_controller.reset()
                else:
                    if not heading_good:
                        rospy.logwarn_throttle(2.0, "⚠ 机头方向不正，优先调整姿态朝向目标")
                        # 可加入大幅调整推力/PID参数以加速转向

            elif self.state == WPState.APPROACH:
                if dist_3d < self.decel_radius:
                    self.state = WPState.DECEL

            elif self.state == WPState.DECEL:
                if dist_3d < self.final_tol:
                    self.state = WPState.DONE

            if self.state != prev_state:
                phase_names = {
                    WPState.ALIGN_ATT: "ALIGN_ATT 对准阶段",
                    WPState.APPROACH: "APPROACH 航行阶段",
                    WPState.DECEL: "DECEL 减速阶段",
                    WPState.DONE: "DONE 完成"
                }
                msg = f"✓ {phase_names[prev_state]} → {phase_names[self.state]}"
                rospy.loginfo(msg)
                print(msg, flush=True)

                # 重置阶段计时器
                if self.state == WPState.ALIGN_ATT:
                    self.align_start_time = time.time()

            # —— 控制分派 ——
            try:
                if self.state == WPState.ALIGN_ATT:
                    self.align_att_phase(pitch, yaw)
                elif self.state == WPState.APPROACH:
                    self.approach_or_decel_phase(dist_xy, dz, pitch, yaw, decel=False)
                elif self.state == WPState.DECEL:
                    self.approach_or_decel_phase(dist_xy, dz, pitch, yaw, decel=True, dist_3d=dist_3d)
            except Exception as e:
                rospy.logerr(f"控制器执行异常: {e}")
                self.set_thrusters(0, 0, 0)  # 失败时安全停止

            # self.debug_print(dist_3d, roll, pitch, yaw)
            rate.sleep()

        self.set_thrusters(0, 0, 0)
        if self.state == WPState.DONE:
            rospy.loginfo("Arrived waypoint ✔")
            print("Arrived waypoint ✔", flush=True)
        else:
            rospy.loginfo("导航终止")
            print("导航终止", flush=True)

    # ------------------------------------------------- 控制器阶段
    def align_att_phase(self, pitch_now, yaw_now):
        """姿态对齐阶段 - 修正版"""
        now = time.time()
        pos = self.current_pose.position
        dx = self.wp_x - pos.x
        dy = self.wp_y - pos.y
        dz = self.wp_z - pos.z
        dist_xy = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.wrap(target_yaw - yaw_now)
        is_behind = abs(yaw_err) > math.radians(90)

        if is_behind:
            rospy.loginfo_throttle(2.0,
                                   f"⚠ 目标在背后 (偏航误差={math.degrees(yaw_err):.1f}°)，纯转向模式")

            delta = self.yaw_pid_controller.update(target_yaw, yaw_now, now)

            # ✅ 修复：保持水平时不取负号
            T_pitch = self.pitch_pid_controller.update(0.0, pitch_now, now)

            self.set_thrusters(T_pitch, delta, -delta)
        else:
            pitch_sp_safe = -math.atan2(dz, max(dist_xy, 0.01))
            pitch_sp_safe = self.saturate(pitch_sp_safe,
                                          -math.radians(30),
                                          math.radians(30))

            # ✅ 修复：直接使用PID输出，不取负号
            T_pitch = self.pitch_pid_controller.update(pitch_sp_safe, pitch_now, now)
            delta = self.yaw_pid_controller.update(target_yaw, yaw_now, now)

            self.set_thrusters(T_pitch, delta, -delta)

            rospy.loginfo_throttle(2.0,
                                   f"对齐中: pitch_sp={math.degrees(pitch_sp_safe):.1f}° "
                                   f"pitch_now={math.degrees(pitch_now):.1f}° "
                                   f"T_pitch={T_pitch:.2f}")

    def approach_or_decel_phase(self, dist_xy, dz, pitch_now, yaw_now,
                                decel=False, dist_3d=0.0):
        """航行/减速阶段 - 修正版"""
        now = time.time()
        pos = self.current_pose.position
        dx = self.wp_x - pos.x
        dy = self.wp_y - pos.y

        target_yaw = math.atan2(dy, dx)
        pitch_sp_safe = -math.atan2(dz, max(dist_xy, 0.01))
        pitch_sp_safe = self.saturate(pitch_sp_safe,
                                      -math.radians(30),
                                      math.radians(30))

        pitch_err = self.wrap(pitch_sp_safe - pitch_now)
        align_ok = abs(pitch_err) < self.pitch_guard

        if not align_ok:
            self.state = WPState.ALIGN_ATT
            self.align_start_time = now
            self.pitch_pid_controller.reset()
            self.yaw_pid_controller.reset()
            rospy.logwarn(f"⚠ 俯仰偏差过大 {math.degrees(pitch_err):.1f}°，返回对齐")
            return

        pitch_factor = math.cos(pitch_err)

        if decel:
            scale = max(dist_3d / self.decel_radius, 0.1)
            T_fwd = self.T_fwd_max * scale
            if self.energy_efficient_mode:
                T_fwd = self.T_fwd_max * (scale ** 2)
        else:
            T_fwd = min(self.K_fwd * dist_xy, self.T_fwd_max)

        T_fwd *= pitch_factor

        if self.energy_efficient_mode and T_fwd < self.min_thrust:
            T_fwd = 0

        # ✅ 修复：直接使用PID输出
        T_pitch = self.pitch_pid_controller.update(pitch_sp_safe, pitch_now, now)
        delta = self.yaw_pid_controller.update(target_yaw, yaw_now, now)

        self.set_thrusters(T_pitch, T_fwd + delta, T_fwd - delta)

        rospy.loginfo_throttle(2.0,
                               f"航行中: T_fwd={T_fwd:.2f} T_pitch={T_pitch:.2f} "
                               f"pitch={math.degrees(pitch_now):.1f}° "
                               f"dist={dist_3d:.2f}m")

    def emergency_stop(self, reason):
        """紧急停止所有推进器并记录原因"""
        self.set_thrusters(0, 0, 0)
        rospy.logerr(f"紧急停止: {reason}")
        print(f"❌ 紧急停止: {reason}", flush=True)

    # --------------------------------- 底层助手函数
    def set_thrusters(self, T_pitch, T_r, T_l):
        """设置推进器 - 修正版"""
        # ✅ 修复：移除所有符号翻转
        # T_pitch *= 1.0  # 删除这行

        T_pitch = self.saturate(T_pitch, -15.0, 15.0)
        T_r = self.saturate(T_r, -15.0, 15.0)
        T_l = self.saturate(T_l, -15.0, 15.0)

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
            rospy.logerr(f"发布推进器命令失败: {e}")
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

        # 添加PID诊断信息，仅在调试级别记录
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


# --------------------- argparse 参数 ----------------------
def parse_args():
    parser = argparse.ArgumentParser()
    # PID参数
    parser.add_argument('--K_pitch', type=float, default=15.0, help="Pitch P增益")
    parser.add_argument('--K_pitch_i', type=float, default=0.1, help="Pitch I增益")
    parser.add_argument('--K_pitch_d', type=float, default=5.0, help="Pitch D增益")

    parser.add_argument('--K_yaw', type=float, default=15.0, help="Yaw P增益")
    parser.add_argument('--K_yaw_i', type=float, default=0.1, help="Yaw I增益")
    parser.add_argument('--K_yaw_d', type=float, default=5.0, help="Yaw D增益")

    parser.add_argument('--K_fwd', type=float, default=10)
    parser.add_argument('--T_fwd_max', type=float, default=15.0)

    # 姿态限制
    parser.add_argument('--pitch_limit_deg', type=float, default=180)
    parser.add_argument('--pitch_guard_deg', type=float, default=180)
    parser.add_argument('--yaw_tol_deg', type=float, default=180)
    parser.add_argument('--pitch_tol_deg', type=float, default=180)
    parser.add_argument('--roll_tol_deg', type=float, default=180)
    parser.add_argument('--heading_tol_deg', type=float, default=45)  # 机头指向容忍角

    # 导航参数
    parser.add_argument('--final_tol', type=float, default=0.50)
    parser.add_argument('--decel_radius', type=float, default=0.60)

    # 航点
    parser.add_argument('--wp_x', type=float, default=2.0)
    parser.add_argument('--wp_y', type=float, default=0.0)
    parser.add_argument('--wp_z', type=float, default=-1.0)

    # 超时参数
    parser.add_argument('--max_mission_time', type=float, default=3003.0, help="任务最大执行时间(秒)")
    parser.add_argument('--max_align_time', type=float, default=3000.0, help="姿态对准最长时间(秒)")

    # 能量优化
    parser.add_argument('--energy_efficient', action='store_true', help="启用能量优化模式")
    parser.add_argument('--min_thrust', type=float, default=1.0, help="最小推力阈值")

    args, unknown = parser.parse_known_args()  # roslaunch 兼容
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
        rospy.logerr(f"❌ 程序异常: {e}")
        import traceback

        traceback.print_exc()
