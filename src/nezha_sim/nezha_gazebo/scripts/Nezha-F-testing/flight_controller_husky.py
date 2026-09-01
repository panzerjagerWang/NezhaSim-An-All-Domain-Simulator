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
import threading
import time
from collections import deque
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry

# Matplotlib configuration
import matplotlib

matplotlib.use('TkAgg')  # interactive backend
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D


class RealtimePlotter:
    """Real-time 3D trajectory plotter (separate-thread version - UAV specific)"""

    def __init__(self, robot_name="UAV", max_points=None, update_interval=100):
        self.robot_name = robot_name
        self.max_points = max_points
        self.update_interval = update_interval

        # Data buffer - 3D
        self.x_data = deque(maxlen=max_points) if max_points else []
        self.y_data = deque(maxlen=max_points) if max_points else []
        self.z_data = deque(maxlen=max_points) if max_points else []
        self.target_x = None
        self.target_y = None
        self.target_z = None
        self.current_epoch = 1

        # Thread control
        self.lock = threading.Lock()
        self.running = True
        self.plot_thread = None
        self.initialized = False
        self.fig = None
        self.ax = None

        print(f"🎨 Starting {robot_name} plotting thread...")
        self.start_plotting_thread()

        # Wait for initialization to complete
        timeout = 10.0
        start_time = time.time()
        while not self.initialized and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if self.initialized:
            print(f"✓ {robot_name} plotting thread started")
        else:
            print(f"⚠️ {robot_name} plotting thread startup timed out")

    def start_plotting_thread(self):
        """Run matplotlib in a separate thread"""
        self.plot_thread = threading.Thread(target=self._plot_loop, daemon=True)
        self.plot_thread.start()

    def _plot_loop(self):
        """Main plotting loop"""
        try:
            print(f"  → Plotting thread started, initializing matplotlib...")

            # 🔥 Key step: create the 3D figure
            self.fig = plt.figure(figsize=(14, 10))
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.setup_plot()

            # Initialize the 3D lines
            self.line_actual, = self.ax.plot(
                [], [], [], color='#2E86DE', linewidth=2.5,
                label='Actual Trajectory', zorder=3
            )
            self.line_target, = self.ax.plot(
                [], [], [], color='#EE5A6F', linewidth=2,
                linestyle='--', alpha=0.6,
                label='Target Trajectory', zorder=2
            )
            self.point_current, = self.ax.plot(
                [], [], [], 'o', color='#26DE81', markersize=10,
                markeredgecolor='white', markeredgewidth=2,
                label='Current Position', zorder=4
            )

            self.ax.legend(loc='upper right', fontsize=11,
                           framealpha=0.95, shadow=True)

            self.initialized = True
            print(f"  ✓ matplotlib initialization complete (3D mode)")

            # 🔥 Use FuncAnimation for automatic updates
            self.ani = FuncAnimation(
                self.fig,
                self.update_plot,
                interval=self.update_interval,
                blit=False,
                cache_frame_data=False
            )

            print(f"  ✓ Animation loop started, the window should be visible")

            # 🔥 Key step: show the window (blocks the current thread)
            plt.show(block=True)

        except Exception as e:
            print(f"❌ {self.robot_name} plotting thread error: {e}")
            import traceback
            traceback.print_exc()
            self.initialized = False

    def setup_plot(self):
        """Set up the plot style"""
        # 3D axis labels
        self.ax.set_xlabel('X-direction (m)', fontsize=10, fontweight='bold', labelpad=10)
        self.ax.set_ylabel('Y-direction (m)', fontsize=10, fontweight='bold', labelpad=10)
        self.ax.set_zlabel('Z-direction (m)', fontsize=10, fontweight='bold', labelpad=10)

        self.ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)

        # Set the initial viewing angle
        self.ax.view_init(elev=20, azim=45)  # elevation 20 degrees, azimuth 45 degrees

        # Set the axis ranges
        self.ax.set_xlim(-2, 35)
        self.ax.set_ylim(-10, 5)
        self.ax.set_zlim(-1, 8)

    def set_target_trajectory(self, trajectory):
        """设置目标轨迹"""
        with self.lock:
            self.target_x = trajectory[:, 0]
            self.target_y = trajectory[:, 1]
            self.target_z = trajectory[:, 2]

    def add_point(self, x, y, z):
        """添加新数据点"""
        with self.lock:
            if isinstance(self.x_data, deque):
                self.x_data.append(x)
                self.y_data.append(y)
                self.z_data.append(z)
            else:
                self.x_data.append(x)
                self.y_data.append(y)
                self.z_data.append(z)
                if self.max_points and len(self.x_data) > self.max_points:
                    self.x_data.pop(0)
                    self.y_data.pop(0)
                    self.z_data.pop(0)

    def set_epoch(self, epoch):
        """设置当前轮次"""
        with self.lock:
            self.current_epoch = epoch
            # 更新标题
            if self.ax is not None:
                self.ax.set_title(
                    f'Real-time {self.robot_name} Trajectory (3D) - Epoch {self.current_epoch}',
                    fontsize=16, fontweight='bold', pad=20
                )

    def clear_trajectory(self):
        """清空轨迹数据"""
        with self.lock:
            if isinstance(self.x_data, deque):
                self.x_data.clear()
                self.y_data.clear()
                self.z_data.clear()
            else:
                self.x_data = []
                self.y_data = []
                self.z_data = []
        print(f"✓ {self.robot_name}轨迹数据已清空")

    def update_plot(self, frame):
        """更新图表（由FuncAnimation调用）"""
        with self.lock:
            # 更新目标轨迹（3D）
            if self.target_x is not None:
                self.line_target.set_data(self.target_x, self.target_y)
                self.line_target.set_3d_properties(self.target_z)

            # 更新实际轨迹（3D）
            if len(self.x_data) > 0:
                self.line_actual.set_data(list(self.x_data), list(self.y_data))
                self.line_actual.set_3d_properties(list(self.z_data))

                self.point_current.set_data([self.x_data[-1]], [self.y_data[-1]])
                self.point_current.set_3d_properties([self.z_data[-1]])

            # 动态调整坐标轴范围（3D）
            if len(self.x_data) > 1 or self.target_x is not None:
                x_vals = []
                y_vals = []
                z_vals = []

                if len(self.x_data) > 0:
                    x_vals.extend(self.x_data)
                    y_vals.extend(self.y_data)
                    z_vals.extend(self.z_data)

                if self.target_x is not None:
                    x_vals.extend(self.target_x)
                    y_vals.extend(self.target_y)
                    z_vals.extend(self.target_z)

                if x_vals and y_vals and z_vals:
                    x_min, x_max = min(x_vals), max(x_vals)
                    y_min, y_max = min(y_vals), max(y_vals)
                    z_min, z_max = min(z_vals), max(z_vals)

                    x_margin = max((x_max - x_min) * 0.1, 1.0)
                    y_margin = max((y_max - y_min) * 0.1, 1.0)
                    z_margin = max((z_max - z_min) * 0.1, 0.5)

                    self.ax.set_xlim(x_min - x_margin, x_max + x_margin)
                    self.ax.set_ylim(y_min - y_margin, y_max + y_margin)
                    self.ax.set_zlim(z_min - z_margin, z_max + z_margin)

        return self.line_actual, self.line_target, self.point_current

    def save_figure(self, filename):
        """保存图表"""
        try:
            with self.lock:
                if self.fig is not None:
                    self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                    print(f"✓ 图表已保存: {filename}")
        except Exception as e:
            print(f"⚠️ 保存图表失败: {e}")

    def close(self):
        """关闭绘图窗口"""
        try:
            self.running = False
            if self.fig is not None:
                plt.close(self.fig)
            if self.plot_thread is not None:
                self.plot_thread.join(timeout=2.0)
            print(f"✓ {self.robot_name}绘图窗口已关闭")
        except Exception as e:
            print(f"⚠️ 关闭图表失败: {e}")


class PIDController:
    """PID控制器"""

    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time):
        """计算PID输出"""
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = (current_time - self.prev_time).to_sec()
        if dt <= 0:
            return 0.0

        # 比例项
        p_term = self.kp * error

        # 积分项（带抗饱和）
        self.integral += error * dt
        if self.output_limit is not None:
            self.integral = np.clip(
                self.integral,
                -self.output_limit / self.ki if self.ki != 0 else -1e6,
                self.output_limit / self.ki if self.ki != 0 else 1e6
            )
        i_term = self.ki * self.integral

        # 微分项
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # 总输出
        output = p_term + i_term + d_term

        # 限幅
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)

        self.prev_error = error
        self.prev_time = current_time

        return output


class GlobalTrajectoryController:
    def __init__(self):
        rospy.init_node('global_trajectory_controller', anonymous=True)

        # 发布器：发布轨迹命令
        self.traj_pub = rospy.Publisher('/nezha_husky1/command/trajectory',
                                        MultiDOFJointTrajectory,
                                        queue_size=10)

        # 订阅器：获取当前位置信息
        self.odom_sub = rospy.Subscriber('/nezha_husky1/ground_truth/odometry',
                                         Odometry,
                                         self.odom_callback)

        # 当前状态
        self.current_position = Vector3(0, 0, 0)
        self.current_velocity = Vector3(0, 0, 0)

        # 航点列表
        self.waypoints = []

        # **数据记录相关**
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.velocity_x = []
        self.velocity_y = []
        self.velocity_z = []

        self.max_records = 100
        self.recording = False
        self.record_counter = 0
        self.target_records = 0
        self.current_epoch = 0

        # 数据记录锁
        self.first_waypoint_recorded = False
        self.last_waypoint_recorded = False

        # 里程计频率测量
        self.odom_callback_count = 0
        self.odom_start_time = None

        # 🔥 实时绘图器
        self.plotter = None

        rospy.sleep(1.0)
        rospy.loginfo("全局轨迹控制器初始化完成")

    def odom_callback(self, msg):
        """里程计回调函数，更新当前位置并记录数据"""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear

        # 🔥 实时更新绘图（3D）
        if self.plotter is not None:
            self.plotter.add_point(
                self.current_position.x,
                self.current_position.y,
                self.current_position.z
            )

        # 频率测量
        if self.odom_start_time is not None:
            self.odom_callback_count += 1

        # 数据记录逻辑
        if self.recording and len(self.position_x) < self.max_records:
            self.record_counter += 1
            current_recorded = len(self.position_x) - 1

            if current_recorded >= self.target_records:
                return

            if self.target_records > 0 and self.total_expected_callbacks > 0:
                expected_records = int(
                    (self.record_counter / float(self.total_expected_callbacks)) * self.target_records)

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
                            f"已记录 {len(self.position_x)}/{self.max_records} 个数据点 (进度: {progress:.1f}%)")

    def record_current_state(self):
        """手动记录当前状态（位置和速度）"""
        self.position_x.append(self.current_position.x)
        self.position_y.append(self.current_position.y)
        self.position_z.append(self.current_position.z)

        self.velocity_x.append(self.current_velocity.x)
        self.velocity_y.append(self.current_velocity.y)
        self.velocity_z.append(self.current_velocity.z)

        rospy.loginfo(f"✓ 手动记录数据点 {len(self.position_x)}/{self.max_records}")

    def save_to_txt(self, data_list, filename, mode='a'):
        """将列表数据保存到txt文件"""
        try:
            with open(filename, mode) as f:
                formatted_data = '[' + ','.join([f"{value:.6f}" for value in data_list]) + '],\n'
                f.write(formatted_data)
            rospy.loginfo(f"✓ {filename} - Epoch {self.current_epoch} (共 {len(data_list)} 个数据点)")
        except Exception as e:
            rospy.logerr(f"✗ 保存文件 {filename} 失败: {e}")

    def save_recorded_data(self):
        """保存记录的数据到6个不同的txt文件"""
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"保存 Epoch {self.current_epoch} 的飞行数据...")
        rospy.loginfo("-" * 60)

        mode = 'w' if self.current_epoch == 1 else 'a'

        rospy.loginfo("位置数据:")
        self.save_to_txt(self.position_x, 'position_x.txt', mode)
        self.save_to_txt(self.position_y, 'position_y.txt', mode)
        self.save_to_txt(self.position_z, 'position_z.txt', mode)

        rospy.loginfo("-" * 60)

        rospy.loginfo("速度数据:")
        self.save_to_txt(self.velocity_x, 'velocity_x.txt', mode)
        self.save_to_txt(self.velocity_y, 'velocity_y.txt', mode)
        self.save_to_txt(self.velocity_z, 'velocity_z.txt', mode)

        rospy.loginfo("-" * 60)
        rospy.loginfo(f"Epoch {self.current_epoch} 数据保存完成！")
        rospy.loginfo("=" * 60)

    def reset_for_next_epoch(self):
        """重置数据以准备下一个epoch"""
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

        # 🔥 清空绘图数据
        if self.plotter is not None:
            self.plotter.clear_trajectory()

        rospy.loginfo("数据已重置，准备下一个epoch...")

    def ensure_exact_record_count(self):
        """确保数据点数精确为100个"""
        current_count = len(self.position_x)

        if current_count == self.max_records:
            rospy.loginfo(f"✓ 数据点数正确: {current_count}/{self.max_records}")
            return

        if current_count < self.max_records:
            shortage = self.max_records - current_count
            rospy.logwarn(f"⚠ 数据点不足！当前: {current_count}, 目标: {self.max_records}, 缺少: {shortage}")
            rospy.loginfo("开始补充采集...")

            rate = rospy.Rate(10)
            补充计数 = 0

            while len(self.position_x) < self.max_records and not rospy.is_shutdown():
                self.position_x.append(self.current_position.x)
                self.position_y.append(self.current_position.y)
                self.position_z.append(self.current_position.z)
                self.velocity_x.append(self.current_velocity.x)
                self.velocity_y.append(self.current_velocity.y)
                self.velocity_z.append(self.current_velocity.z)

                rate.sleep()
                补充计数 += 1
                if 补充计数 % 10 == 0:
                    rospy.loginfo(f"补充采集中... {len(self.position_x)}/{self.max_records}")

            rospy.loginfo(f"✓ 补充采集完成: {len(self.position_x)}/{self.max_records}")

        elif current_count > self.max_records:
            excess = current_count - self.max_records
            rospy.logwarn(f"⚠ 数据点过多！当前: {current_count}, 目标: {self.max_records}, 多余: {excess}")
            rospy.loginfo("裁剪多余数据...")

            self.position_x = self.position_x[:self.max_records]
            self.position_y = self.position_y[:self.max_records]
            self.position_z = self.position_z[:self.max_records]
            self.velocity_x = self.velocity_x[:self.max_records]
            self.velocity_y = self.velocity_y[:self.max_records]
            self.velocity_z = self.velocity_z[:self.max_records]

            rospy.loginfo(f"✓ 裁剪完成: {len(self.position_x)}/{self.max_records}")

    def return_to_start_with_pid(self, timeout=20.0, position_threshold=0.5):
        """使用PID控制精确返回到第一个航点位置"""
        if len(self.waypoints) == 0:
            rospy.logerr("没有航点信息，无法返回起点")
            return False

        start_waypoint = self.waypoints[0]
        target_position = Vector3(
            start_waypoint['x'],
            start_waypoint['y'],
            start_waypoint['z']
        )

        rospy.loginfo("=" * 60)
        rospy.loginfo("🏠 使用PID控制返回起点...")
        rospy.loginfo(f"  目标位置: ({target_position.x:.3f}, "
                      f"{target_position.y:.3f}, {target_position.z:.3f})")
        rospy.loginfo(f"  当前位置: ({self.current_position.x:.3f}, "
                      f"{self.current_position.y:.3f}, {self.current_position.z:.3f})")

        dx = target_position.x - self.current_position.x
        dy = target_position.y - self.current_position.y
        dz = target_position.z - self.current_position.z
        initial_distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        rospy.loginfo(f"  初始距离: {initial_distance:.3f} 米")
        rospy.loginfo(f"  位置阈值: {position_threshold} 米")
        rospy.loginfo(f"  超时时间: {timeout} 秒")
        rospy.loginfo("=" * 60)

        if initial_distance < position_threshold:
            rospy.loginfo("✓ 已在起点附近，无需移动")
            return True

        # 初始化PID控制器 - 🔥 降低参数以提高稳定性
        pid_x = PIDController(kp=0.8, ki=0.02, kd=0.5, output_limit=2.0)
        pid_y = PIDController(kp=0.8, ki=0.02, kd=0.5, output_limit=2.0)
        pid_z = PIDController(kp=1.0, ki=0.03, kd=0.6, output_limit=2.5)

        pid_vx = PIDController(kp=0.5, ki=0.01, kd=0.2, output_limit=1.0)
        pid_vy = PIDController(kp=0.5, ki=0.01, kd=0.2, output_limit=1.0)
        pid_vz = PIDController(kp=0.6, ki=0.015, kd=0.25, output_limit=1.2)

        rospy.loginfo("🚁 开始PID控制返回...")

        rate = rospy.Rate(50)
        start_time = rospy.Time.now()
        last_log_time = 0
        stable_counter = 0
        required_stable_count = 150  # 🔥 增加稳定性要求

        max_position_error = initial_distance
        min_position_error = initial_distance

        phase = "approaching"

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = (current_time - start_time).to_sec()

            if elapsed > timeout:
                current_distance = np.sqrt(
                    (target_position.x - self.current_position.x) ** 2 +
                    (target_position.y - self.current_position.y) ** 2 +
                    (target_position.z - self.current_position.z) ** 2
                )

                if current_distance < position_threshold * 2:
                    rospy.logwarn(f"⚠ 超时但位置可接受 (误差: {current_distance:.3f}m < {position_threshold * 2}m)")
                    rospy.loginfo("=" * 60)
                    rospy.loginfo(f"✓ 返回起点完成（超时但位置可接受）")
                    rospy.loginfo(f"  最终位置: ({self.current_position.x:.3f}, "
                                  f"{self.current_position.y:.3f}, "
                                  f"{self.current_position.z:.3f})")
                    rospy.loginfo(f"  位置误差: {current_distance:.4f} 米")
                    rospy.loginfo(f"  用时: {elapsed:.1f} 秒")
                    rospy.loginfo("=" * 60)
                    return True
                else:
                    rospy.logerr(f"✗ 返回起点超时且位置误差过大")
                    rospy.logerr(f"  当前距离起点: {current_distance:.3f} 米 (阈值: {position_threshold * 2}m)")
                    return False

            # 计算位置误差
            pos_error_x = target_position.x - self.current_position.x
            pos_error_y = target_position.y - self.current_position.y
            pos_error_z = target_position.z - self.current_position.z

            position_error = np.sqrt(pos_error_x ** 2 + pos_error_y ** 2 + pos_error_z ** 2)

            max_position_error = max(max_position_error, position_error)
            min_position_error = min(min_position_error, position_error)

            # 位置PID输出
            desired_vx = pid_x.compute(pos_error_x, current_time)
            desired_vy = pid_y.compute(pos_error_y, current_time)
            desired_vz = pid_z.compute(pos_error_z, current_time)

            # 速度误差
            vel_error_x = desired_vx - self.current_velocity.x
            vel_error_y = desired_vy - self.current_velocity.y
            vel_error_z = desired_vz - self.current_velocity.z

            # 速度PID输出
            ax = pid_vx.compute(vel_error_x, current_time)
            ay = pid_vy.compute(vel_error_y, current_time)
            az = pid_vz.compute(vel_error_z, current_time)

            # 构建控制命令
            control_velocity = Vector3(desired_vx, desired_vy, desired_vz)

            # 发布控制命令
            traj = MultiDOFJointTrajectory()
            traj.header.stamp = current_time
            traj.header.frame_id = "world"
            traj.joint_names = ["base_link"]

            point = MultiDOFJointTrajectoryPoint()

            transform = Transform()
            transform.translation = target_position
            transform.rotation = Quaternion(0, 0, 0, 1)
            point.transforms.append(transform)

            twist = Twist()
            twist.linear = control_velocity
            twist.angular = Vector3(0, 0, 0)
            point.velocities.append(twist)

            point.time_from_start = rospy.Duration(0.02)

            traj.points.append(point)
            self.traj_pub.publish(traj)

            # 阶段判断
            if position_error < position_threshold:
                if phase == "approaching":
                    phase = "stabilizing"
                    rospy.loginfo(f"  >> 进入稳定阶段（误差: {position_error:.4f}m）")
            else:
                if phase == "stabilizing":
                    phase = "approaching"
                    rospy.loginfo(f"  >> 返回接近阶段（误差: {position_error:.4f}m）")

            # 日志输出
            if int(elapsed) > last_log_time:
                speed = np.sqrt(
                    self.current_velocity.x ** 2 +
                    self.current_velocity.y ** 2 +
                    self.current_velocity.z ** 2
                )
                rospy.loginfo(
                    f"  [{phase:12s}] {elapsed:4.1f}s | "
                    f"误差: {position_error:.4f}m | "
                    f"速度: {speed:.2f}m/s | "
                    f"稳定: {stable_counter}/{required_stable_count}"
                )
                last_log_time = int(elapsed)

            # 到达判定
            if position_error < position_threshold:
                stable_counter += 1

                if stable_counter >= required_stable_count:
                    rospy.loginfo("=" * 60)
                    rospy.loginfo(f"✓ 成功返回起点并稳定！")
                    rospy.loginfo(f"  最终位置: ({self.current_position.x:.3f}, "
                                  f"{self.current_position.y:.3f}, "
                                  f"{self.current_position.z:.3f})")
                    rospy.loginfo(f"  位置误差: {position_error:.4f} 米")
                    rospy.loginfo(f"  用时: {elapsed:.1f} 秒")
                    rospy.loginfo(f"  最大误差: {max_position_error:.4f} 米")
                    rospy.loginfo(f"  最小误差: {min_position_error:.4f} 米")
                    rospy.loginfo("=" * 60)
                    return True
            else:
                stable_counter = 0

            rate.sleep()

        return False

    def measure_odom_frequency(self, duration=2.0):
        """测量实际的里程计频率"""
        rospy.loginfo("正在测量里程计频率...")
        self.odom_callback_count = 0
        self.odom_start_time = rospy.Time.now()

        rospy.sleep(duration)

        elapsed = (rospy.Time.now() - self.odom_start_time).to_sec()
        measured_frequency = self.odom_callback_count / elapsed if elapsed > 0 else 0

        self.odom_start_time = None

        rospy.loginfo(f"测量完成: 里程计频率 ≈ {measured_frequency:.1f} Hz")
        return measured_frequency

    def load_waypoints_from_csv(self, csv_file):
        """从CSV文件加载航点"""
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                waypoints = []
                for row in reader:
                    waypoint = {
                        'index': int(row['index']),
                        'x': float(row['x_m']),
                        'y': float(row['y_m']),
                        'z': float(row['z_m'])  # 🔥 添加 0.5 米高度偏移
                    }
                    waypoints.append(waypoint)

                self.waypoints = waypoints
                rospy.loginfo("=" * 60)
                rospy.loginfo(f"✓ 成功加载 {len(self.waypoints)} 个航点（已添加 0.5m 高度偏移）")
                rospy.loginfo(f"  起点: ({waypoints[0]['x']:.3f}, {waypoints[0]['y']:.3f}, {waypoints[0]['z']:.3f})")
                rospy.loginfo(f"  终点: ({waypoints[-1]['x']:.3f}, {waypoints[-1]['y']:.3f}, {waypoints[-1]['z']:.3f})")
                rospy.loginfo("=" * 60)

                # 🔥 初始化绘图器并设置目标轨迹
                self.plotter = RealtimePlotter(
                    robot_name="UAV",
                    max_points=None,
                    update_interval=100
                )

                # 设置目标轨迹（3D）
                trajectory_xyz = np.array([[wp['x'], wp['y'], wp['z']] for wp in waypoints])
                self.plotter.set_target_trajectory(trajectory_xyz)

                return True

        except FileNotFoundError:
            rospy.logerr(f"✗ 文件未找到: {csv_file}")
            return False
        except Exception as e:
            rospy.logerr(f"✗ 加载CSV文件失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def downsample_waypoints(self, target_count=None, min_distance=None):
        """航点降采样"""
        if len(self.waypoints) == 0:
            return

        original_count = len(self.waypoints)

        if target_count is not None and len(self.waypoints) > target_count:
            step = len(self.waypoints) / target_count
            selected_indices = [int(i * step) for i in range(target_count)]

            if selected_indices[-1] != len(self.waypoints) - 1:
                selected_indices[-1] = len(self.waypoints) - 1

            downsampled = [self.waypoints[i] for i in selected_indices]

        elif min_distance is not None:
            downsampled = [self.waypoints[0]]

            for i in range(1, len(self.waypoints)):
                last_wp = downsampled[-1]
                current_wp = self.waypoints[i]

                dx = current_wp['x'] - last_wp['x']
                dy = current_wp['y'] - last_wp['y']
                dz = current_wp['z'] - last_wp['z']
                distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                if distance >= min_distance:
                    downsampled.append(current_wp)

            if downsampled[-1]['index'] != self.waypoints[-1]['index']:
                downsampled.append(self.waypoints[-1])
        else:
            rospy.loginfo("未指定降采样参数，保持原始航点")
            return

        for i, wp in enumerate(downsampled):
            wp['index'] = i

        self.waypoints = downsampled

        # 🔥 更新绘图器的目标轨迹（3D）
        if self.plotter is not None:
            trajectory_xyz = np.array([[wp['x'], wp['y'], wp['z']] for wp in self.waypoints])
            self.plotter.set_target_trajectory(trajectory_xyz)

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✓ 航点降采样完成: {original_count} → {len(self.waypoints)} 个")
        rospy.loginfo("=" * 60)

    def calculate_segment_time(self, wp1, wp2, max_speed=5.0, min_time=0.3):
        """计算两个航点之间的飞行时间"""
        dx = wp2['x'] - wp1['x']
        dy = wp2['y'] - wp1['y']
        dz = wp2['z'] - wp1['z']
        distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        time = max(distance / max_speed, min_time)

        return time

    def create_global_trajectory(self, max_speed=5.0, min_segment_time=0.3):
        """创建全局轨迹消息"""
        if len(self.waypoints) == 0:
            rospy.logerr("没有可用的航点！")
            return None, 0.0

        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]

        cumulative_time = 0.0

        rospy.loginfo("=" * 60)
        rospy.loginfo("开始构建全局轨迹（高速模式）...")
        rospy.loginfo(f"最大速度: {max_speed} m/s")
        rospy.loginfo("=" * 60)

        for i, waypoint in enumerate(self.waypoints):
            point = MultiDOFJointTrajectoryPoint()

            transform = Transform()
            transform.translation.x = waypoint['x']
            transform.translation.y = waypoint['y']
            transform.translation.z = waypoint['z']+0.5
            transform.rotation = Quaternion(0, 0, 0, 1)
            point.transforms.append(transform)

            velocity = Twist()
            if i < len(self.waypoints) - 1:
                next_wp = self.waypoints[i + 1]

                dx = next_wp['x'] - waypoint['x']
                dy = next_wp['y'] - waypoint['y']
                dz = next_wp['z'] - waypoint['z']
                distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                segment_time = self.calculate_segment_time(
                    waypoint, next_wp, max_speed, min_segment_time
                )

                if distance > 0:
                    speed = distance / segment_time
                    velocity.linear.x = (dx / distance) * speed
                    velocity.linear.y = (dy / distance) * speed
                    velocity.linear.z = (dz / distance) * speed

                cumulative_time += segment_time
            else:
                velocity.linear = Vector3(0, 0, 0)

            velocity.angular = Vector3(0, 0, 0)
            point.velocities.append(velocity)

            point.time_from_start = rospy.Duration(cumulative_time)

            traj.points.append(point)

            if i % 10 == 0 or i == len(self.waypoints) - 1:
                rospy.loginfo(f"  航点 {i + 1}/{len(self.waypoints)}: "
                              f"位置({waypoint['x']:.2f}, {waypoint['y']:.2f}, {waypoint['z']:.2f}) "
                              f"时间: {cumulative_time:.1f}s")

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✓ 全局轨迹构建完成！")
        rospy.loginfo(f"  总航点数: {len(traj.points)}")
        rospy.loginfo(f"  预计飞行时间: {cumulative_time:.1f} 秒 ⚡")
        rospy.loginfo("=" * 60)

        return traj, cumulative_time

    def execute_global_trajectory(self, epoch, max_speed=5.0, min_segment_time=0.3):
        """执行全局轨迹跟踪"""
        if len(self.waypoints) == 0:
            rospy.logerr("没有可用的航点！")
            return False

        traj_msg, total_time = self.create_global_trajectory(max_speed, min_segment_time)

        if traj_msg is None:
            return False

        odom_frequency = self.measure_odom_frequency(duration=2.0)

        self.total_expected_callbacks = int(total_time * odom_frequency)
        self.target_records = self.max_records - 2
        self.record_counter = 0

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"数据记录设置:")
        rospy.loginfo(f"  里程计频率: {odom_frequency:.1f} Hz")
        rospy.loginfo(f"  预计飞行时间: {total_time:.1f} 秒")
        rospy.loginfo(f"  预期总回调次数: {self.total_expected_callbacks}")
        rospy.loginfo(f"  目标记录数（中间点）: {self.target_records}")
        rospy.loginfo(
            f"  平均采样间隔: 每 {self.total_expected_callbacks / float(self.target_records):.1f} 次回调记录1次")
        rospy.loginfo("=" * 60)

        if not self.first_waypoint_recorded:
            rospy.loginfo("记录第一个航点数据...")
            self.record_current_state()
            self.first_waypoint_recorded = True

        self.recording = True

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo(f"🚁 Epoch {epoch}: 开始执行全局轨迹跟踪（高速模式）...")
        rospy.loginfo("=" * 60)

        self.traj_pub.publish(traj_msg)

        rate = rospy.Rate(5)
        start_time = rospy.Time.now()
        last_progress = -1

        # 🔥 添加：标记是否已到达第一个航点并清空轨迹
        first_waypoint_reached = False
        first_waypoint_threshold = 0.5  # 距离阈值（米）

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            progress = min(int(elapsed / total_time * 100), 100)

            # 🔥 添加：检查是否到达第一个航点
            if not first_waypoint_reached and len(self.waypoints) > 0:
                first_wp = self.waypoints[0]
                dx = self.current_position.x - first_wp['x']
                dy = self.current_position.y - first_wp['y']
                dz = self.current_position.z - first_wp['z']
                distance_to_first = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                if distance_to_first < first_waypoint_threshold:
                    first_waypoint_reached = True
                    rospy.loginfo("✓ 已到达第一个航点，清空之前的轨迹数据")
                    if self.plotter is not None:
                        self.plotter.clear_trajectory()

            if progress // 10 > last_progress // 10:
                rospy.loginfo(f"飞行进度: {progress}% ({elapsed:.1f}s / {total_time:.1f}s) "
                              f"当前位置: ({self.current_position.x:.2f}, "
                              f"{self.current_position.y:.2f}, "
                              f"{self.current_position.z:.2f}) "
                              f"速度: ({self.current_velocity.x:.2f}, "
                              f"{self.current_velocity.y:.2f}, "
                              f"{self.current_velocity.z:.2f}) "
                              f"已记录: {len(self.position_x)}/{self.max_records}")
                last_progress = progress

            if elapsed >= total_time + 1.0:
                break

            rate.sleep()

        # 🔥 修改：先记录最后一个航点，再停止记录
        if not self.last_waypoint_recorded:
            rospy.loginfo("记录最后一个航点数据...")
            self.record_current_state()
            self.last_waypoint_recorded = True

        # 🔥 然后再停止记录
        self.recording = False

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo(f"✓ Epoch {epoch}: 全局轨迹跟踪完成！")
        rospy.loginfo(f"  实际飞行时间: {elapsed:.1f} 秒")
        rospy.loginfo(f"  最终位置: ({self.current_position.x:.2f}, "
                      f"{self.current_position.y:.2f}, "
                      f"{self.current_position.z:.2f})")
        rospy.loginfo(f"  最终速度: ({self.current_velocity.x:.2f}, "
                      f"{self.current_velocity.y:.2f}, "
                      f"{self.current_velocity.z:.2f})")
        rospy.loginfo(f"  记录数据点: {len(self.position_x)}")
        rospy.loginfo(f"  实际回调次数: {self.record_counter}")
        rospy.loginfo("=" * 60)

        return True

    def run_multiple_epochs(self, num_epochs=5, max_speed=5.0, min_segment_time=0.3,
                            wait_between_epochs=3.0, return_speed=2.0):
        """执行多次循环飞行"""
        rospy.loginfo("\n" + "🎯" * 30)
        rospy.loginfo(f"开始执行 {num_epochs} 次循环飞行")
        rospy.loginfo("🎯" * 30 + "\n")

        for epoch in range(1, num_epochs + 1):
            self.current_epoch = epoch

            # 🔥 更新绘图器的epoch
            if self.plotter is not None:
                self.plotter.set_epoch(epoch)

            rospy.loginfo("\n" + "📍" * 30)
            rospy.loginfo(f"Epoch {epoch}/{num_epochs}")
            rospy.loginfo("📍" * 30 + "\n")

            if epoch > 1:
                rospy.loginfo("🔄 准备开始新的循环，先返回起点...")
                success = self.return_to_start_with_pid()

                if not success:
                    rospy.logerr(f"✗ Epoch {epoch}: 返回起点失败！")
                    return False

                rospy.loginfo(f"⏳ 在起点等待 {wait_between_epochs} 秒，准备开始飞行...\n")
                rospy.sleep(wait_between_epochs)

            success = self.execute_global_trajectory(
                epoch=epoch,
                max_speed=max_speed,
                min_segment_time=min_segment_time
            )

            if not success:
                rospy.logerr(f"✗ Epoch {epoch} 执行失败！")
                return False

            self.ensure_exact_record_count()

            self.save_recorded_data()

            # 🔥 保存绘图
            if self.plotter is not None:
                self.plotter.save_figure(f'uav_trajectory_epoch_{epoch}.png')

            if epoch < num_epochs:
                rospy.loginfo(f"\n✓ Epoch {epoch} 完成，重置数据准备下一轮...\n")
                self.reset_for_next_epoch()
            else:
                rospy.loginfo(f"\n✓ Epoch {epoch} 完成（最后一轮）\n")

        rospy.loginfo("\n" + "🎉" * 30)
        rospy.loginfo(f"所有 {num_epochs} 次循环飞行完成！")
        rospy.loginfo("🎉" * 30 + "\n")

        # 🔥 关闭绘图器
        if self.plotter is not None:
            self.plotter.close()

        return True


def signal_handler(sig, frame):
    """处理 Ctrl+C 信号"""
    rospy.loginfo("\n收到中断信号，正在清理...")
    sys.exit(0)


def main():
    controller = None

    try:
        signal.signal(signal.SIGINT, signal_handler)

        controller = GlobalTrajectoryController()

        csv_file = 'waypoints.csv'

        if not controller.load_waypoints_from_csv(csv_file):
            rospy.logerr("无法加载航点文件，程序退出")
            return

        controller.downsample_waypoints(target_count=20)

        rospy.loginfo("等待系统初始化...")
        rospy.sleep(2.0)

        success = controller.run_multiple_epochs(
            num_epochs=101,
            max_speed=2.0,
            min_segment_time=0.3,
            wait_between_epochs=1.0
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
    finally:
        # 🔥 确保关闭绘图器
        if controller is not None and controller.plotter is not None:
            controller.plotter.close()


if __name__ == '__main__':
    main()
