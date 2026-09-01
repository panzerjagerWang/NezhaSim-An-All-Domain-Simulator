#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# ============================================================================
# 修改后的 UUVContorlV14_angle_video.py
# 核心改动:移除前瞻算法,改为顺序导航
# ============================================================================
import csv
import os

# Try to import the custom services
try:
    from nezha_plugins.srv import GetPhaseSample
    from nezha_plugins.srv import HydrodynamicsForces
except ImportError as e:
    print(f"⚠️ Warning: Could not import custom services: {e}")

def get_forces(service_proxy):
    """获取当前机器人的受力"""
    if service_proxy is None:
        return None
    try:
        response = service_proxy()
        return {
            'buoyancy_x': response.buoyancy_x, 'buoyancy_y': response.buoyancy_y, 'buoyancy_z': response.buoyancy_z,
            'damping_x': response.damping_x, 'damping_y': response.damping_y, 'damping_z': response.damping_z,
            'wave_x': response.wave_x, 'wave_y': response.wave_y, 'wave_z': response.wave_z,
            'added_mass_x': response.added_mass_x, 'added_mass_y': response.added_mass_y, 'added_mass_z': response.added_mass_z,
            'coriolis_x': response.coriolis_x, 'coriolis_y': response.coriolis_y, 'coriolis_z': response.coriolis_z
        }
    except Exception:
        return None

def initialize_wave_service():
    """初始化波浪高度服务"""
    try:
        rospy.wait_for_service('/nezha_mini/transmedia/get_phase_sample', timeout=2.0)
        service_proxy = rospy.ServiceProxy('/nezha_mini/transmedia/get_phase_sample', GetPhaseSample, persistent=True)
        print("✓ 波浪高度服务已连接 (High-Frequency Persistent Mode)")
        return service_proxy
    except Exception as e:
        print(f"⚠️ 波浪高度服务未找到: {e}")
        return None

def initialize_force_services():
    """初始化受力服务"""
    try:
        rospy.wait_for_service('/nezha_mini/get_hydrodynamics_forces', timeout=2.0)
        service_proxy = rospy.ServiceProxy('/nezha_mini/get_hydrodynamics_forces', HydrodynamicsForces, persistent=True)
        print("✓ UUV受力服务已连接 (High-Frequency Persistent Mode)")
        return service_proxy
    except Exception as e:
        print(f"⚠️ UUV受力服务未找到: {e}")
        return None

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from std_msgs.msg import Header
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

# ============================================================================
# 🎯 推进器物理特性(根据测试结果)
# ============================================================================
# 推进器推力      Pitch角度变化        物理效果        Gazebo中的表现
# 正推力 (+)     Pitch 减小(变负)    机头抬头 ⬆️     仰角增大
# 负推力 (-)     Pitch 增大(变正)    机头低头 ⬇️     俯角增大
#
# 控制逻辑:
# - 需要上浮(depth减小)→ 需要抬头 → 输出正推力
# - 需要下潜(depth增大)→ 需要低头 → 输出负推力
# - 机体有正浮力 → 需要持续负推力补偿 → 防止抬头
# ============================================================================

def quaternion_from_euler(roll, pitch, yaw):
    """
    将欧拉角(roll, pitch, yaw)转换为四元数(x, y, z, w)

    参数:
        roll: 横滚角(弧度)
        pitch: 俯仰角(弧度)
        yaw: 偏航角(弧度)

    返回:
        (qx, qy, qz, qw): 四元数
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


class RealtimePlotter:
    """实时X-Z轨迹绘制器 - 修复版"""

    def __init__(self, max_points=200000, update_interval=100):  # 50ms更新间隔
        self.max_points = max_points
        self.update_interval = update_interval
        self.x_data = []
        self.z_data = []
        # 数据缓冲
        self.x_data = deque(maxlen=max_points)
        self.z_data = deque(maxlen=max_points)
        self.target_x = None
        self.target_z = None
        self.current_epoch = 1

        # 线程控制
        self.lock = threading.Lock()
        self.running = True
        self.plot_thread = None

        # 初始化标志
        self.initialized = False

        print("🎨 启动绘图线程...")
        self.start_plotting_thread()

        # 等待初始化完成
        timeout = 5.0
        start_time = time.time()
        while not self.initialized and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if self.initialized:
            print("✓ 绘图线程已启动")
        else:
            print("⚠️ 绘图线程启动超时")

    def start_plotting_thread(self):
        """在独立线程中运行matplotlib"""
        self.plot_thread = threading.Thread(target=self._plot_loop, daemon=True)
        self.plot_thread.start()

    def _plot_loop(self):
        """绘图主循环(在独立线程中运行)"""
        try:
            # 🔥 在线程内部导入matplotlib
            import matplotlib
            matplotlib.use('TkAgg')
            import matplotlib.pyplot as plt
            from matplotlib.animation import FuncAnimation

            # 创建图形
            self.fig, self.ax = plt.subplots(figsize=(12, 8))
            self.setup_plot()

            # 初始化线条
            self.line_actual, = self.ax.plot(
                [], [], color='#2E86DE', linewidth=2.5,
                label='Actual Trajectory', zorder=3
            )
            self.line_target, = self.ax.plot(
                [], [], color='#EE5A6F', linewidth=2,
                linestyle='--', alpha=0.6,
                label='Target Trajectory', zorder=2
            )
            self.point_current, = self.ax.plot(
                [], [], 'o', color='#26DE81', markersize=12,
                markeredgecolor='white', markeredgewidth=2,
                label='Current Position', zorder=4
            )

            # 添加图例
            self.ax.legend(loc='upper right', fontsize=12,
                          framealpha=0.95, shadow=True)

            # 标记初始化完成
            self.initialized = True

            # 🔥 使用 FuncAnimation 自动更新
            self.ani = FuncAnimation(
                self.fig,
                self.update_plot,
                interval=self.update_interval,  # 50ms
                blit=False,  # 不使用blitting,更稳定
                cache_frame_data=False
            )

            plt.show()

        except Exception as e:
            print(f"❌ 绘图线程错误: {e}")
            import traceback
            traceback.print_exc()
            self.initialized = False

    def setup_plot(self):
        """设置图表样式"""
        self.ax.set_xlabel('X-direction (m)', fontsize=14, fontweight='bold')
        self.ax.set_ylabel('Z-direction (m)', fontsize=14, fontweight='bold')
        self.ax.set_title(
            f'Real-time UUV Trajectory (X-Z Plane) - Epoch {self.current_epoch}',
            fontsize=16, fontweight='bold', pad=20
        )
        self.ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
        self.ax.set_axisbelow(True)

        # 设置初始范围
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(-6, 0.5)

    def set_target_trajectory(self, trajectory):
        """设置目标轨迹"""
        with self.lock:
            self.target_x = trajectory[:, 0]- 0.2
            self.target_z = trajectory[:, 1]

    def add_point(self, x, z):
        """添加新数据点"""
        with self.lock:
            self.x_data.append(x)
            self.z_data.append(z)

            # 🔥 可选：如果设置了max_points，手动限制
            if self.max_points is not None:
                if len(self.x_data) > self.max_points:
                    self.x_data.pop(0)
                    self.z_data.pop(0)

    def set_epoch(self, epoch):
        """设置当前轮次"""
        with self.lock:
            self.current_epoch = epoch

    def clear_trajectory(self):
        """清空轨迹数据"""
        with self.lock:
            self.x_data.clear()
            self.z_data.clear()
        print("✓ 轨迹数据已清空")

    def update_plot(self, frame):
        """更新图表 - 简化版"""
        with self.lock:
            # 更新标题
            self.ax.set_title(
                f'Real-time UUV Trajectory (X-Z Plane) - Epoch {self.current_epoch}',
                fontsize=16, fontweight='bold', pad=20
            )

            # 更新目标轨迹
            if self.target_x is not None:
                self.line_target.set_data(self.target_x, self.target_z)

            # 更新实际轨迹
            if len(self.x_data) > 0:
                self.line_actual.set_data(self.x_data, self.z_data)  # 🔥 直接使用列表
                self.point_current.set_data([self.x_data[-1]], [self.z_data[-1]])

            # 动态调整坐标轴范围
            if len(self.x_data) > 1 or self.target_x is not None:
                x_vals = []
                z_vals = []

                if len(self.x_data) > 0:
                    x_vals.extend(self.x_data)
                    z_vals.extend(self.z_data)

                if self.target_x is not None:
                    x_vals.extend(self.target_x)
                    z_vals.extend(self.target_z)

                if x_vals and z_vals:
                    x_min, x_max = min(x_vals), max(x_vals)
                    z_min, z_max = min(z_vals), max(z_vals)

                    x_margin = max((x_max - x_min) * 0.1, 1.0)
                    z_margin = max((z_max - z_min) * 0.1, 0.5)

                    self.ax.set_xlim(x_min - x_margin, x_max + x_margin)
                    self.ax.set_ylim(z_min - z_margin, z_max + z_margin)

        return self.line_actual, self.line_target, self.point_current

    def save_figure(self, filename):
        """保存图表"""
        try:
            with self.lock:
                self.fig.savefig(filename, dpi=300, bbox_inches='tight')
                print(f"✓ 图表已保存: {filename}")
        except Exception as e:
            print(f"⚠️ 保存图表失败: {e}")

    def close(self):
        """关闭绘图窗口"""
        try:
            self.running = False
            if self.plot_thread is not None:
                self.plot_thread.join(timeout=2.0)

            import matplotlib.pyplot as plt
            plt.close(self.fig)
            print("✓ 绘图窗口已关闭")
        except Exception as e:
            print(f"⚠️ 关闭图表失败: {e}")



class UUVTrajectoryTracker:
    def __init__(self, trajectory_file,enable_realtime_plot=True):
        rospy.init_node("uuv_trajectory_tracker")

        # ========== 🔥 先初始化所有属性,避免回调函数访问未定义的属性 ==========
        self.current_pose = None
        self.prev_thrust_left = 0.0
        self.prev_thrust_right = 0.0
        self.prev_thrust_pitch = 0.0
        self.smoothing_factor = 0.75  # 🔥 More smoothing = less twitchy movement
        self.enable_buoyancy_compensation = True  # 🔥 确保启用
        # ============================================================================
        # 🎈 浮力补偿参数
        # ============================================================================
        self.buoyancy_compensation = -8.0
        if self.enable_buoyancy_compensation:
            buoyancy_comp = self.buoyancy_compensation
        else:
            buoyancy_comp = 0.0
        # ============================================================================

        # 轨迹跟踪参数
        self.current_waypoint_index = 0
        # 🔥 关键修改1: 减小容差,确保精确到达每个点
        self.waypoint_tolerance = 1.2  # Changed from 0.4 to 1.2
        # 🔥 关键修改2: 移除前瞻距离(后面会删除前瞻算法)
        # self.lookahead_distance = 4.0  # 不再使用
        self.fixed_y = 0.0
        self.current_epoch = 0

        # 加载轨迹(必须在订阅之前)
        self.trajectory = self.load_trajectory(trajectory_file)
        # ====================================================================
        self.enable_realtime_plot = enable_realtime_plot
        if self.enable_realtime_plot:
            self.plotter = RealtimePlotter(
                max_points=None,  # 🔥 设为None表示不限制
                update_interval=50
            )
            # 设置目标轨迹
            if self.trajectory is not None:
                self.plotter.set_target_trajectory(self.trajectory)
            print("✓ 实时轨迹绘制器已启动")
        else:
            self.plotter = None
        # 现在可以安全地订阅话题了
        rospy.Subscriber("/nezha_mini/ground_truth/pose", Pose,
                         self._pose_callback, queue_size=1)

        self.pub_pitch = rospy.Publisher("/nezha_mini/thrusters/0/input",
                                         FloatStamped, queue_size=10)
        self.pub_left = rospy.Publisher("/nezha_mini/thrusters/1/input",
                                        FloatStamped, queue_size=10)
        self.pub_right = rospy.Publisher("/nezha_mini/thrusters/2/input",
                                         FloatStamped, queue_size=10)

        from UUVControllerV8 import PIDController

        # 🚀 PID增益(简化版)
        self.pitch_pid = PIDController(
            kp=80.0,   # ⬆️ INCREASED: Faster response to pitch errors (was 50.0)
            ki=2.0,
            kd=80.0,   # ⬇️ DECREASED: Less damping so it moves faster (was 120.0)
            output_min=-100.0, output_max=100.0, # ⬆️ INCREASED: Allow full thruster power (was 50.0)
            windup_limit=20.0
        )

        self.position_pid = PIDController(
            kp=35.0, ki=2.0, kd=15.0,          # 🔥 INCREASED: Push harder forward
            output_min=-60.0, output_max=60.0, # 🔥 INCREASED: Allow much higher forward thrust (was 30.0)
            windup_limit=20.0
        )
        # 🆕 Roll PID控制器
        self.roll_pid = PIDController(
            kp=30.0,  # 比例增益
            ki=3.0,  # 积分增益
            kd=15.0,  # 微分增益
            output_min=-20.0,
            output_max=20.0,
            windup_limit=10.0
        )

        # 🔥 增强的Yaw PID控制器
        self.yaw_pid = PIDController(
            kp=30.0,  # 从25.0增加
            ki=3.0,  # 从2.0增加
            kd=12.0,  # 从10.0增加
            output_min=-20.0,  # 从-15.0增加
            output_max=20.0,  # 从15.0增加
            windup_limit=10.0  # 从8.0增加
        )
        self.depth_pid = PIDController(
            kp=55.0,  # 🔥 Increased: push harder toward target depth
            ki=4.0,  # 🔥 Slightly increased: eliminate steady-state depth error
            kd=45.0,
            output_min=-35.0, output_max=35.0,  # 🔥 Wider range
            windup_limit=15.0
        )

        self.trajectory = self.load_trajectory(trajectory_file)

        # 等待Gazebo服务
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.wave_service = initialize_wave_service()
        self.force_service = initialize_force_services()

        self.csv_filename = None
        self.global_step = 0
        self.fieldnames = [
            'episode', 'step', 'time', 'reward',
            'target_x', 'target_z', 'current_x', 'current_z',
            'depth_u', 'pitch_u', # PID outputs
            'motor_1', 'motor_2', 'motor_3', 'motor_4',
            'roll_deg', 'pitch_deg', 'yaw_deg',
            'altitude', 'water_surface_height',
            'buoyancy_x', 'buoyancy_y', 'buoyancy_z',
            'damping_x', 'damping_y', 'damping_z',
            'wave_x', 'wave_y', 'wave_z',
            'added_mass_x', 'added_mass_y', 'added_mass_z',
            'coriolis_x', 'coriolis_y', 'coriolis_z'
        ]

    def init_csv_log(self, epoch):
        """为每一轮(epoch)创建一个新的CSV日志文件"""
        # Storing data into a specific file for PID control
        self.csv_filename = f"uuv_pid_log_{epoch}.csv"
        self.global_step = 0
        with open(self.csv_filename, mode='w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames)
            writer.writeheader()
        print(f"✓ CSV日志记录已初始化: {self.csv_filename}")
    def set_thrusters_smooth(self, T_pitch, T_left, T_right):
        """带指数平滑的推进器控制 - 消除抖动"""
        # 限幅
        T_pitch = max(min(T_pitch, 100.0), -100.0)
        T_r = max(min(T_right, 80.0), -80.0)
        T_l = max(min(T_left, 80.0), -80.0)

        # 🚀 指数移动平均平滑
        alpha = self.smoothing_factor
        T_pitch_smooth = alpha * self.prev_thrust_pitch + (1 - alpha) * T_pitch
        T_l_smooth = alpha * self.prev_thrust_left + (1 - alpha) * T_l
        T_r_smooth = alpha * self.prev_thrust_right + (1 - alpha) * T_r

        # 更新历史值
        self.prev_thrust_pitch = T_pitch_smooth
        self.prev_thrust_left = T_l_smooth
        self.prev_thrust_right = T_r_smooth

        # 发布
        now = rospy.Time.now()
        self.pub_pitch.publish(FloatStamped(Header(stamp=now), T_pitch_smooth))
        self.pub_right.publish(FloatStamped(Header(stamp=now), T_r_smooth))
        self.pub_left.publish(FloatStamped(Header(stamp=now), T_l_smooth))

    def reset_model_to_origin(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """
        重置Gazebo中的UUV模型到指定位置和姿态
        """
        try:
            print(f"\n{'─' * 70}")
            print(f"🔄 重置Gazebo模型到原点...")
            print(f"{'─' * 70}")

            # 创建模型状态消息
            model_state = ModelState()
            model_state.model_name = 'nezha_mini'

            # 设置位置
            model_state.pose.position = Point(x, y, z)

            # 🔥 使用tf库转换欧拉角到四元数
            import tf
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            model_state.pose.orientation = Quaternion(
                quaternion[0],  # x
                quaternion[1],  # y
                quaternion[2],  # z
                quaternion[3]  # w
            )

            # 设置速度为零
            model_state.twist = Twist()

            # 设置参考坐标系
            model_state.reference_frame = 'world'

            # 调用服务
            response = self.set_model_state(model_state)

            if response.success:
                print(f"✅ 模型位置已重置到: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                print(f"✅ 模型姿态已重置为: Roll={math.degrees(roll):.2f}°, "
                      f"Pitch={math.degrees(pitch):.2f}°, Yaw={math.degrees(yaw):.2f}°")

                # 等待物理引擎稳定
                print(f"⏳ 等待物理引擎稳定...")
                rospy.sleep(3.0)

                # 重置PID控制器
                self.position_pid.reset()
                self.yaw_pid.reset()
                self.pitch_pid.reset()
                self.depth_pid.reset()
                self.roll_pid.reset()
                print(f"✅ PID控制器已重置")

                # 验证重置
                rospy.sleep(0.5)
                if self.current_pose is not None:
                    current_roll, current_pitch, current_yaw = self.get_current_angles()
                    print(f"📊 当前姿态: Roll={math.degrees(current_roll):.2f}°, "
                          f"Pitch={math.degrees(current_pitch):.2f}°, "
                          f"Yaw={math.degrees(current_yaw):.2f}°")

                return True
            else:
                print(f"❌ 模型重置失败: {response.status_message}")
                return False

        except Exception as e:
            print(f"❌ 重置过程发生错误: {e}")
            import traceback
            traceback.print_exc()
            return False

    def load_trajectory(self, filename):
        """加载轨迹文件(x,z格式) - 使用pandas，并下采样"""
        try:
            import pandas as pd

            # pandas会自动处理表头
            df = pd.read_csv(filename)

            # 检查列名
            if 'x' in df.columns and 'z' in df.columns:
                data = df[['x', 'z']].values
            elif df.shape[1] == 2:
                # 如果没有列名,使用前两列
                data = df.iloc[:, :2].values
            else:
                print(f"❌ 无法识别CSV格式")
                return None

            # 🔥 关键修改：下采样，只取1/4的点
            sampling_rate = 8  # 每4个点取1个
            data = data[::sampling_rate]

            print(f"✓ 成功加载轨迹文件: {filename}")
            print(f"  原始轨迹点数: {len(df)}")  # 显示原始点数
            print(f"  下采样后点数: {len(data)} (采样率: 1/{sampling_rate})")
            print(f"  X范围: [{data[:, 0].min():.2f}, {data[:, 0].max():.2f}]")
            print(f"  Z范围: [{data[:, 1].min():.2f}, {data[:, 1].max():.2f}]")

            return data

        except ImportError:
            print("⚠️ pandas未安装,使用numpy加载...")
            # 回退到numpy方法
            return self._load_trajectory_numpy(filename)
        except Exception as e:
            print(f"❌ 加载轨迹文件失败: {e}")
            import traceback
            traceback.print_exc()
            return None

    def _load_trajectory_numpy(self, filename):
        """使用numpy加载(备用方法)，并下采样"""
        try:
            data = np.loadtxt(filename, delimiter=',', skiprows=1)

            # 🔥 下采样
            sampling_rate = 4
            original_count = len(data)
            data = data[::sampling_rate]

            print(f"✓ 使用numpy加载成功")
            print(f"  原始轨迹点数: {original_count}")
            print(f"  下采样后点数: {len(data)} (采样率: 1/{sampling_rate})")

            return data
        except:
            # 如果跳过1行失败,尝试不跳过
            try:
                data = np.loadtxt(filename, delimiter=',')

                # 🔥 下采样
                sampling_rate = 4
                original_count = len(data)
                data = data[::sampling_rate]

                print(f"✓ 使用numpy加载成功(无表头)")
                print(f"  原始轨迹点数: {original_count}")
                print(f"  下采样后点数: {len(data)} (采样率: 1/{sampling_rate})")

                return data
            except Exception as e:
                print(f"❌ numpy加载失败: {e}")
                return None

    def _pose_callback(self, pose):
        """位姿回调函数"""
        self.current_pose = pose

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

    def stop_thrusters(self):
        self.set_thrusters_smooth(0, 0, 0)

    def wait_for_stable(self, timeout=2.0):
        time.sleep(timeout)

    def calculate_distance_xz(self, x1, z1, x2, z2):
        """计算x-z平面距离"""
        return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

    # 🔥 关键修改3: 完全移除前瞻算法
    # def find_lookahead_waypoint(self):
    #     """
    #     前瞻算法:找到前方lookahead_distance距离处的目标点(x-z平面)
    #     """
    #     # 此函数已删除

    def navigate_to_waypoint(self, target_x, target_z, target_y=None,
                             distance_tolerance=0.8, timeout=60.0):
        """
        导航到指定waypoint - 🔥 增加前向检测，防止回头
        """
        if target_y is None:
            target_y = self.fixed_y

        # 等待位姿数据
        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # 🔥 新增：初始前向检测
        current_x, current_y, current_z = self.get_current_position()
        roll, pitch, yaw = self.get_current_angles()

        dx = target_x - current_x
        dy = target_y - current_y

        # 计算目标相对于当前朝向的角度
        target_yaw = math.atan2(dy, dx)
        yaw_diff = math.atan2(math.sin(target_yaw - yaw),
                              math.cos(target_yaw - yaw))

        # 🔥 如果目标在后方（超过90度），直接跳过
        if abs(yaw_diff) > math.radians(90):
            print(f"  ⚠️ 航点在后方 (角度差:{math.degrees(yaw_diff):.1f}°)，跳过")
            return False

        # 重置所有PID控制器
        self.position_pid.reset()
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.depth_pid.reset()

        if not hasattr(self, 'roll_pid'):
            from UUVControllerV8 import PIDController
            self.roll_pid = PIDController(
                kp=30.0, ki=3.0, kd=15.0,
                output_min=-20.0, output_max=20.0,
                windup_limit=10.0
            )
        self.roll_pid.reset()

        rate = rospy.Rate(100)
        start_time = time.time()

        # 容差定义
        # 容差定义
        DEPTH_TOLERANCE = 0.5  # 改为 0.5 (原本是 0.3，你的日志里dz是0.37所以卡住了)
        ROLL_TOLERANCE = math.radians(2.0)
        YAW_TOLERANCE = math.radians(3.0)

        # 状态变量
        prev_pitch = None
        prev_time = time.time()
        prev_depth = None
        last_print_time = time.time()
        PRINT_INTERVAL = 2.0

        target_pitch_final = 0.0
        pitch_correction_deg = 0.0
        target_pitch_geometry = 0.0

        # 🔥 新增：连续后方检测计数器
        backward_count = 0
        MAX_BACKWARD_COUNT = 5 # 连续10次检测到后方就放弃

        while not rospy.is_shutdown():
            current_time = time.time()

            # ============================================================
            # ⏱️ 超时检查
            # ============================================================
            if current_time - start_time > timeout:
                print("  ✗ 导航超时!")
                return False

            # ============================================================
            # 📍 获取当前状态
            # ============================================================
            current_x, current_y, current_z = self.get_current_position()
            roll, pitch, yaw = self.get_current_angles()

            # 计算误差
            dx = target_x - current_x
            dy = target_y - current_y
            dz = target_z - current_z

            distance_xz = math.sqrt(dx ** 2 + dz ** 2)
            distance_z = abs(dz)

            dt = max(current_time - prev_time, 1e-3)

            # 计算深度速度
            if prev_depth is not None:
                depth_velocity = (current_z - prev_depth) / dt
            else:
                depth_velocity = 0.0
            prev_depth = current_z

            # ============================================================
            # 🎯 到达判断(位置 + 姿态)
            # ============================================================
            # 计算目标Yaw
            if abs(dx) > 0.1 or abs(dy) > 0.1:
                target_yaw = math.atan2(dy, dx)
            else:
                target_yaw = yaw

            yaw_error = math.atan2(math.sin(target_yaw - yaw),
                                   math.cos(target_yaw - yaw))

            # 🔥 新增：实时检测目标是否在后方
            if abs(yaw_error) > math.radians(90):
                backward_count += 1
                if backward_count > MAX_BACKWARD_COUNT:
                    print(f"  ⚠️ 航点持续在后方 (角度差:{math.degrees(yaw_error):.1f}°)，放弃")
                    return False
            else:
                backward_count = 0  # 重置计数器

            # 位置检查
            position_ok = (distance_xz < distance_tolerance and
                           distance_z < DEPTH_TOLERANCE)

            # 姿态检查
            attitude_ok = (abs(roll) < ROLL_TOLERANCE and
                           abs(yaw_error) < YAW_TOLERANCE)

            if position_ok and attitude_ok:
                if current_time - last_print_time > 0.5:
                    print(f"  ✓ 到达waypoint! X-Z:{distance_xz:.2f}m, Z误差:{dz:.2f}m")
                    print(f"    姿态: Roll={math.degrees(roll):+5.1f}°, "
                          f"Yaw误差={math.degrees(yaw_error):+5.1f}°")
                break

            # 计算Pitch速度
            if prev_pitch is not None:
                pitch_velocity = (pitch - prev_pitch) / dt
            else:
                pitch_velocity = 0.0
            prev_pitch = pitch
            prev_time = current_time

            # ============================================================
            # 🎯 1. 外环:深度控制(Pitch目标生成)
            # ============================================================
            raw_pid_output = self.depth_pid.update(current_z, target_z, current_time)

            # ⬆️ INCREASED: Allow depth error to demand a stronger pitch angle
            if abs(dz) > 1.0:
                MAX_DEPTH_CORRECTION = 30.0  # Was 20.0
            elif abs(dz) > 0.5:
                MAX_DEPTH_CORRECTION = 20.0  # Was 15.0
            elif abs(dz) > 0.2:
                MAX_DEPTH_CORRECTION = 12.0  # Was 8.0
            else:
                MAX_DEPTH_CORRECTION = 5.0   # Was 3.0

            pitch_correction_deg = max(min(raw_pid_output, MAX_DEPTH_CORRECTION),
                                       -MAX_DEPTH_CORRECTION)

            # 死区
            if abs(dz) < 0.05:
                pitch_correction_deg = 0.0

            pitch_correction_rad = math.radians(pitch_correction_deg)

            # ============================================================
            # 🎯 2. 几何前馈
            # ============================================================
            horizontal_base = max(abs(dx), 0.1)
            target_pitch_geometry_raw = -math.atan2(dz, horizontal_base)

            # TO THIS:
            if distance_xz > 3.0:
                distance_weight = 0.6  # 🔥 Reduced: stop geometry from dominating
            elif distance_xz > 1.5:
                if abs(dz) > 0.5:
                    distance_weight = 0.4
                else:
                    distance_weight = 0.5
            else:
                distance_weight = 0.3

            target_pitch_geometry = target_pitch_geometry_raw * distance_weight

            # ============================================================
            # 🎯 3. 计算总目标Pitch
            # ============================================================
            target_pitch_final = target_pitch_geometry + pitch_correction_rad

            # ⬆️ INCREASED: Allow steeper angles for faster depth changes
            if abs(dz) > 1.5:
                GLOBAL_MAX_PITCH = math.radians(45.0)  # Was 35.0
            elif abs(dz) > 0.8:
                GLOBAL_MAX_PITCH = math.radians(35.0)  # Was 28.0
            elif abs(dz) > 0.3:
                GLOBAL_MAX_PITCH = math.radians(25.0)  # Was 20.0
            else:
                GLOBAL_MAX_PITCH = math.radians(15.0)  # Was 10.0

            target_pitch_final = max(min(target_pitch_final, GLOBAL_MAX_PITCH),
                                     -GLOBAL_MAX_PITCH)

            # ============================================================
            # 🎯 4. 内环:Pitch姿态控制
            # ============================================================
            pid_output = self.pitch_pid.update(target_pitch_final, pitch, current_time)
            pitch_thrust = -pid_output

            # 浮力补偿
            # 浮力补偿
            buoyancy_comp = self.buoyancy_compensation \
                if self.enable_buoyancy_compensation else 0.0

            pitch_output = pitch_thrust + buoyancy_comp

            # ⬆️ INCREASED: Change 50.0 to 100.0 to match your thruster max
            pitch_output = max(min(pitch_output, 100.0), -100.0)

            # ============================================================
            # 🆕 5. Roll控制
            # ============================================================
            TARGET_ROLL = 0.0

            roll_correction = self.roll_pid.update(TARGET_ROLL, roll, current_time)

            roll_deg = math.degrees(abs(roll))
            if roll_deg > 10.0:
                roll_correction *= 1.5
            elif roll_deg < 2.0:
                roll_correction *= 0.5

            if roll_deg < 1.0:
                roll_correction *= 0.3

            # ============================================================
            # 🆕 6. Yaw控制(使用PID)
            # ============================================================
            yaw_correction = self.yaw_pid.update(target_yaw, yaw, current_time)

            yaw_error_deg = abs(math.degrees(yaw_error))
            if yaw_error_deg > 30.0:
                yaw_correction *= 1.3
            elif yaw_error_deg < 5.0:
                yaw_correction *= 0.6

            if yaw_error_deg < 2.0:
                yaw_correction *= 0.3

            # ============================================================
            # 🎯 7. 水平推力(距离自适应)
            # ============================================================
            thrust_raw = self.position_pid.update(0.0, distance_xz, current_time)

            # 🔥 INCREASED: 距离自适应推力范围大幅提升
            if distance_xz > 5.0:
                min_thrust = 20.0  # Was 8.0
                max_thrust = 60.0  # Was 30.0
            elif distance_xz > 2.0:
                min_thrust = 15.0  # Was 5.0
                max_thrust = 50.0  # Was 25.0
            else:
                min_thrust = 12.0  # Was 8.0
                max_thrust = 40.0  # Was 25.0

            thrust = max(min(abs(thrust_raw), max_thrust), min_thrust)

            # ============================================================
            # 🎯 8. 差分推力分配(Roll优先策略)
            # ============================================================
            if distance_xz < 2.0:
                roll_weight = 0.7
                yaw_weight = 0.3
            elif distance_xz < 5.0:
                roll_weight = 0.5
                yaw_weight = 0.5
            else:
                roll_weight = 0.4
                yaw_weight = 0.6

            differential = (roll_correction * roll_weight +
                            yaw_correction * yaw_weight)

            MAX_DIFFERENTIAL = 25.0  # 🔥 INCREASED: Allow faster turning (was 15.0)
            differential = max(min(differential, MAX_DIFFERENTIAL),
                               -MAX_DIFFERENTIAL)

            thrust_left = thrust - differential
            thrust_right = thrust + differential

            # 🔥 INCREASED: Allow motors to spin much faster (was 40.0)
            thrust_left = max(min(thrust_left, 80.0), -80.0)
            thrust_right = max(min(thrust_right, 80.0), -80.0)

            # ============================================================
            # 📊 增强的打印信息
            # ============================================================
            if current_time - last_print_time > PRINT_INTERVAL:
                depth_direction = "⬆️上浮" if dz < 0 else "⬇️下潜" if dz > 0 else "➡️平行"

                print(f"[{self.current_waypoint_index + 1}/{len(self.trajectory)}] "
                      f"X:{current_x:+6.2f}→{target_x:+6.2f}({dx:+5.2f}) | "
                      f"Z:{current_z:+6.2f}→{target_z:+6.2f}({dz:+5.2f}) {depth_direction} | "
                      f"Dist:{distance_xz:5.2f}m")

                print(f"  Pitch:{math.degrees(pitch):+5.1f}°→{math.degrees(target_pitch_final):+5.1f}° "
                      f"(geo{math.degrees(target_pitch_geometry):+4.1f}° + corr{pitch_correction_deg:+4.1f}°) | "
                      f"Thrust:{pitch_output:+5.1f}(PID{pitch_thrust:+5.1f}+Buoy{buoyancy_comp:+5.1f})")

                print(f"  🎯 Roll:{math.degrees(roll):+5.1f}°→0.0° (corr:{roll_correction:+5.1f}, "
                      f"weight:{roll_weight:.1f}) | "
                      f"Yaw:{math.degrees(yaw):+5.1f}°→{math.degrees(target_yaw):+5.1f}° "
                      f"(err:{math.degrees(yaw_error):+5.1f}°, corr:{yaw_correction:+5.1f}, "
                      f"weight:{yaw_weight:.1f})")

                print(f"  ⚙️ Horiz: Base={thrust:+5.1f}, Diff={differential:+5.1f} → "
                      f"L={thrust_left:+5.1f}, R={thrust_right:+5.1f}\n")

                last_print_time = current_time

            # 🚀 发送控制指令
            # ============================================================
            self.set_thrusters_smooth(
                pitch_output,
                thrust_left,
                thrust_right
            )
            if self.plotter is not None:
                self.plotter.add_point(current_x, current_z)

            # ============================================================
            # NEW: 📝 DATA LOGGING TO CSV
            # ============================================================
            # 1. Get Hydrodynamic Forces
            forces = get_forces(self.force_service)
            if forces is None:
                # Fallback to zeros if service fails/isn't running
                forces = {k: 0.0 for k in self.fieldnames if k.endswith('_x') or k.endswith('_y') or k.endswith('_z')}
                forces.pop('current_x', None)
                forces.pop('current_z', None)
                forces.pop('target_x', None)
                forces.pop('target_z', None)

            # 2. Get Water Surface Height (Wave)
            water_surface_height = 0.0
            if self.wave_service is not None:
                try:
                    wave_resp = self.wave_service(current_x, current_y)
                    water_surface_height = wave_resp.phase
                except Exception:
                    pass

            # 3. Calculate dummy reward (e.g., negative distance to target)
            reward = -distance_xz

            # 4. Compile row data
            row_data = {
                'episode': self.current_epoch,
                'step': self.global_step,
                'time': current_time,
                'reward': reward,
                'target_x': target_x, 'target_z': target_z,
                'current_x': current_x, 'current_z': current_z,

                # PID Outputs
                'depth_u': raw_pid_output,
                'pitch_u': pid_output,

                # Motors & Attitude
                'motor_1': pitch_output,
                'motor_2': thrust_left,
                'motor_3': thrust_right,
                'motor_4': 0.0,  # Assuming unused
                'roll_deg': math.degrees(roll),
                'pitch_deg': math.degrees(pitch),
                'yaw_deg': math.degrees(yaw),
                'altitude': current_z,
                'water_surface_height': water_surface_height
            }
            # Merge forces into row_data
            row_data.update(forces)

            # 5. Write to CSV
            if self.csv_filename is not None:
                with open(self.csv_filename, mode='a', newline='') as f:
                    writer = csv.DictWriter(f, fieldnames=self.fieldnames)
                    writer.writerow(row_data)

            self.global_step += 1
            # ============================================================

            rate.sleep()

        return True

    def track_trajectory_with_lookahead(self):
        """
        轨迹跟踪主函数 - 🔥 修改为顺序导航版本
        """
        if self.trajectory is None or len(self.trajectory) == 0:
            print("轨迹为空,无法跟踪!")
            return False

        print(f"\n{'#' * 60}")
        print(f"# 开始轨迹跟踪(顺序导航版)")
        print(f"{'#' * 60}\n")

        self.current_waypoint_index = 0
        start_time = time.time()

        while not rospy.is_shutdown():
            # 检查是否完成轨迹
            if self.current_waypoint_index >= len(self.trajectory):
                print(f"\n{'=' * 60}")
                print(f"✓✓✓ 轨迹跟踪完成!")
                print(f"总用时: {time.time() - start_time:.1f}秒")
                print(f"{'=' * 60}\n")
                break

            # 🔥 关键修改4: 直接使用当前索引的航点,不使用前瞻
            target_x, target_z = self.trajectory[self.current_waypoint_index]

            print(f"\n{'─' * 60}")
            print(f"📍 Waypoint {self.current_waypoint_index + 1}/{len(self.trajectory)}")
            print(f"{'─' * 60}")
            print(f"目标: X={target_x:.2f}, Z={target_z:.2f}, Y={self.fixed_y:.2f}")

            # 导航到目标点
            success = self.navigate_to_waypoint(
                target_x, target_z,
                target_y=self.fixed_y,
                distance_tolerance=self.waypoint_tolerance,  # 🔥 Now uses the 1.2m tolerance
                timeout=30.0
            )

            if success:
                # 🔥 关键修改5: 成功后只前进1个航点(严格按顺序)
                self.current_waypoint_index += 1
                print(f"✓ 航点 {self.current_waypoint_index} 完成,前进到下一个")
            else:
                print(f"✗ 导航失败,尝试下一个航点")
                self.current_waypoint_index += 1

        self.stop_thrusters()
        return True

    def reset_for_next_epoch(self):
        """每轮结束后的重置"""
        self.current_waypoint_index = 0
        self.prev_thrust_left = 0.0
        self.prev_thrust_right = 0.0
        self.prev_thrust_pitch = 0.0

        # PID清零
        self.position_pid.reset()
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.depth_pid.reset()
        self.roll_pid.reset()


if __name__ == "__main__":
    try:
        ORIGIN_X = 0.0
        ORIGIN_Y = 0.0
        ORIGIN_Z = 0.0
        NUM_REPEATS = 100

        print(f"\n{'#' * 70}")
        print(f"# 开始 {NUM_REPEATS} 次轨迹跟踪循环(顺序导航版)")
        print(f"{'#' * 70}\n")

        # 创建tracker实例
        tracker = UUVTrajectoryTracker(
            "trajectory.csv",
            enable_realtime_plot=True
        )
        reset_success = tracker.reset_model_to_origin(x=ORIGIN_X, y=ORIGIN_Y, z=ORIGIN_Z)

        # 🔥 等待绘图窗口完全初始化
        if tracker.plotter is not None:
            print("⏳ 等待绘图窗口初始化...")
            time.sleep(2.0)
            print("✓ 绘图窗口已就绪\n")

        for repeat_count in range(1, NUM_REPEATS + 1):
            print(f"\n{'=' * 70}")
            print(f"  🔄 第 {repeat_count}/{NUM_REPEATS} 次循环")
            print(f"{'=' * 70}\n")

            tracker.current_epoch = repeat_count

            # 更新图表标题
            if tracker.plotter is not None:
                tracker.plotter.set_epoch(repeat_count)

            # ============================================================
            # NEW: Initialize the specific CSV file for this run
            # ============================================================
            tracker.init_csv_log(repeat_count)

            # 执行轨迹跟踪
            success = tracker.track_trajectory_with_lookahead()

            if success:
                print(f"\n✅ 第 {repeat_count} 次轨迹跟踪完成")
            else:
                print(f"\n⚠️ 第 {repeat_count} 次轨迹跟踪失败")

            # 保存当前轮次的图表
            if tracker.plotter is not None:
                tracker.plotter.save_figure(
                    f'trajectory_epoch_{repeat_count}.png'
                )

            # 如果不是最后一次循环,重置
            if repeat_count < NUM_REPEATS:
                tracker.stop_thrusters()
                tracker.wait_for_stable(1.0)

                reset_success = tracker.reset_model_to_origin(
                    x=ORIGIN_X, y=ORIGIN_Y, z=ORIGIN_Z
                )

                if not reset_success:
                    print(f"⚠️ 模型重置失败")

                tracker.wait_for_stable(2.0)
                tracker.reset_for_next_epoch()

                # 🔥 清空轨迹数据
                if tracker.plotter is not None:
                    tracker.plotter.clear_trajectory()

                print(f"\n⏳ 准备第 {repeat_count + 1} 次循环...\n")


        # 最后关闭
        tracker.stop_thrusters()
        if tracker.plotter is not None:
            tracker.plotter.save_figure('trajectory_final.png')
            print("\n📊 按 Ctrl+C 关闭程序和图表窗口")
            input("或按回车键继续...")
            tracker.plotter.close()

    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
        if 'tracker' in locals() and tracker.plotter is not None:
            tracker.plotter.close()
    except Exception as e:
        import traceback

        print(f"\n❌ 发生错误: {e}")
        traceback.print_exc()
