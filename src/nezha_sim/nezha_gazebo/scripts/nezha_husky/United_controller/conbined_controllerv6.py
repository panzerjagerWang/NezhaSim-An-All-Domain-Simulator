#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
三机器人协同轨迹跟踪系统
支持: UGV (地面) -> UUV (水下) -> UAV (空中)
改进:
1. 基于x方向行进距离的进度追踪
2. 整合UUV_CONTROLV15的顺序导航和实时绘图
3. 新增UAV控制器（基于flight_controllerv2）
4. 支持smoothed_trajectory_data.csv的路径点跟踪
"""
from mav_msgs.msg import Actuators

import rospy
import math
import time
import numpy as np
import threading
from enum import Enum
from collections import deque

# ROS消息类型
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Transform, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from uuv_gazebo_ros_plugins_msgs.srv import GetModelProperties
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nezha_plugins.srv import GetPhaseSample, HydrodynamicsForces
# Matplotlib配置
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cv2  # 新增
class RobotType(Enum):
    UGV = "UGV"
    UUV = "UUV"
    UAV = "UAV"


class DistanceTracker:
    """路程追踪器 - 用于基于距离的数据采样"""

    def __init__(self, total_distance, num_samples):
        """
        初始化路程追踪器

        参数:
            total_distance: 总路程（米）
            num_samples: 目标采样点数
        """
        self.total_distance = total_distance
        self.num_samples = num_samples
        self.sample_interval = total_distance / num_samples if num_samples > 0 else 1.0

        # 状态变量
        self.accumulated_distance = 0.0
        self.next_sample_distance = self.sample_interval
        self.sample_count = 0

        # 上一次位置（用于计算增量距离）
        self.last_x = None
        self.last_y = None
        self.last_z = None

        print(f"📏 路程追踪器初始化:")
        print(f"   总路程: {total_distance:.2f}m")
        print(f"   目标采样数: {num_samples}")
        print(f"   采样间隔: {self.sample_interval:.3f}m")

    def update(self, x, y, z):
        """
        更新当前位置，返回是否应该采样

        参数:
            x, y, z: 当前位置

        返回:
            should_sample: 是否应该采样
        """
        if self.last_x is None:
            # 第一次调用，初始化位置
            self.last_x, self.last_y, self.last_z = x, y, z
            return True  # 记录起点

        # 计算距离上次位置的增量距离
        dx = x - self.last_x
        dy = y - self.last_y
        dz = z - self.last_z
        increment = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # 累积距离
        self.accumulated_distance += increment

        # 更新上次位置
        self.last_x, self.last_y, self.last_z = x, y, z

        # 判断是否达到采样点
        if self.accumulated_distance >= self.next_sample_distance:
            self.next_sample_distance += self.sample_interval
            self.sample_count += 1
            return True

        return False

    def get_progress(self):
        """获取进度百分比"""
        if self.total_distance == 0:
            return 0.0
        return min(100.0, (self.accumulated_distance / self.total_distance) * 100.0)

    def reset(self):
        """重置追踪器"""
        self.accumulated_distance = 0.0
        self.next_sample_distance = self.sample_interval
        self.sample_count = 0
        self.last_x = None
        self.last_y = None
        self.last_z = None


class GlobalDataRecorder:
    """全局数据记录器 - 记录所有机器人的轨迹和受力数据"""

    def __init__(self):
        # 位置数据
        self.position_x = []
        self.position_y = []
        self.position_z = []

        # 速度数据
        self.velocity_x = []
        self.velocity_y = []
        self.velocity_z = []

        # 姿态数据
        self.roll_data = []
        self.pitch_data = []
        self.yaw_data = []

        # 受力数据
        self.buoyancy_x = []
        self.buoyancy_y = []
        self.buoyancy_z = []

        self.damping_x = []
        self.damping_y = []
        self.damping_z = []
        self.surface_z = []
        self.z_over_l = []
        self.phase_name = []
        self.wave_x = []
        self.wave_y = []
        self.wave_z = []

        self.added_mass_x = []
        self.added_mass_y = []
        self.added_mass_z = []

        self.coriolis_x = []
        self.coriolis_y = []
        self.coriolis_z = []

        # 机器人类型标记
        self.robot_type = []

        # 时间戳
        self.timestamps = []

        # 服务代理（用于获取UUV受力）
        self.get_forces_service = None
        self.get_phase_service = None  # 🔥 新增

        print("✓ 全局数据记录器已初始化")

    def initialize_force_services(self):
        """初始化受力服务和波浪面服务"""
        # 受力服务初始化...
        try:
            rospy.wait_for_service('/nezha_husky1/get_hydrodynamics_forces', timeout=5.0)
            from nezha_plugins.srv import HydrodynamicsForces
            self.get_forces_service = rospy.ServiceProxy(
                '/nezha_husky1/get_hydrodynamics_forces',
                HydrodynamicsForces
            )
            print("✓ UUV受力服务已连接")
        except:
            print("⚠️ UUV受力服务未找到")

        # 🔥 波浪面服务初始化
        try:
            rospy.wait_for_service('/nezha_husky1/transmedia/get_phase_sample', timeout=5.0)
            from nezha_plugins.srv import GetPhaseSample
            self.get_phase_service = rospy.ServiceProxy(
                '/nezha_husky1/transmedia/get_phase_sample',
                GetPhaseSample
            )
            print("✓ 波浪面服务已连接")
        except:
            print("⚠️ 波浪面服务未找到")

    def get_wave_surface(self):
        """🔥 获取当前波浪面高度"""
        if self.get_phase_service is None:
            return None

        try:
            response = self.get_phase_service()
            return {
                'surface_z': response.surface_z,
                'z_over_l': response.z_over_l,
                'phase_name': response.phase_name
            }
        except Exception as e:
            return None
    def record_data_point(self, robot_type_str, position, velocity, attitude, forces=None):
        """
        记录单个数据点

        参数:
            robot_type_str: 机器人类型 ('UGV', 'UUV', 'UAV')
            position: (x, y, z) 位置元组
            velocity: (vx, vy, vz) 速度元组
            attitude: (roll, pitch, yaw) 姿态元组
            forces: 受力字典（仅UUV需要）
        """
        # 位置
        self.position_x.append(position[0])
        self.position_y.append(position[1])
        self.position_z.append(position[2])

        # 速度
        self.velocity_x.append(velocity[0])
        self.velocity_y.append(velocity[1])
        self.velocity_z.append(velocity[2])

        # 姿态
        self.roll_data.append(math.degrees(attitude[0]))
        self.pitch_data.append(math.degrees(attitude[1]))
        self.yaw_data.append(math.degrees(attitude[2]))
        wave_surface = self.get_wave_surface()
        if wave_surface is not None:
            self.surface_z.append(wave_surface['surface_z'])
            self.z_over_l.append(wave_surface['z_over_l'])
            self.phase_name.append(wave_surface['phase_name'])
        else:
            self.surface_z.append(0.0)
            self.z_over_l.append(0.0)
            self.phase_name.append('UNKNOWN')
        # 受力（如果有）
        if forces is not None:
            self.buoyancy_x.append(forces.get('buoyancy_x', 0.0))
            self.buoyancy_y.append(forces.get('buoyancy_y', 0.0))
            self.buoyancy_z.append(forces.get('buoyancy_z', 0.0))

            self.damping_x.append(forces.get('damping_x', 0.0))
            self.damping_y.append(forces.get('damping_y', 0.0))
            self.damping_z.append(forces.get('damping_z', 0.0))

            self.wave_x.append(forces.get('wave_x', 0.0))
            self.wave_y.append(forces.get('wave_y', 0.0))
            self.wave_z.append(forces.get('wave_z', 0.0))

            self.added_mass_x.append(forces.get('added_mass_x', 0.0))
            self.added_mass_y.append(forces.get('added_mass_y', 0.0))
            self.added_mass_z.append(forces.get('added_mass_z', 0.0))

            self.coriolis_x.append(forces.get('coriolis_x', 0.0))
            self.coriolis_y.append(forces.get('coriolis_y', 0.0))
            self.coriolis_z.append(forces.get('coriolis_z', 0.0))
        else:
            # 填充默认值（UGV和UAV没有这些力）
            for attr in ['buoyancy_x', 'buoyancy_y', 'buoyancy_z',
                         'damping_x', 'damping_y', 'damping_z',
                         'wave_x', 'wave_y', 'wave_z',
                         'added_mass_x', 'added_mass_y', 'added_mass_z',
                         'coriolis_x', 'coriolis_y', 'coriolis_z']:
                getattr(self, attr).append(0.0)

        # 机器人类型
        self.robot_type.append(robot_type_str)

        # 时间戳
        self.timestamps.append(rospy.Time.now().to_sec())

    def get_forces(self):
        """获取当前机器人的受力（通用方法）"""
        if self.get_forces_service is None:
            return None

        try:
            response = self.get_forces_service()
            return {
                'buoyancy_x': response.buoyancy_x,
                'buoyancy_y': response.buoyancy_y,
                'buoyancy_z': response.buoyancy_z,
                'damping_x': response.damping_x,
                'damping_y': response.damping_y,
                'damping_z': response.damping_z,
                'wave_x': response.wave_x,
                'wave_y': response.wave_y,
                'wave_z': response.wave_z,
                'added_mass_x': response.added_mass_x,
                'added_mass_y': response.added_mass_y,
                'added_mass_z': response.added_mass_z,
                'coriolis_x': response.coriolis_x,
                'coriolis_y': response.coriolis_y,
                'coriolis_z': response.coriolis_z
            }
        except Exception as e:
            print(f"⚠️ 获取受力失败: {e}")
            return None

    def save_to_csv(self, filename='trajectory_data_with_forces.csv'):
        """保存数据到CSV文件（包含波浪面数据）"""
        if len(self.position_x) == 0:
            print("⚠️ 没有数据可保存")
            return False

        try:
            import pandas as pd

            # 构建DataFrame
            data = {
                'timestamp': self.timestamps,
                'x': self.position_x,
                'y': self.position_y,
                'z': self.position_z,
                'vx': self.velocity_x,
                'vy': self.velocity_y,
                'vz': self.velocity_z,
                'roll': self.roll_data,
                'pitch': self.pitch_data,
                'yaw': self.yaw_data,
                'buoyancy_x': self.buoyancy_x,
                'buoyancy_y': self.buoyancy_y,
                'buoyancy_z': self.buoyancy_z,
                'damping_x': self.damping_x,
                'damping_y': self.damping_y,
                'damping_z': self.damping_z,
                'wave_x': self.wave_x,
                'wave_y': self.wave_y,
                'wave_z': self.wave_z,
                'added_mass_x': self.added_mass_x,
                'added_mass_y': self.added_mass_y,
                'added_mass_z': self.added_mass_z,
                'coriolis_x': self.coriolis_x,
                'coriolis_y': self.coriolis_y,
                'coriolis_z': self.coriolis_z,
                # 🔥 新增：波浪面数据列
                'surface_z': self.surface_z,
                'z_over_l': self.z_over_l,
                'phase_name': self.phase_name,
                'robot_type': self.robot_type
            }

            df = pd.DataFrame(data)
            df.to_csv(filename, index=False)

            print(f"\n{'=' * 70}")
            print(f"✅ 数据已保存到: {filename}")
            print(f"   总数据点数: {len(df)}")
            print(f"   UGV数据点: {len(df[df['robot_type'] == 'UGV'])}")
            print(f"   UUV数据点: {len(df[df['robot_type'] == 'UUV'])}")
            print(f"   UAV数据点: {len(df[df['robot_type'] == 'UAV'])}")
            print(f"   🌊 波浪面数据列: surface_z, z_over_l, phase_name")
            print(f"{'=' * 70}\n")

            return True

        except Exception as e:
            print(f"❌ 保存CSV失败: {e}")
            import traceback
            traceback.print_exc()
            return False

    def clear(self):
        """清空所有数据"""
        self.position_x.clear()
        self.position_y.clear()
        self.position_z.clear()
        self.velocity_x.clear()
        self.velocity_y.clear()
        self.velocity_z.clear()
        self.roll_data.clear()
        self.pitch_data.clear()
        self.yaw_data.clear()
        self.buoyancy_x.clear()
        self.buoyancy_y.clear()
        self.buoyancy_z.clear()
        self.damping_x.clear()
        self.damping_y.clear()
        self.damping_z.clear()
        self.wave_x.clear()
        self.wave_y.clear()
        self.wave_z.clear()
        self.added_mass_x.clear()
        self.added_mass_y.clear()
        self.added_mass_z.clear()
        self.coriolis_x.clear()
        self.coriolis_y.clear()
        self.coriolis_z.clear()
        self.robot_type.clear()
        self.timestamps.clear()
        self.surface_z.clear()
        self.z_over_l.clear()
        self.phase_name.clear()

class AdaptiveSensorVisualizer:
    """自适应传感器可视化器"""

    def __init__(self):
        self.current_robot_type = None
        self.enabled = False

        # 传感器数据
        self.latest_laser = None
        self.latest_sonar = None
        self.latest_image = None
        self.latest_points = None

        # 订阅器（初始为None）
        self.laser_sub = None
        self.sonar_sub = None
        self.image_sub = None
        self.points_sub = None  # 🔥 点云订阅器

        print("✓ 传感器可视化器已初始化")

    def switch_to_robot(self, robot_type):
        """切换到指定机器人类型"""
        if self.current_robot_type == robot_type:
            return

        # 关闭旧窗口
        self._close_all_windows()

        # 取消旧订阅
        self._unsubscribe_all()

        self.current_robot_type = robot_type

        # 订阅新传感器
        if robot_type == RobotType.UGV:
            print("📡 启动UGV点云可视化")  # 🔥 修改提示
            # 🔥 修改：订阅点云而非激光雷达
            self.points_sub = rospy.Subscriber(
                '/nezha_husky1/points',
                PointCloud2,
                self._points_callback,
                queue_size=1
            )
            self.enabled = True

        elif robot_type == RobotType.UUV:
            print("🌊 启动UUV声呐可视化")
            self.sonar_sub = rospy.Subscriber(
                '/nezha_husky1/sonar',
                LaserScan,
                self._sonar_callback,
                queue_size=1
            )
            self.enabled = True

        elif robot_type == RobotType.UAV:
            print("📷 启动UAV相机可视化")
            self.image_sub = rospy.Subscriber(
                '/nezha_husky1/camera_/image_raw',
                Image,
                self._image_callback,
                queue_size=1
            )
            self.enabled = True

    def _points_callback(self, msg):
        """🔥 处理点云数据（参考sensor.py）"""
        try:
            points_list = []
            for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
                points_list.append([point[0], point[1], point[2]])

            if len(points_list) > 0:
                self.latest_points = np.array(points_list)
                # 可视化点云
                img = self._visualize_pointcloud()
                if img is not None:
                    cv2.imshow('UGV Point Cloud', img)
                    cv2.waitKey(1)
        except Exception as e:
            print(f"⚠️ 点云处理错误: {e}")

    def _sonar_callback(self, msg):
        """处理声呐数据"""
        self.latest_sonar = msg
        img = self._visualize_laserscan(msg, "UUV Sonar")
        if img is not None:
            cv2.imshow('UUV Sonar', img)
            cv2.waitKey(1)

    def _image_callback(self, msg):
        """处理图像数据"""
        try:
            cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1
            )
            if cv_image.shape[2] == 1:
                cv_image = cv_image.squeeze()
            cv2.imshow('UAV Camera', cv_image)
            cv2.waitKey(1)
        except Exception as e:
            print(f"⚠️ 图像处理错误: {e}")

    def _visualize_pointcloud(self):
        """🔥 可视化点云（俯视图）- 从sensor.py移植"""
        if self.latest_points is None or len(self.latest_points) == 0:
            return None

        # 创建一个空白图像
        img_size = 600
        img = np.zeros((img_size, img_size, 3), dtype=np.uint8)

        points = self.latest_points
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # 归一化坐标到图像空间
        if len(x) > 0:
            scale = 50
            center = img_size // 2

            px = (x * scale + center).astype(int)
            py = (-y * scale + center).astype(int)

            # 根据z值着色
            z_min, z_max = z.min(), z.max()
            if z_max > z_min:
                z_normalized = (z - z_min) / (z_max - z_min)
            else:
                z_normalized = np.zeros_like(z)

            # 绘制点
            valid_mask = (px >= 0) & (px < img_size) & (py >= 0) & (py < img_size)
            for i in np.where(valid_mask)[0]:
                color = int(z_normalized[i] * 255)
                cv2.circle(img, (px[i], py[i]), 1, (color, 255 - color, 128), -1)

        # 绘制中心十字
        center = img_size // 2
        cv2.line(img, (center - 20, center), (center + 20, center), (0, 255, 0), 1)
        cv2.line(img, (center, center - 20), (center, center + 20), (0, 255, 0), 1)

        # 添加文字信息
        cv2.putText(img, "UGV Point Cloud", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, f"Points: {len(self.latest_points)}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return img

    def _visualize_laserscan(self, scan, title):
        """可视化LaserScan（用于UUV声呐）"""
        img_size = 600
        img = np.zeros((img_size, img_size, 3), dtype=np.uint8)
        center = (img_size // 2, img_size // 2)

        max_range = scan.range_max
        scale = (img_size // 2 - 50) / max_range if max_range > 0 else 1

        # 绘制范围圆
        cv2.circle(img, center, int(max_range * scale), (50, 50, 50), 1)

        # 绘制距离圈（每米一圈）
        for r in range(1, int(max_range) + 1):
            radius = int(r * scale)
            cv2.circle(img, center, radius, (30, 30, 30), 1)

        # 绘制激光点
        angle = scan.angle_min
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max:
                x = int(center[0] + r * scale * np.cos(angle))
                y = int(center[1] + r * scale * np.sin(angle))

                color_ratio = (r - scan.range_min) / (scan.range_max - scan.range_min)
                color = (0, int(255 * color_ratio), int(255 * (1 - color_ratio)))

                cv2.circle(img, (x, y), 2, color, -1)
            angle += scan.angle_increment

        # 中心点和方向
        cv2.circle(img, center, 5, (255, 255, 255), -1)
        cv2.arrowedLine(img, center, (center[0] + 40, center[1]),
                        (0, 255, 0), 2, tipLength=0.3)

        # 信息文字
        cv2.putText(img, title, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        return img

    def _unsubscribe_all(self):
        """取消所有订阅"""
        if self.laser_sub:
            self.laser_sub.unregister()
            self.laser_sub = None
        if self.sonar_sub:
            self.sonar_sub.unregister()
            self.sonar_sub = None
        if self.image_sub:
            self.image_sub.unregister()
            self.image_sub = None
        if self.points_sub:  # 🔥 添加点云订阅器注销
            self.points_sub.unregister()
            self.points_sub = None

    def _close_all_windows(self):
        """关闭所有OpenCV窗口"""
        cv2.destroyAllWindows()

    def close(self):
        """关闭可视化器"""
        self._unsubscribe_all()
        self._close_all_windows()
        self.enabled = False
        print("✓ 传感器可视化器已关闭")

class BaseRobotController:
    """机器人控制器基类"""

    # 定义每种机器人的轨迹段配置
    TRAJECTORY_CONFIG = {
        RobotType.UGV: {
            'row_range': (0, 22),  # 修改：第0-25行（Python索引0-25，共26行）
            'segment_name': '地面段',
            'description': 'UGV地面行驶段'
        },
        RobotType.UUV: {
            'row_range': (23, 52),  # 修改：第26-54行
            'segment_name': '水下段',
            'description': 'UUV水下航行段'
        },
        RobotType.UAV: {
            'row_range': (53, None),  # 修改：第55行起
            'segment_name': '空中段',
            'description': 'UAV空中飞行段'
        }
    }

    def calculate_total_path_length(self):
        """
        计算路径总长度

        返回:
            total_length: 总路程（米）
        """
        if len(self.waypoints) < 2:
            return 0.0

        total_length = 0.0
        for i in range(len(self.waypoints) - 1):
            current = self.waypoints[i]
            next_wp = self.waypoints[i + 1]

            dx = next_wp[0] - current[0]
            dy = next_wp[1] - current[1]
            dz = next_wp[2] - current[2]
            segment_length = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            total_length += segment_length

        return total_length
    def load_trajectory(self):
        """统一的轨迹加载方法（修复Y坐标读取）"""
        try:
            import pandas as pd
            df = pd.read_csv(self.waypoint_file)

            # 获取当前机器人的配置
            config = self.TRAJECTORY_CONFIG.get(self.robot_type)
            if config is None:
                print(f"❌ 未知机器人类型: {self.robot_type}")
                return

            # 按行号切片
            start_row, end_row = config['row_range']
            if end_row is None:
                filtered_df = df.iloc[start_row:]
            else:
                filtered_df = df.iloc[start_row:end_row]

            if len(filtered_df) == 0:
                print(f"⚠️ {self.robot_type.value}: 没有数据")
                return

            # 🔥 修复：读取x, y, z三列
            # 检查CSV是否包含y列
            if 'y' in filtered_df.columns:
                data = filtered_df[['x', 'y', 'z']].values
                self.waypoints = [(x, y, z) for x, y, z in data]
                print(f"✓ 读取完整的3D路径点 (x, y, z)")
            else:
                # 如果CSV没有y列，使用默认值0.0
                data = filtered_df[['x', 'z']].values
                self.waypoints = [(x, 0.0, z) for x, z in data]
                print(f"⚠️ CSV缺少y列，使用默认值y=0.0")

            # 🔥 修复：trajectory也应该包含y坐标（用于绘图）
            # 但绘图器只需要x-z平面，所以保持原样
            self.trajectory = filtered_df[['x', 'z']].values

            # 计算总距离
            if len(self.waypoints) >= 2:
                self.start_x = self.waypoints[0][0]
                self.target_x = self.waypoints[-1][0]
                self.total_x_distance = abs(self.target_x - self.start_x)

            # 打印信息
            print(f"✓ 成功加载 {len(self.waypoints)} 个{self.robot_type.value}路径点 ({config['segment_name']})")
            print(f"  行号范围: {start_row + 1}-{filtered_df.index[-1] + 1}")
            print(f"  {config['description']}")

            # 🔥 新增：显示Y坐标范围
            if 'y' in filtered_df.columns:
                y_values = filtered_df['y'].values
                print(f"  实际X: {data[:, 0].min():.2f} → {data[:, 0].max():.2f}")
                print(f"  实际Y: {y_values.min():.2f} → {y_values.max():.2f}")
                print(f"  实际Z: {filtered_df['z'].min():.2f} → {filtered_df['z'].max():.2f}")
            else:
                print(f"  实际X: {data[:, 0].min():.2f} → {data[:, 0].max():.2f}")
                print(f"  实际Z: {data[:, 1].min():.2f} → {data[:, 1].max():.2f}")

            # 显示前3个点
            for i in range(min(3, len(self.waypoints))):
                x, y, z = self.waypoints[i]
                print(f"    [{i + 1}] X={x:.2f}, Y={y:.2f}, Z={z:.2f}")

        except Exception as e:
            print(f"❌ 加载轨迹失败: {e}")
            import traceback
            traceback.print_exc()


# ==================== 工具函数 ====================

def euler_from_quaternion(q):
    """四元数转欧拉角"""
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

class PIDController:
    """通用PID控制器"""

    def __init__(self, kp, ki, kd, output_min=-float('inf'), output_max=float('inf'),
                 windup_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.windup_limit = windup_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, setpoint, measured_value, current_time):
        """更新PID输出"""
        error = setpoint - measured_value

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            return 0.0

        dt = (current_time - self.prev_time) if isinstance(current_time, float) else \
            (current_time - self.prev_time).to_sec()

        if dt <= 0:
            return 0.0

        # 比例项
        p_term = self.kp * error

        # 积分项（带抗饱和）
        self.integral += error * dt
        if self.windup_limit is not None:
            self.integral = np.clip(self.integral, -self.windup_limit, self.windup_limit)
        i_term = self.ki * self.integral

        # 微分项
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # 总输出
        output = p_term + i_term + d_term
        output = np.clip(output, self.output_min, self.output_max)

        self.prev_error = error
        self.prev_time = current_time

        return output
class RealtimePlotter:
    """实时X-Z轨迹绘制器（通用）"""

    def __init__(self, robot_name="Robot", max_points=None, update_interval=50):
        self.robot_name = robot_name
        self.max_points = max_points
        self.update_interval = update_interval

        # 数据缓冲
        self.x_data = deque(maxlen=max_points) if max_points else []
        self.z_data = deque(maxlen=max_points) if max_points else []
        self.target_x = None
        self.target_z = None
        self.current_epoch = 1

        # 线程控制
        self.lock = threading.Lock()
        self.running = True
        self.plot_thread = None
        self.initialized = False

        print(f"🎨 启动{robot_name}绘图线程...")
        self.start_plotting_thread()

        # 等待初始化完成
        timeout = 5.0
        start_time = time.time()
        while not self.initialized and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        if self.initialized:
            print(f"✓ {robot_name}绘图线程已启动")
        else:
            print(f"⚠️ {robot_name}绘图线程启动超时")

    def start_plotting_thread(self):
        """在独立线程中运行matplotlib"""
        self.plot_thread = threading.Thread(target=self._plot_loop, daemon=True)
        self.plot_thread.start()

    def _plot_loop(self):
        """绘图主循环"""
        try:
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

            self.ax.legend(loc='lower left', fontsize=12,
                           framealpha=0.95, shadow=True)

            self.initialized = True

            # 使用 FuncAnimation 自动更新
            self.ani = FuncAnimation(
                self.fig,
                self.update_plot,
                interval=self.update_interval,
                blit=False,
                cache_frame_data=False
            )

            plt.show()

        except Exception as e:
            print(f"❌ {self.robot_name}绘图线程错误: {e}")
            import traceback
            traceback.print_exc()
            self.initialized = False

    def setup_plot(self):
        """设置图表样式"""
        self.ax.set_xlabel('X-direction (m)', fontsize=10, fontweight='bold')
        self.ax.set_ylabel('Z-direction (m)', fontsize=10, fontweight='bold')
        self.ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.8)
        self.ax.set_axisbelow(True)
        self.ax.set_xlim(-2, 15)
        self.ax.set_ylim(-6, 2)

    def set_target_trajectory(self, trajectory):
        """设置目标轨迹"""
        with self.lock:
            self.target_x = trajectory[:, 0]
            self.target_z = trajectory[:, 1]

    def add_point(self, x, z):
        """添加新数据点"""
        with self.lock:
            if isinstance(self.x_data, deque):
                self.x_data.append(x)
                self.z_data.append(z)
            else:
                self.x_data.append(x)
                self.z_data.append(z)
                if self.max_points and len(self.x_data) > self.max_points:
                    self.x_data.pop(0)
                    self.z_data.pop(0)

    def set_epoch(self, epoch):
        """设置当前轮次"""
        with self.lock:
            self.current_epoch = epoch

    def clear_trajectory(self):
        """清空轨迹数据"""
        with self.lock:
            if isinstance(self.x_data, deque):
                self.x_data.clear()
                self.z_data.clear()
            else:
                self.x_data = []
                self.z_data = []
        print(f"✓ {self.robot_name}轨迹数据已清空")

    def update_plot(self, frame):
        """更新图表"""
        with self.lock:

            # 更新目标轨迹
            if self.target_x is not None:
                self.line_target.set_data(self.target_x, self.target_z)

            # 更新实际轨迹
            if len(self.x_data) > 0:
                self.line_actual.set_data(list(self.x_data), list(self.z_data))
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
            plt.close(self.fig)
            print(f"✓ {self.robot_name}绘图窗口已关闭")
        except Exception as e:
            print(f"⚠️ 关闭图表失败: {e}")

class UAVController(BaseRobotController):
    """无人机控制器 - 基于 flight_controller_huskyv2.py 的轨迹跟踪逻辑"""

    def __init__(self, waypoint_file='smoothed_trajectory_data.csv', enable_realtime_plot=True):
        self.robot_type = RobotType.UAV
        self.waypoint_file = waypoint_file
        self.waypoints = []
        self.trajectory = None
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self._waypoints_interpolated = False

        # 当前状态
        self.current_position = Vector3(0, 0, 0)
        self.current_velocity = Vector3(0, 0, 0)

        # 控制参数
        self.cruise_speed = 1.5  # 巡航速度 (m/s)
        self.accel = 0.1  # 加速度 (m/s²)
        self.height_offset = 0.6  # 高度偏移 (m)

        # 基于x方向距离的进度追踪
        self.start_x = None
        self.target_x = None
        self.total_x_distance = 0.0
        self.initial_x = None

        # 加载轨迹
        self.load_trajectory()

        # 实时绘图器
        self.enable_realtime_plot = enable_realtime_plot
        if self.enable_realtime_plot:
            self.plotter = RealtimePlotter(
                robot_name="UAV",
                max_points=None,
                update_interval=50
            )
            print("✓ UAV实时轨迹绘制器已启动")
        else:
            self.plotter = None

        # 发布器：发布轨迹命令
        self.traj_pub = rospy.Publisher(
            '/nezha_husky1/command/trajectory',
            MultiDOFJointTrajectory,
            queue_size=10
        )

        # 订阅器：获取当前位置信息
        rospy.Subscriber(
            '/nezha_husky1/ground_truth/odometry',
            Odometry,
            self._odom_callback,
            queue_size=1
        )

        # 设置目标轨迹到绘图器
        if self.plotter is not None and self.trajectory is not None:
            self.plotter.set_target_trajectory(self.trajectory)

        print("✓ UAV控制器初始化完成")

    def _odom_callback(self, msg):
        """里程计回调"""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear

        # 🔥 添加姿态获取
        q = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        self.current_roll, self.current_pitch, self.current_yaw = euler_from_quaternion(q)

        if self.initial_x is None:
            self.initial_x = self.current_position.x

        # 更新实时绘图
        if self.plotter is not None:
            self.plotter.add_point(self.current_position.x, self.current_position.z)

    def interpolate_waypoints(self, max_segment_length=0.5):
        """插值航点，使航点间距更小更均匀"""
        if len(self.waypoints) < 2:
            return

        interpolated = [self.waypoints[0]]

        for i in range(len(self.waypoints) - 1):
            current = self.waypoints[i]
            next_wp = self.waypoints[i + 1]

            dx = next_wp[0] - current[0]
            dy = next_wp[1] - current[1]
            dz = next_wp[2] - current[2]
            distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            if distance > max_segment_length:
                # 需要插值
                num_segments = int(np.ceil(distance / max_segment_length))

                for j in range(1, num_segments):
                    t = j / float(num_segments)
                    interpolated_wp = (
                        current[0] + t * dx,
                        current[1] + t * dy,
                        current[2] + t * dz
                    )
                    interpolated.append(interpolated_wp)

            interpolated.append(next_wp)

        original_count = len(self.waypoints)
        self.waypoints = interpolated

        print("=" * 60)
        print(f"✓ 航点插值完成: {original_count} → {len(self.waypoints)} 个")
        print(f"  最大段长: {max_segment_length} 米")
        print("=" * 60)

    def create_trajectory_with_smooth_velocity(self):
        """
        创建带速度平滑的轨迹

        返回:
            traj_msg: MultiDOFJointTrajectory消息
            total_time: 预计飞行时间
        """
        if len(self.waypoints) == 0:
            rospy.logerr("没有可用的航点！")
            return None, 0.0
        if not self._waypoints_interpolated:
            self.interpolate_waypoints(max_segment_length=0.5)
            self._waypoints_interpolated = True
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]

        cumulative_time = 0.0
        current_speed = 0.0  # 初始速度为0

        print("=" * 60)
        print("开始构建平滑轨迹...")
        print(f"巡航速度: {self.cruise_speed} m/s")
        print(f"加速度: {self.accel} m/s²")
        print(f"高度偏移: {self.height_offset} m")
        print("=" * 60)

        for i, waypoint in enumerate(self.waypoints):
            point = MultiDOFJointTrajectoryPoint()

            # 设置位置
            transform = Transform()
            transform.translation.x = waypoint[0]
            transform.translation.y = waypoint[1]
            transform.translation.z = waypoint[2] + self.height_offset
            transform.rotation = Quaternion(0, 0, 0, 1)
            point.transforms.append(transform)

            # 计算速度
            velocity = Twist()

            if i < len(self.waypoints) - 1:
                next_wp = self.waypoints[i + 1]

                # 计算方向和距离
                dx = next_wp[0] - waypoint[0]
                dy = next_wp[1] - waypoint[1]
                dz = next_wp[2] - waypoint[2]
                distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                if distance > 0:
                    # 单位方向向量
                    dir_x = dx / distance
                    dir_y = dy / distance
                    dir_z = dz / distance

                    # 计算目标速度（考虑加速/减速）
                    # 加速阶段
                    if current_speed < self.cruise_speed:
                        target_speed = min(self.cruise_speed, current_speed + self.accel * 0.5)
                    else:
                        target_speed = self.cruise_speed

                    # 减速阶段（最后几个点）
                    remaining_points = len(self.waypoints) - i
                    if remaining_points <= 5:
                        decel_factor = remaining_points / 5.0
                        target_speed = min(target_speed, self.cruise_speed * decel_factor)

                    current_speed = target_speed

                    # 设置速度分量
                    velocity.linear.x = dir_x * current_speed
                    velocity.linear.y = dir_y * current_speed
                    velocity.linear.z = dir_z * current_speed

                    # 计算到达该点所需时间
                    if current_speed > 0:
                        segment_time = distance / current_speed
                    else:
                        segment_time = 0.5

                    cumulative_time += segment_time
                else:
                    velocity.linear = Vector3(0, 0, 0)
                    cumulative_time += 0.3
            else:
                # 最后一个点，速度为0
                velocity.linear = Vector3(0, 0, 0)
                current_speed = 0.0

            velocity.angular = Vector3(0, 0, 0)
            point.velocities.append(velocity)

            point.time_from_start = rospy.Duration(cumulative_time)
            traj.points.append(point)

            # 每10个点或最后一个点打印信息
            if i % 10 == 0 or i == len(self.waypoints) - 1:
                speed = np.sqrt(velocity.linear.x ** 2 + velocity.linear.y ** 2 + velocity.linear.z ** 2)
                print(f"  航点 {i + 1}/{len(self.waypoints)}: "
                      f"位置({waypoint[0]:.2f}, {waypoint[1]:.2f}, {waypoint[2]:.2f}) "
                      f"速度: {speed:.2f}m/s "
                      f"时间: {cumulative_time:.1f}s")

        print("=" * 60)
        print(f"✓ 平滑轨迹构建完成！")
        print(f"  总航点数: {len(traj.points)}")
        print(f"  预计飞行时间: {cumulative_time:.1f} 秒")
        print("=" * 60)

        return traj, cumulative_time

    def track_waypoints(self):
        """执行轨迹跟踪（基于路程进度显示）"""
        if len(self.waypoints) == 0:
            print("❌ 轨迹为空，无法跟踪！")
            return False

        # 🔥 插值航点（如果还没插值）
        if not self._waypoints_interpolated:
            self.interpolate_waypoints(max_segment_length=0.5)
            self._waypoints_interpolated = True

        print("等待系统初始化...")
        rospy.sleep(2.0)

        # 创建轨迹消息
        traj_msg, total_time = self.create_trajectory_with_smooth_velocity()

        if traj_msg is None:
            return False

        # 🔥 计算总路程（用于进度显示）
        total_distance = self.calculate_total_path_length()

        print("\n" + "=" * 60)
        print("🚁 开始执行精确轨迹跟踪...")
        print(f"   总路程: {total_distance:.2f}m")
        print(f"   预计时间: {total_time:.1f}s")
        print("=" * 60)

        # 发布轨迹命令
        self.traj_pub.publish(traj_msg)

        # 🔥 使用路程追踪器监控进度
        rate = rospy.Rate(10)  # 10Hz监控频率
        start_time = rospy.Time.now()

        # 初始化路程追踪（仅用于进度显示，不用于采样）
        last_x = None
        last_y = None
        last_z = None
        accumulated_distance = 0.0
        last_progress = -1

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            # 🔥 计算累积路程
            current_x = self.current_position.x
            current_y = self.current_position.y
            current_z = self.current_position.z

            if last_x is not None:
                dx = current_x - last_x
                dy = current_y - last_y
                dz = current_z - last_z
                increment = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
                accumulated_distance += increment

            last_x, last_y, last_z = current_x, current_y, current_z

            # 🔥 基于路程的进度百分比
            if total_distance > 0:
                distance_progress = min(100, int((accumulated_distance / total_distance) * 100))
            else:
                distance_progress = 0

            # 每5%打印一次（基于路程进度）
            if distance_progress // 5 > last_progress // 5:
                speed = np.sqrt(
                    self.current_velocity.x ** 2 +
                    self.current_velocity.y ** 2 +
                    self.current_velocity.z ** 2
                )
                print(f"进度: {distance_progress:3d}% "
                      f"({accumulated_distance:5.1f}m/{total_distance:.1f}m) "
                      f"[{elapsed:5.1f}s] | "
                      f"位置: ({self.current_position.x:6.2f}, "
                      f"{self.current_position.y:6.2f}, "
                      f"{self.current_position.z:6.2f}) | "
                      f"速度: {speed:4.2f}m/s")
                last_progress = distance_progress

            # 🔥 终止条件：路程达到95%且时间超过预计时间
            if accumulated_distance >= total_distance * 0.95 and elapsed >= total_time:
                print(f"\n✓ 已到达目标区域（路程: {accumulated_distance:.2f}m）")
                break

            # 🔥 备用终止条件：时间超过预计时间的1.2倍
            if elapsed >= total_time * 1.2:
                print(f"\n⚠️ 超时终止（路程: {accumulated_distance:.2f}m / {total_distance:.2f}m）")
                break

            rate.sleep()

        print("\n" + "=" * 60)
        print(f"✓ 轨迹跟踪完成！")
        print(f"  实际飞行时间: {elapsed:.1f}s")
        print(f"  实际飞行路程: {accumulated_distance:.2f}m / {total_distance:.2f}m")
        print(f"  最终位置: ({self.current_position.x:.2f}, "
              f"{self.current_position.y:.2f}, "
              f"{self.current_position.z:.2f})")

        # 计算与目标点的误差
        if len(self.waypoints) > 0:
            target = self.waypoints[-1]
            error = np.sqrt(
                (self.current_position.x - target[0]) ** 2 +
                (self.current_position.y - target[1]) ** 2 +
                (self.current_position.z - target[2]) ** 2
            )
            print(f"  位置误差: {error:.3f}m")
        print("=" * 60)

        return True

    def get_current_state(self):
        """获取当前状态"""
        return (self.current_position.x,
                self.current_position.y,
                self.current_position.z)

    def get_progress_percentage(self):
        """计算进度百分比"""
        if self.initial_x is None or self.total_x_distance == 0:
            return 0.0

        traveled_x = abs(self.current_position.x - self.initial_x)
        progress = (traveled_x / self.total_x_distance) * 100.0
        return min(progress, 100.0)

    def reset_for_next_epoch(self):
        """每轮结束后的重置"""
        self.initial_x = None

        # 清空绘图数据
        if self.plotter is not None:
            self.plotter.clear_trajectory()

        print(f"  🔄 UAV 控制器已重置")



class UGVController(BaseRobotController):
    """地面车辆控制器（纯轨迹跟踪）"""

    def __init__(self, waypoint_file='smoothed_trajectory_data.csv', enable_realtime_plot=True):
        self.robot_type = RobotType.UGV
        self.waypoint_file = waypoint_file
        self.waypoints = []
        self.current_waypoint_idx = 0
        self._waypoints_interpolated = False

        # 当前状态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        # 传感器状态
        self._odom_received = False

        # 控制参数
        self.max_linear_speed = 1.5
        self.max_angular_speed = 2.0
        self.waypoint_tolerance = 0.5

        # 基于x方向距离的进度追踪
        self.start_x = None
        self.target_x = None
        self.total_x_distance = 0.0
        self.initial_x = None
        self.load_trajectory()
        # 🔥 新增：实时绘图器
        self.enable_realtime_plot = enable_realtime_plot
        if self.enable_realtime_plot:
            self.plotter = RealtimePlotter(
                robot_name="UGV",
                max_points=None,
                update_interval=50
            )
            print("✓ UGV实时轨迹绘制器已启动")
        else:
            self.plotter = None

        # 数据记录器

        # 发布器
        self.cmd_vel_pub = rospy.Publisher('/nezha_husky1/cmd_vel', Twist, queue_size=10)

        # 订阅器
        rospy.Subscriber('/nezha_husky1/ground_truth/odometry', Odometry, self._odom_callback)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)


        # 🔥 设置目标轨迹到绘图器
        if self.plotter is not None and len(self.waypoints) > 0:
            trajectory = np.array([(x, z) for x, y, z in self.waypoints])
            self.plotter.set_target_trajectory(trajectory)

    def delete_asphalt_plane(self):
        """删除asphalt_plane模型（让UGV入水）"""
        from gazebo_msgs.srv import DeleteModel
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            # 删除地面
        response = delete_model('asphalt_plane')

        if response.success:
                rospy.sleep(2.0)  # 等待物理引擎更新
                return True
        else:
                return False
    def delete_asphalt_plane_small(self):
        """删除asphalt_plane模型（让UGV入水）"""
        from gazebo_msgs.srv import DeleteModel
        rospy.wait_for_service('/gazebo/delete_model')
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

            # 删除地面
        response = delete_model('asphalt_plane_small')

        if response.success:
                rospy.sleep(2.0)  # 等待物理引擎更新
                return True
        else:
                return False
    def adjust_attitude_before_water_entry(self):
        """入水前调整姿态：roll=0, yaw=0"""
        TARGET_ROLL = 0.0
        TARGET_YAW = 0.0
        TOLERANCE_ANGLE = math.radians(3.0)  # 3度容差

        print(f"\n{'=' * 70}")
        print("🎯 调整UGV姿态准备入水...")
        print(f"{'=' * 70}")
        print(f"  目标: Roll=0°, Yaw=0°")

        rate = rospy.Rate(20)
        timeout = 20.0
        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                print("  ⚠️ 姿态调整超时，继续执行")
                break

            # 计算姿态误差
            roll_error = abs(self.current_roll - TARGET_ROLL)
            yaw_error = abs(math.atan2(math.sin(self.current_yaw - TARGET_YAW),
                                       math.cos(self.current_yaw - TARGET_YAW)))

            # 检查是否达到目标
            if roll_error < TOLERANCE_ANGLE and yaw_error < TOLERANCE_ANGLE:
                print(f"  ✓ 姿态已调整完成")
                print(f"    Roll={math.degrees(self.current_roll):.2f}°")
                print(f"    Yaw={math.degrees(self.current_yaw):.2f}°")
                break

            # 发送调整指令
            cmd = Twist()

            # Yaw调整（原地转向）
            if yaw_error > TOLERANCE_ANGLE:
                yaw_correction = math.atan2(math.sin(TARGET_YAW - self.current_yaw),
                                            math.cos(TARGET_YAW - self.current_yaw))
                cmd.angular.z = 1.5 * yaw_correction
                cmd.linear.x = 0.0
            else:
                # Yaw已对齐，缓慢前进帮助Roll调整
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

            self.cmd_vel_pub.publish(cmd)

            rate.sleep()

        # 停止
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)
        print(f"{'=' * 70}\n")
    def move_forward_a_little_bit(self):



        print(f"  目标: Roll=0°, Yaw=0°")

        rate = rospy.Rate(20)
        timeout = 1
        start_time = time.time()

        while not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                print("  ⚠️ 姿态调整超时，继续执行")
                break



            # 发送调整指令
            cmd = Twist()
            cmd.linear.x = 0.3
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()
        # 停止
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)

    def reset_for_next_epoch(self):
        """每轮结束后的重置"""
        self.current_waypoint_idx = 0
        self.initial_x = None

        # 🔥 清空绘图数据
        if self.plotter is not None:
            self.plotter.clear_trajectory()

        print(f"  🔄 UGV 控制器已重置")
    def _odom_callback(self, msg):
        """里程计回调"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

        if self.initial_x is None:
            self.initial_x = self.current_x

        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y
        self.current_vz = msg.twist.twist.linear.z

        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        self.current_roll = roll
        self.current_pitch = pitch
        self.current_yaw = yaw

        if not self._odom_received:
            self._odom_received = True
            print(f"✓ UGV首次接收里程计数据")

    def get_progress_percentage(self):
        """计算基于x方向距离的进度百分比"""
        if self.initial_x is None or self.total_x_distance == 0:
            return 0.0

        traveled_x = abs(self.current_x - self.initial_x)
        progress = (traveled_x / self.total_x_distance) * 100.0
        return min(progress, 100.0)

    def track_waypoints(self):
        """路径点跟踪主循环（完成后调整姿态并删除地面）"""
        print(f"\n🚀 开始UGV路径点跟踪...")
        print(f"   总距离: {self.total_x_distance:.2f}m")
        print(f"   路径点数量: {len(self.waypoints)}\n")

        # 等待里程计数据
        print("⏳ 等待里程计数据...")
        while not self._odom_received and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print("✓ 里程计数据已接收\n")

        waypoint_idx = 0
        rate = rospy.Rate(20)

        # ========== 阶段1：跟踪所有路径点 ==========
        while waypoint_idx < len(self.waypoints) and not rospy.is_shutdown():
            target_x, target_y, target_z = self.waypoints[waypoint_idx]

            current_x = self.current_x
            current_y = self.current_y
            current_z = self.current_z

            # 计算距离
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            # 到达判断
            if distance < self.waypoint_tolerance:
                waypoint_idx += 1
                if waypoint_idx < len(self.waypoints):
                    print(f"📍 到达路径点 {waypoint_idx}/{len(self.waypoints)}")
                    print(f"   位置: X={current_x:.2f}m, Y={current_y:.2f}m, Z={current_z:.2f}m")
                continue

            # 计算目标方向
            angle_to_target = math.atan2(dy, dx)
            angle_error = math.atan2(math.sin(angle_to_target - self.current_yaw),
                                     math.cos(angle_to_target - self.current_yaw))

            # 控制律
            if distance > 2.0:
                linear_velocity = self.max_linear_speed
            elif distance > 1.0:
                linear_velocity = self.max_linear_speed * 0.7
            else:
                linear_velocity = self.max_linear_speed * 0.5 * (distance / 1.0)

            angular_velocity = 2.0 * angle_error
            angular_velocity = np.clip(angular_velocity,
                                       -self.max_angular_speed,
                                       self.max_angular_speed)

            if abs(angle_error) > math.radians(30):
                linear_velocity *= 0.3

            # 发送速度命令
            cmd = Twist()
            cmd.linear.x = linear_velocity
            cmd.angular.z = angular_velocity
            self.cmd_vel_pub.publish(cmd)

            # 更新实时绘图
            if self.plotter is not None:
                self.plotter.add_point(current_x, current_z)

            rate.sleep()

        # ========== 阶段2：停止并调整姿态 ==========
        print(f"\n✅ UGV已完成所有路径点")
        print(f"   最终位置: X={self.current_x:.2f}m, Y={self.current_y:.2f}m, Z={self.current_z:.2f}m")

        # 停止
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1.0)

        # 调整姿态
        self.adjust_attitude_before_water_entry()

        # ========== 阶段3：删除地面 ==========
        success = self.delete_asphalt_plane()
        rospy.sleep(2.0)
        self.move_forward_a_little_bit()
        success = self.delete_asphalt_plane_small()



class UUVController(BaseRobotController):
    """
    🔥 UUV控制器 V16 - 完整版 (包含所有前瞻算法)
    核心特性:
    1. 顺序导航 (移除前瞻算法)
    2. Pitch平滑 (指数平滑)
    3. X-Z双向补偿
    4. 前瞻趋势预测 (深度变化分析)
    5. 推力平滑
    6. 浮力补偿
    7. 动态推力调整
    8. 后方航点检测
    """

    def __init__(self, waypoint_file='smoothed_trajectory_data.csv', enable_realtime_plot=True):
        self.robot_type = RobotType.UUV
        self.waypoint_file = waypoint_file
        self.waypoints = []
        self.trajectory = None
        self.current_waypoint_index = 0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self._waypoints_interpolated = False

        # 订阅odometry获取速度
        rospy.Subscriber("/nezha_husky1/ground_truth/odometry", Odometry,
                         self._odom_callback, queue_size=1)
        # ========== 当前状态 ==========
        self.current_pose = None
        self.prev_thrust_left = 0.0
        self.prev_thrust_right = 0.0
        self.prev_thrust_pitch = 0.0
        self.prev_target_pitch = 0.0
        self.pitch_smoothing_alpha = 0.7  # Pitch平滑系数(0-1,越大越平滑)
        self.smoothing_factor = 0.6  # 推力平滑系数 (0-1),越大越平滑
        self.enable_buoyancy_compensation = True
        self.buoyancy_compensation = 0  # 浮力补偿值(负推力,抵消正浮力)

        # ========== 控制参数 ==========
        self.current_epoch = 0
        self.waypoint_tolerance = 0.8  # 航点容差(从1.5改为0.8米)
        self.fixed_y = 0.0

        # ========== 进度追踪 ==========
        self.start_x = None
        self.target_x = None
        self.total_x_distance = 0.0
        self.initial_x = None

        # 加载轨迹
        self.load_trajectory()

        # ========== 实时绘图器 ==========
        self.enable_realtime_plot = enable_realtime_plot
        if self.enable_realtime_plot:
            self.plotter = RealtimePlotter(
                robot_name="UUV",
                max_points=None,  # 不限制点数
                update_interval=50
            )
            if self.trajectory is not None:
                self.plotter.set_target_trajectory(self.trajectory)
            print("✓ UUV实时轨迹绘制器已启动")
        else:
            self.plotter = None

        # ========== ROS订阅器 ==========
        rospy.Subscriber("/nezha_husky1/ground_truth/pose", Pose,
                         self._pose_callback, queue_size=1)

        # ========== ROS发布器 ==========
        self.pub_pitch = rospy.Publisher("/nezha_husky1/thrusters/0/input",
                                         FloatStamped, queue_size=10)
        self.pub_left = rospy.Publisher("/nezha_husky1/thrusters/1/input",
                                        FloatStamped, queue_size=10)
        self.pub_right = rospy.Publisher("/nezha_husky1/thrusters/2/input",
                                         FloatStamped, queue_size=10)

        # ========== PID控制器 ==========
        # Pitch PID控制器
        self.pitch_pid = PIDController(
            kp=100.0,  # 从90.0增加（更强响应）
            ki=10.0,  # 从8.0增加
            kd=80.0,  # 从70.0增加
            output_min=-70.0, output_max=70.0,
            windup_limit=30.0
        )

        # 位置PID控制器
        self.position_pid = PIDController(
            kp=18.0, ki=2.0, kd=10.0,
            output_min=-30.0, output_max=30.0,
            windup_limit=12.0
        )

        # Roll PID控制器
        self.roll_pid = PIDController(
            kp=30.0,  # 比例增益
            ki=3.0,  # 积分增益
            kd=15.0,  # 微分增益
            output_min=-20.0,
            output_max=20.0,
            windup_limit=10.0
        )

        # Yaw PID控制器
        self.yaw_pid = PIDController(
            kp=30.0,  # 从25.0增加
            ki=3.0,  # 从2.0增加
            kd=12.0,  # 从10.0增加
            output_min=-20.0,  # 从-15.0增加
            output_max=20.0,  # 从15.0增加
            windup_limit=10.0  # 从8.0增加
        )

        # 深度PID控制器
        self.depth_pid = PIDController(
            kp=70.0,  # 从50.0增加（提高响应）
            ki=10.0,  # 从5.0增加（增强积分）
            kd=80.0,  # 从60.0增加（减少震荡）
            output_min=-50.0,  # 从-40.0增加
            output_max=50.0,  # 从40.0增加
            windup_limit=25.0  # 从15.0增加
        )

        # ========== Gazebo服务 ==========
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        print("✓ UUV控制器V16初始化完成")

    def _odom_callback(self, msg):
        """里程计回调（获取速度）"""
        self.current_vx = msg.twist.twist.linear.x
        self.current_vy = msg.twist.twist.linear.y
        self.current_vz = msg.twist.twist.linear.z

    def _pose_callback(self, pose):
        """位姿回调函数"""
        self.current_pose = pose

        if self.initial_x is None:
            self.initial_x = pose.position.x

        # 更新实时绘图
        if self.plotter is not None:
            self.plotter.add_point(pose.position.x, pose.position.z)

    def get_current_angles(self):
        """获取当前欧拉角"""
        if self.current_pose is None:
            return None, None, None

        q = [self.current_pose.orientation.x,
             self.current_pose.orientation.y,
             self.current_pose.orientation.z,
             self.current_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)
        return roll, pitch, yaw

    def get_current_position(self):
        """获取当前位置"""
        if self.current_pose is None:
            return None, None, None
        return (self.current_pose.position.x,
                self.current_pose.position.y,
                self.current_pose.position.z)

    def set_thrusters_smooth(self, T_pitch, T_left, T_right):
        """
        带指数平滑的推进器控制 - 消除抖动

        推进器物理特性:
        - 正推力 (+) → Pitch减小(变负) → 机头抬头 ⬆️ → 仰角增大
        - 负推力 (-) → Pitch增大(变正) → 机头低头 ⬇️ → 俯角增大

        控制逻辑:
        - 需要上浮(depth减小) → 需要抬头 → 输出正推力
        - 需要下潜(depth增大) → 需要低头 → 输出负推力
        - 机体有正浮力 → 需要持续负推力补偿 → 防止抬头
        """
        # 限幅
        T_pitch = max(min(T_pitch, 50.0), -50.0)
        T_r = max(min(T_right, 40.0), -40.0)
        T_l = max(min(T_left, 40.0), -40.0)

        # 🔥 指数移动平均平滑
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

    def stop_thrusters(self):
        """停止所有推进器"""
        self.set_thrusters_smooth(0, 0, 0)

    def get_lookahead_depth_trend(self, lookahead_points=4):
        """
        🔥 前瞻未来几个航点，计算深度变化趋势

        参数:
            lookahead_points: 前瞻的航点数量(默认4个)

        返回:
            average_dz: 平均深度变化量
            trend: 趋势类型 ('steep_climb', 'climb', 'level', 'descent', 'steep_descent')
        """
        if self.trajectory is None or len(self.trajectory) == 0:
            return 0.0, 'level'

        current_x, current_y, current_z = self.get_current_position()
        if current_x is None:
            return 0.0, 'level'

        start_idx = self.current_waypoint_index
        end_idx = min(start_idx + lookahead_points, len(self.trajectory))

        if start_idx >= len(self.trajectory):
            return 0.0, 'level'

        # 收集未来航点的深度
        future_depths = []
        for i in range(start_idx, end_idx):
            future_depths.append(self.trajectory[i][1])  # Z坐标

        if not future_depths:
            return 0.0, 'level'

        # 计算平均未来深度
        average_future_z = sum(future_depths) / len(future_depths)
        average_dz = average_future_z - current_z

        # 判断趋势类型
        if average_dz > 1.0:
            trend = 'steep_climb'  # 陡峭上升
        elif average_dz > 0.3:
            trend = 'climb'  # 上升
        elif average_dz < -1.0:
            trend = 'steep_descent'  # 陡峭下降
        elif average_dz < -0.3:
            trend = 'descent'  # 下降
        else:
            trend = 'level'  # 平稳

        return average_dz, trend

    def calculate_distance_xz(self, x1, z1, x2, z2):
        """计算x-z平面距离"""
        return math.sqrt((x2 - x1) ** 2 + (z2 - z1) ** 2)

    def navigate_to_waypoint(self, target_x, target_z, target_y=None,
                             distance_tolerance=0.8, timeout=60.0):
        """
        🔥 导航到指定waypoint - V16完整版
        融合: 前瞻趋势预测 + X-Z补偿 + Pitch平滑 + 动态推力调整

        参数:
            target_x: 目标X坐标
            target_z: 目标Z坐标(深度)
            target_y: 目标Y坐标(默认为fixed_y)
            distance_tolerance: 到达容差(米)
            timeout: 超时时间(秒)

        返回:
            bool: 是否成功到达
        """
        if target_y is None:
            target_y = self.fixed_y

        # 🔥 X方向补偿
        X_OFFSET = 0.8
        target_x = target_x + X_OFFSET

        # 等待位姿数据
        while self.current_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # ========== 初始后方检测 ==========
        current_x, current_y, current_z = self.get_current_position()
        roll, pitch, yaw = self.get_current_angles()

        dx = target_x - current_x
        dy = target_y - current_y

        target_yaw = math.atan2(dy, dx)
        yaw_diff = math.atan2(math.sin(target_yaw - yaw),
                              math.cos(target_yaw - yaw))

        # 如果Pitch角度不大，且航点在后方，直接跳过
        if abs(pitch) < math.radians(20.0):
            if abs(yaw_diff) > math.radians(120):
                print(f"  ⚠️ 航点在后方(yaw_diff={math.degrees(yaw_diff):.1f}°)，跳过")
                return False

        # ========== 重置PID ==========
        self.position_pid.reset()
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.depth_pid.reset()
        self.roll_pid.reset()

        rate = rospy.Rate(100)  # 100Hz控制频率
        start_time = time.time()

        # ========== 容差定义 ==========
        DEPTH_TOLERANCE = 0.35
        ROLL_TOLERANCE = math.radians(2.0)
        YAW_TOLERANCE = math.radians(3.0)

        # 🔥 X-Z补偿参数
        LOOKAHEAD_X_OFFSET = 1.2  # X方向前瞻补偿
        LOOKAHEAD_Z_RATIO = 0.8  # Z方向前瞻比例

        # ========== 状态变量 ==========
        prev_pitch = None
        prev_time = time.time()
        prev_depth = None
        last_print_time = time.time()
        PRINT_INTERVAL = 2.0

        backward_count = 0
        MAX_BACKWARD_COUNT = 15

        # ========== 主控制循环 ==========
        while not rospy.is_shutdown():
            current_time = time.time()

            # 超时检查
            if current_time - start_time > timeout:
                print("  ✗ 导航超时!")
                return False

            # 获取当前状态
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

            # ========== 到达判断 ==========
            target_yaw_angle = 0.0  # 保持朝向X轴正方向
            yaw_error = math.atan2(math.sin(target_yaw_angle - yaw),
                                   math.cos(target_yaw_angle - yaw))

            # 后方检测
            if abs(yaw_error) > math.radians(90):
                backward_count += 1
                if backward_count > MAX_BACKWARD_COUNT:
                    print(f"  ⚠️ 航点持续在后方，放弃")
                    return False
            else:
                backward_count = 0

            position_ok = (distance_xz < distance_tolerance and
                           distance_z < DEPTH_TOLERANCE)
            attitude_ok = (abs(roll) < ROLL_TOLERANCE and
                           abs(yaw_error) < YAW_TOLERANCE)

            if position_ok and attitude_ok:
                print(f"  ✓ 到达waypoint! X-Z:{distance_xz:.2f}m, Z:{distance_z:.2f}m")
                break

            # 计算Pitch速度
            if prev_pitch is not None:
                pitch_velocity = (pitch - prev_pitch) / dt
            else:
                pitch_velocity = 0.0
            prev_pitch = pitch
            prev_time = current_time

            # ========== 🔥 前瞻深度趋势分析 ==========
            lookahead_dz, depth_trend = self.get_lookahead_depth_trend(lookahead_points=4)

            # 根据趋势调整策略
            if depth_trend == 'steep_climb':
                trend_boost = math.radians(15.0)  # 陡峭上升: 大幅增加抬头角度
                thrust_boost = 1.3  # 增加推力30%
            elif depth_trend == 'climb':
                trend_boost = math.radians(8.0)  # 上升: 适度增加抬头角度
                thrust_boost = 1.15  # 增加推力15%
            elif depth_trend == 'steep_descent':
                trend_boost = math.radians(-15.0)  # 陡峭下降: 大幅增加低头角度
                thrust_boost = 1.3  # 增加推力30%
            elif depth_trend == 'descent':
                trend_boost = math.radians(-8.0)  # 下降: 适度增加低头角度
                thrust_boost = 1.15  # 增加推力15%
            else:
                trend_boost = 0.0  # 平稳: 不调整
                thrust_boost = 1.0

            # ========== 🔥 深度控制(Pitch目标生成) ==========
            # 1. PID深度控制
            raw_pid_output = self.depth_pid.update(current_z, target_z, current_time)

            # 2. 动态限幅(根据深度误差)
            if abs(dz) > 1.0:
                MAX_DEPTH_CORRECTION = 35.0
            elif abs(dz) > 0.5:
                MAX_DEPTH_CORRECTION = 30.0
            elif abs(dz) > 0.2:
                MAX_DEPTH_CORRECTION = 25.0
            else:
                MAX_DEPTH_CORRECTION = 20.0

            pitch_correction_deg = max(min(raw_pid_output, MAX_DEPTH_CORRECTION),
                                       -MAX_DEPTH_CORRECTION)
            pitch_correction_rad = math.radians(pitch_correction_deg)

            # ========== 🔥 X-Z双向补偿 ==========
            # X方向补偿
            if dx > 0:
                compensated_target_x = target_x + LOOKAHEAD_X_OFFSET
            else:
                compensated_target_x = target_x - LOOKAHEAD_X_OFFSET

            dx_compensated = compensated_target_x - current_x

            # Z方向补偿(基于前瞻趋势)
            if abs(lookahead_dz) > 0.2:
                z_offset = lookahead_dz * LOOKAHEAD_Z_RATIO
                compensated_target_z = target_z + z_offset
            else:
                compensated_target_z = target_z

            dz_compensated = compensated_target_z - current_z

            horizontal_base_compensated = max(abs(dx_compensated), 0.1)
            distance_xz_compensated = math.sqrt(dx_compensated ** 2 + dz_compensated ** 2)

            # ========== 🔥 几何前馈 ==========
            target_pitch_geometry_raw = -math.atan2(dz_compensated, horizontal_base_compensated)

            # 权重策略(根据趋势和距离)
            if depth_trend in ['steep_climb', 'steep_descent']:
                distance_weight = 1.0  # 陡峭变化: 完全信任几何前馈
            elif depth_trend in ['climb', 'descent']:
                distance_weight = 0.85 if distance_xz < 1.0 else 0.75
            else:
                if abs(dz) > 1.0:
                    distance_weight = 1.0
                elif abs(dz) > 0.5:
                    distance_weight = 0.85 if distance_xz < 1.0 else 0.75
                elif abs(dz) > 0.3:
                    distance_weight = 0.75 if distance_xz < 1.0 else 0.6
                else:
                    distance_weight = 0.5

            target_pitch_geometry = target_pitch_geometry_raw * distance_weight

            # ========== 🔥 计算总目标Pitch + 平滑 ==========
            target_pitch_raw = target_pitch_geometry + pitch_correction_rad + trend_boost

            # 指数平滑
            target_pitch_final = (self.pitch_smoothing_alpha * self.prev_target_pitch +
                                  (1 - self.pitch_smoothing_alpha) * target_pitch_raw)

            self.prev_target_pitch = target_pitch_final

            # 动态限幅
            if depth_trend in ['steep_climb', 'steep_descent']:
                GLOBAL_MAX_PITCH = math.radians(55.0)
            elif depth_trend in ['climb', 'descent']:
                GLOBAL_MAX_PITCH = math.radians(45.0)
            elif abs(dz) > 1.0:
                GLOBAL_MAX_PITCH = math.radians(50.0)
            elif abs(dz) > 0.5:
                GLOBAL_MAX_PITCH = math.radians(40.0)
            else:
                GLOBAL_MAX_PITCH = math.radians(30.0)

            target_pitch_final = max(min(target_pitch_final, GLOBAL_MAX_PITCH),
                                     -GLOBAL_MAX_PITCH)

            # ========== 🔥 Pitch姿态控制 ==========
            pid_output = self.pitch_pid.update(target_pitch_final, pitch, current_time)
            pitch_thrust = -pid_output  # 注意符号反转

            # 浮力补偿
            buoyancy_comp = self.buoyancy_compensation \
                if self.enable_buoyancy_compensation else 0.0

            pitch_output = pitch_thrust + buoyancy_comp

            # 大角度爬升时强制推力下限
            if abs(dz) > 0.3:
                if dz > 0:  # 需要上升
                    if abs(pitch) > math.radians(25.0):
                        min_pitch_thrust = 25.0
                    elif abs(pitch) > math.radians(20.0):
                        min_pitch_thrust = 20.0
                    else:
                        min_pitch_thrust = 15.0

                    if pitch_output > 0:
                        pitch_output = max(pitch_output, min_pitch_thrust)

                elif dz < 0:  # 需要下降
                    if abs(pitch) > math.radians(20.0):
                        max_pitch_thrust = -20.0
                        if pitch_output < 0:
                            pitch_output = min(pitch_output, max_pitch_thrust)

            pitch_output = max(min(pitch_output, 70.0), -70.0)

            # ========== 🔥 Roll控制 ==========
            TARGET_ROLL = 0.0
            roll_correction = self.roll_pid.update(TARGET_ROLL, roll, current_time)

            # 根据Roll误差动态调整增益
            roll_deg = math.degrees(abs(roll))
            if roll_deg > 10.0:
                roll_correction *= 1.5  # 大误差: 增强控制
            elif roll_deg < 2.0:
                roll_correction *= 0.5  # 小误差: 减弱控制
            if roll_deg < 1.0:
                roll_correction *= 0.3  # 极小误差: 进一步减弱

            # ========== 🔥 Yaw控制 ==========
            yaw_correction = self.yaw_pid.update(target_yaw_angle, yaw, current_time)

            # 根据Yaw误差动态调整增益
            yaw_error_deg = abs(math.degrees(yaw_error))
            if yaw_error_deg > 30.0:
                yaw_correction *= 1.3
            elif yaw_error_deg < 5.0:
                yaw_correction *= 0.6
            if yaw_error_deg < 2.0:
                yaw_correction *= 0.3

            # ========== 🔥 水平推力 ==========
            thrust_raw = self.position_pid.update(0.0, distance_xz_compensated, current_time)

            # 动态推力范围
            if distance_xz_compensated > 5.0:
                min_thrust = 8.0
                max_thrust = 30.0
            elif distance_xz_compensated > 2.0:
                min_thrust = 5.0
                max_thrust = 25.0
            else:
                min_thrust = 3.0
                max_thrust = 20.0

            thrust = max(min(abs(thrust_raw), max_thrust), min_thrust)
            thrust *= thrust_boost  # 应用趋势增益

            # 接近目标时的推力策略
            if distance_xz < 1.0:
                is_steep_climb = (depth_trend in ['steep_climb', 'climb'] or
                                  (abs(dz) > 0.3 and abs(pitch) > math.radians(20.0)))

                if is_steep_climb:
                    # 陡峭爬升: 保持较高推力
                    decay_factor = max(0.85, distance_xz / 1.0)
                    thrust *= decay_factor
                    thrust = max(thrust, 20.0)  # 最低20N
                else:
                    # 平稳接近: 正常衰减
                    decay_factor = max(0.6, distance_xz / 1.0)
                    thrust *= decay_factor

            # ========== 🔥 差分推力分配 ==========
            # 动态权重(根据距离)
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

            MAX_DIFFERENTIAL = 15.0
            differential = max(min(differential, MAX_DIFFERENTIAL),
                               -MAX_DIFFERENTIAL)

            thrust_left = thrust - differential
            thrust_right = thrust + differential

            thrust_left = max(min(thrust_left, 40.0), -40.0)
            thrust_right = max(min(thrust_right, 40.0), -40.0)

            # ========== 打印信息 ==========
            if current_time - last_print_time > PRINT_INTERVAL:
                print(f"[{self.current_waypoint_index + 1}/{len(self.trajectory)}] "
                      f"X:{current_x:+6.2f}→{target_x:+6.2f}({dx:+5.2f}) | "
                      f"Z:{current_z:+6.2f}→{target_z:+6.2f}({dz:+5.2f}) | "
                      f"Dist:{distance_xz:5.2f}m | "
                      f"Trend:{depth_trend} | "
                      f"Pitch:{math.degrees(pitch):+5.1f}°→{math.degrees(target_pitch_final):+5.1f}°")
                last_print_time = current_time

            # ========== 发送控制指令 ==========
            self.set_thrusters_smooth(
                pitch_output,
                thrust_left,
                thrust_right
            )

            rate.sleep()

        return True

    def track_waypoints(self):
        """轨迹跟踪主函数 - 顺序导航版本"""
        if self.trajectory is None or len(self.trajectory) == 0:
            print("❌ 轨迹为空,无法跟踪!")
            return False

        print(f"\n{'#' * 60}")
        print(f"# 开始UUV轨迹跟踪(顺序导航V16)")
        print(f"# 特性: 前瞻趋势预测 + X-Z补偿 + Pitch平滑")
        print(f"{'#' * 60}\n")

        self.current_waypoint_index = 0
        start_time = time.time()

        while not rospy.is_shutdown():
            if self.current_waypoint_index >= len(self.trajectory):
                print(f"\n{'=' * 60}")
                print(f"✓✓✓ UUV轨迹跟踪完成!")
                print(f"总用时: {time.time() - start_time:.1f}秒")
                print(f"总航点数: {len(self.trajectory)}")
                print(f"{'=' * 60}\n")
                break

            target_x, target_z = self.trajectory[self.current_waypoint_index]

            print(f"\n{'─' * 60}")
            print(f"📍 Waypoint {self.current_waypoint_index + 1}/{len(self.trajectory)}")
            print(f"{'─' * 60}")
            print(f"目标: X={target_x:.2f}, Z={target_z:.2f}, Y={self.fixed_y:.2f}")

            success = self.navigate_to_waypoint(
                target_x, target_z,
                target_y=self.fixed_y,
                distance_tolerance=self.waypoint_tolerance,
                timeout=60.0
            )

            if success:
                self.current_waypoint_index += 1
                print(f"✓ 航点 {self.current_waypoint_index} 完成")
            else:
                print(f"✗ 导航失败,尝试下一个航点")
                self.current_waypoint_index += 1

        self.stop_thrusters()
        return True

    def get_current_state(self):
        """获取当前状态"""
        if self.current_pose is None:
            return None

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z

        q = [self.current_pose.orientation.x,
             self.current_pose.orientation.y,
             self.current_pose.orientation.z,
             self.current_pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        return (x, y, z), (roll, pitch, yaw)

    def get_progress_percentage(self):
        """计算进度百分比(基于X方向行进距离)"""
        if self.initial_x is None or self.total_x_distance == 0:
            return 0.0

        current_x = self.current_pose.position.x if self.current_pose else self.initial_x
        traveled_x = abs(current_x - self.initial_x)
        progress = (traveled_x / self.total_x_distance) * 100.0
        return min(progress, 100.0)

    def reset_model_to_origin(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """重置Gazebo中的模型到指定位置和姿态"""
        try:
            print(f"\n{'─' * 70}")
            print(f"🔄 重置UUV模型到原点...")
            print(f"{'─' * 70}")

            model_state = ModelState()
            model_state.model_name = 'nezha_husky1'
            model_state.pose.position = Point(x, y, z)

            # 使用tf库转换欧拉角到四元数
            import tf
            quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            model_state.pose.orientation = Quaternion(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            )

            model_state.twist = Twist()
            model_state.reference_frame = 'world'

            response = self.set_model_state(model_state)

            if response.success:
                print(f"✅ UUV模型位置已重置到: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                print(f"✅ UUV模型姿态已重置为: Roll={math.degrees(roll):.2f}°, "
                      f"Pitch={math.degrees(pitch):.2f}°, Yaw={math.degrees(yaw):.2f}°")

                # 等待物理引擎稳定
                rospy.sleep(3.0)

                # 重置PID控制器
                self.position_pid.reset()
                self.yaw_pid.reset()
                self.pitch_pid.reset()
                self.depth_pid.reset()
                self.roll_pid.reset()
                print(f"✅ UUV PID控制器已重置")

                return True
            else:
                print(f"❌ UUV模型重置失败: {response.status_message}")
                return False

        except Exception as e:
            print(f"❌ UUV重置错误: {e}")
            import traceback
            traceback.print_exc()
            return False

    def reset_for_next_epoch(self):
        """重置控制器准备下一轮"""
        self.current_waypoint_index = 0
        self.prev_thrust_left = 0.0
        self.prev_thrust_right = 0.0
        self.prev_thrust_pitch = 0.0
        self.prev_target_pitch = 0.0
        self.initial_x = None

        # PID清零
        self.position_pid.reset()
        self.yaw_pid.reset()
        self.pitch_pid.reset()
        self.depth_pid.reset()
        self.roll_pid.reset()

        # 清空绘图数据
        if self.plotter is not None:
            self.plotter.clear_trajectory()

        print(f"  🔄 UUV控制器已重置")


class MultiRobotController:
    """三机器人协同控制器"""

    def __init__(self):
        rospy.init_node('multi_robot_controller', anonymous=True)
        print(f"\n{'#' * 70}")
        print(f"# 三机器人协同轨迹跟踪系统")
        print(f"# 执行顺序: UGV (x:0-10, z>0) -> UUV (x:10-29, z<0) -> UAV (x:29+)")
        print(f"{'#' * 70}\n")

        # 初始化三个控制器（禁用各自的绘图器）
        self.ugv = UGVController('smoothed_trajectory_data.csv', enable_realtime_plot=False)
        self.uuv = UUVController('smoothed_trajectory_data.csv', enable_realtime_plot=False)
        self.uav = UAVController('smoothed_trajectory_data.csv', enable_realtime_plot=False)

        # 🔥 创建全局绘图器
        self.global_plotter = RealtimePlotter(
            robot_name="Multi-Robot System",
            max_points=None,
            update_interval=100
        )

        # 🔥 直接从CSV加载完整轨迹
        self._load_complete_trajectory_from_csv()
        self.data_recorder = GlobalDataRecorder()
        self.data_recorder.initialize_force_services()

        # 初始化三个控制器（禁用各自的绘图器）
        self.ugv = UGVController('smoothed_trajectory_data.csv', enable_realtime_plot=False)
        self.uuv = UUVController('smoothed_trajectory_data.csv', enable_realtime_plot=False)
        self.uav = UAVController('smoothed_trajectory_data.csv', enable_realtime_plot=False)
        rospy.sleep(2.0)

        # 等待绘图窗口初始化
        print("⏳ 等待绘图窗口初始化...")
        time.sleep(3.0)
        print("✓ 所有绘图窗口已就绪\n")

    def _load_complete_trajectory_from_csv(self):
        """直接从CSV文件加载完整轨迹"""
        try:
            import pandas as pd
            df = pd.read_csv('smoothed_trajectory_data.csv')

            # 提取所有x和z坐标
            trajectory = df[['x', 'z']].values

            # 设置到全局绘图器
            self.global_plotter.set_target_trajectory(trajectory)

            print(f"✓ 从CSV加载完整轨迹: {len(trajectory)} 个点")
            print(f"  X范围: {trajectory[:, 0].min():.2f} → {trajectory[:, 0].max():.2f}")
            print(f"  Z范围: {trajectory[:, 1].min():.2f} → {trajectory[:, 1].max():.2f}")

        except Exception as e:
            print(f"❌ 加载完整轨迹失败: {e}")
            import traceback
            traceback.print_exc()

    def interpolate_waypoints(self, max_segment_length=0.5):
        """
        插值航点，使航点间距更小更均匀（通用方法）

        参数:
            max_segment_length: 最大段长（米）
        """
        if len(self.waypoints) < 2:
            print(f"⚠️ {self.robot_type.value}: 航点数量不足，跳过插值")
            return

        interpolated = [self.waypoints[0]]

        for i in range(len(self.waypoints) - 1):
            current = self.waypoints[i]
            next_wp = self.waypoints[i + 1]

            dx = next_wp[0] - current[0]
            dy = next_wp[1] - current[1]
            dz = next_wp[2] - current[2]
            distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            if distance > max_segment_length:
                # 需要插值
                num_segments = int(np.ceil(distance / max_segment_length))

                for j in range(1, num_segments):
                    t = j / float(num_segments)
                    interpolated_wp = (
                        current[0] + t * dx,
                        current[1] + t * dy,
                        current[2] + t * dz
                    )
                    interpolated.append(interpolated_wp)

            interpolated.append(next_wp)

        original_count = len(self.waypoints)
        self.waypoints = interpolated

        print("=" * 60)
        print(f"✓ {self.robot_type.value} 航点插值完成: {original_count} → {len(self.waypoints)} 个")
        print(f"  最大段长: {max_segment_length} 米")
        print("=" * 60)

    def _track_with_global_plot_and_record(self, controller, robot_name):
        """
        🔥 在轨迹跟踪时更新全局绘图 + 基于路程进度均匀记录数据
        """
        print(f"🚀 开始 {robot_name} 轨迹跟踪...")

        # 🔥 关键：先对所有机器人的航点进行插值
        if not hasattr(controller, '_waypoints_interpolated'):
            controller.interpolate_waypoints(max_segment_length=0.5)
            controller._waypoints_interpolated = True
            print(f"✓ {robot_name} 插值后航点数: {len(controller.waypoints)}")

        # 🔥 计算总路程
        total_distance = controller.calculate_total_path_length()
        num_samples = len(controller.waypoints)

        print(f"📏 {robot_name} 路径信息:")
        print(f"   总路程: {total_distance:.2f}m")
        print(f"   目标采样数: {num_samples}")
        print(f"   采样间隔: {total_distance / num_samples:.3f}m")

        # 🔥 创建路程追踪器
        distance_tracker = DistanceTracker(total_distance, num_samples)

        # 启动独立线程：绘图 + 数据记录
        import threading
        stop_event = threading.Event()

        def update_plot_and_record():
            """基于路程进度更新绘图和记录数据"""
            rate = rospy.Rate(20)  # 20Hz更新频率（提高频率以精确捕捉位置）

            while not stop_event.is_set() and not rospy.is_shutdown():
                try:
                    # ========== 获取当前状态 ==========
                    if robot_name == 'UGV':
                        x = controller.current_x
                        y = controller.current_y
                        z = controller.current_z
                        vx = controller.current_vx
                        vy = controller.current_vy
                        vz = controller.current_vz
                        roll = controller.current_roll
                        pitch = controller.current_pitch
                        yaw = controller.current_yaw

                    elif robot_name == 'UUV':
                        if controller.current_pose is None:
                            rate.sleep()
                            continue

                        x = controller.current_pose.position.x
                        y = controller.current_pose.position.y
                        z = controller.current_pose.position.z
                        vx = controller.current_vx
                        vy = controller.current_vy
                        vz = controller.current_vz
                        roll, pitch, yaw = controller.get_current_angles()

                    elif robot_name == 'UAV':
                        x = controller.current_position.x
                        y = controller.current_position.y
                        z = controller.current_position.z
                        vx = controller.current_velocity.x
                        vy = controller.current_velocity.y
                        vz = controller.current_velocity.z
                        roll = controller.current_roll
                        pitch = controller.current_pitch
                        yaw = controller.current_yaw

                    # 更新全局绘图（每次都更新）
                    self.global_plotter.add_point(x, z)

                    # 🔥 基于路程进度判断是否采样
                    should_sample = distance_tracker.update(x, y, z)

                    if should_sample:
                        # 获取受力数据
                        forces = self.data_recorder.get_forces()

                        # 记录数据
                        self.data_recorder.record_data_point(
                            robot_type_str=robot_name,
                            position=(x, y, z),
                            velocity=(vx, vy, vz),
                            attitude=(roll, pitch, yaw),
                            forces=forces
                        )

                        # 打印采样信息
                        progress = distance_tracker.get_progress()
                        sample_count = distance_tracker.sample_count

                        if sample_count % 10 == 0 or sample_count == num_samples:
                            print(f"📊 {robot_name} 数据采样: {sample_count}/{num_samples} "
                                  f"({progress:.1f}%) - 累积路程: {distance_tracker.accumulated_distance:.2f}m")

                except Exception as e:
                    print(f"⚠️ {robot_name} 数据记录错误: {e}")
                    import traceback
                    traceback.print_exc()

                rate.sleep()

            # 🔥 确保最后一个点被记录
            if distance_tracker.sample_count < num_samples:
                try:
                    if robot_name == 'UGV':
                        x, y, z = controller.current_x, controller.current_y, controller.current_z
                        vx, vy, vz = controller.current_vx, controller.current_vy, controller.current_vz
                        roll, pitch, yaw = controller.current_roll, controller.current_pitch, controller.current_yaw
                    elif robot_name == 'UUV':
                        x = controller.current_pose.position.x
                        y = controller.current_pose.position.y
                        z = controller.current_pose.position.z
                        vx, vy, vz = controller.current_vx, controller.current_vy, controller.current_vz
                        roll, pitch, yaw = controller.get_current_angles()
                    elif robot_name == 'UAV':
                        x, y, z = controller.current_position.x, controller.current_position.y, controller.current_position.z
                        vx, vy, vz = controller.current_velocity.x, controller.current_velocity.y, controller.current_velocity.z
                        roll, pitch, yaw = controller.current_roll, controller.current_pitch, controller.current_yaw

                    forces = self.data_recorder.get_forces()
                    self.data_recorder.record_data_point(
                        robot_type_str=robot_name,
                        position=(x, y, z),
                        velocity=(vx, vy, vz),
                        attitude=(roll, pitch, yaw),
                        forces=forces
                    )
                    distance_tracker.sample_count += 1
                    print(f"📊 {robot_name} 终点数据已记录")
                except:
                    pass

            print(f"✓ {robot_name} 数据记录完成: {distance_tracker.sample_count}/{num_samples} 点")
            print(f"  实际路程: {distance_tracker.accumulated_distance:.2f}m / {total_distance:.2f}m")

        plot_thread = threading.Thread(target=update_plot_and_record, daemon=True)
        plot_thread.start()

        # 🔥 执行轨迹跟踪（使用原有方法）
        success = controller.track_waypoints()
        success = True
        # 等待线程完成（给一点时间记录最后的数据）
        rospy.sleep(0.5)

        # 停止绘图线程
        stop_event.set()
        plot_thread.join(timeout=2.0)

        print(f"✓ {robot_name} 完成")
        return success

    def run_single_epoch(self, epoch):
        print(f"\n{'=' * 70}")
        print(f"开始第 {epoch} 轮任务")
        print(f"{'=' * 70}\n")

        # ========== UGV 阶段 ==========
        print("🚗 阶段 1: UGV 地面行驶")
        ugv_success = self._track_with_global_plot_and_record(self.ugv, 'UGV')

        if not ugv_success:
            print("❌ UGV 阶段失败，终止任务")
            return False

        print("✅ UGV 阶段完成\n")
        rospy.sleep(2.0)

        # ========== UUV 阶段 ==========
        print("🌊 阶段 2: UUV 水下航行")
        uuv_success = self._track_with_global_plot_and_record(self.uuv, 'UUV')

        if not uuv_success:
            print("❌ UUV 阶段失败，终止任务")
            return False

        if self.uuv.current_waypoint_index < len(self.uuv.trajectory):
            print(f"⚠️ UUV 未完成所有航点！")
            print(f"   当前进度: {self.uuv.current_waypoint_index}/{len(self.uuv.trajectory)}")
            return False

        print("✅ UUV 阶段完成\n")
        rospy.sleep(2.0)

        # ========== UAV 阶段 ==========
        print("✈️ 阶段 3: UAV 空中飞行")
        uav_success = self._track_with_global_plot_and_record(self.uav, 'UAV')

        if not uav_success:
            print("❌ UAV 阶段失败")
            return False

        print("✅ 所有阶段完成！\n")

        # 🔥 保存数据
        self.data_recorder.save_to_csv(f'trajectory_data_with_forces_{time.time()}.csv')

        return True


# ==================== 主函数 ====================

def main():
    try:
        controller = MultiRobotController()
        controller.run_single_epoch(1)

    except rospy.ROSInterruptException:
        print("\n⚠️ ROS中断")
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 🔥 新增：关闭传感器可视化
        print("\n程序结束")


if __name__ == "__main__":
    main()
