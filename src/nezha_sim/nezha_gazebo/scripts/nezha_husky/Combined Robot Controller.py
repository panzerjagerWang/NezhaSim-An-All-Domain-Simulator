#!/usr/bin/env python
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# -*- coding: utf-8 -*-

"""
Combined Robot Controller (Simplified)

Uses separate controller modules for UGV, UUV, and UAV operations.
"""
import threading

import rospy
import math
from geometry_msgs.msg import Pose
from UGV_controller import *
from UUV_controller import *
from UAV_controller import *
from UUVControllerV8 import UnderwaterNavigation  # 使用新的控制器


class CombinedRobotController:
    """
    Combined Robot Controller

    Mission Profile:
    1. UGV drives forward 20 meters on land
    2. UGV falls into water, transitions to UUV control
    3. UUV ascends to surface (z ≈ -0.5m)
    4. UAV takes off to 2 meters altitude
    5. UAV returns to origin point
    """

    def __init__(self):
        # Initialize controllers
        self.ugv = UGVController()

        # 为UUV控制器准备完整参数
        class UUVArgs:
            def __init__(self):
                # 航点参数
                self.wp_x = 0.0
                self.wp_y = 0.0
                self.wp_z = -0.5  # 目标深度

                # PID参数 - Pitch控制
                self.K_pitch = 30.0  # Pitch比例增益
                self.K_pitch_i = 0.2  # Pitch积分增益
                self.K_pitch_d = 15.0  # Pitch微分增益

                # PID参数 - Yaw控制
                self.K_yaw = 20.0  # Yaw比例增益
                self.K_yaw_i = 0.1  # Yaw积分增益
                self.K_yaw_d = 10.0  # Yaw微分增益

                # 前进推力参数
                self.K_fwd = 1.0  # 前进推力系数
                self.T_fwd_max = 20.0  # 最大前进推力

                # 姿态限制（度）
                self.pitch_limit_deg = 30.0  # Pitch角度限制
                self.pitch_guard_deg = 25.0  # Pitch保护角度
                self.yaw_tol_deg = 10.0  # Yaw容差
                self.pitch_tol_deg = 5.0  # Pitch容差
                self.roll_tol_deg = 10.0  # Roll容差
                self.heading_tol_deg = 15.0  # 航向容差

                # 导航参数
                self.final_tol = 0.5  # 最终位置容差（米）
                self.decel_radius = 2.0  # 减速半径（米）

                # 超时参数
                self.max_mission_time = 300.0  # 任务最长时间（秒）
                self.max_align_time = 30.0  # 姿态对齐最长时间（秒）

                # 能量优化参数
                self.energy_efficient = False  # 是否启用节能模式
                self.min_thrust = 1.0  # 最小推力阈值

        self.uuv_args = UUVArgs()
        self.uuv = None  # 延迟初始化，在进入水中时创建

        self.uav = UAVController()

        # Subscribe to robot pose
        rospy.Subscriber("/nezha_husky/ground_truth/pose", Pose, self.pose_callback)

        # State variables
        self.current_pose = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.start_x = None
        self.start_y = None
        self.water_entry_x = None
        self.water_entry_y = None

        # Mission parameters
        self.ugv_target_distance = 10.0
        self.uuv_target_depth = -0.5
        self.uuv_forward_distance = 5.0
        self.uav_takeoff_height = 2.0
        self.water_threshold = -0.1

        # State machine
        self.state = 0
        self.uuv_mission_complete = False

        rospy.loginfo("Combined Robot Controller initialized")

    def pose_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg
        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            rospy.loginfo("Start position: x=%.2f, y=%.2f", self.start_x, self.start_y)

    def get_distance_traveled(self):
        """Calculate distance from start"""
        if self.start_x is None:
            return 0.0
        return calculate_distance(self.start_x, self.start_y,
                                  self.current_x, self.current_y)

    def state_ugv(self):
        """State 0: UGV drives forward"""
        distance = self.get_distance_traveled()

        # Check if entered water
        if self.current_z < self.water_threshold:
            rospy.loginfo("Entered water at z=%.2f. Switching to UUV mode", self.current_z)
            self.ugv.stop()
            self.state = 1
            return

        # Check if reached target
        if distance >= self.ugv_target_distance:
            rospy.loginfo("UGV reached %.2f meters", distance)
            self.ugv.stop()
            self.state = 1
            return

        # Continue driving
        self.ugv.drive_forward()
        rospy.loginfo_throttle(1.0, "UGV: %.2f/%.2f m, z=%.2f",
                               distance, self.ugv_target_distance, self.current_z)

    import threading

    def state_uuv(self):
        """State 1: UUV navigates underwater using advanced PID control"""

        # 首次进入UUV状态，初始化控制器
        if self.uuv is None:
            # 记录入水点
            self.water_entry_x = self.current_x
            self.water_entry_y = self.current_y

            # 设置目标航点
            self.uuv_args.wp_x = self.water_entry_x + self.uuv_forward_distance
            self.uuv_args.wp_y = self.water_entry_y
            self.uuv_args.wp_z = self.uuv_target_depth

            rospy.loginfo("Initializing UUV controller...")
            rospy.loginfo("Water entry: (%.2f, %.2f, %.2f)",
                          self.water_entry_x, self.water_entry_y, self.current_z)
            rospy.loginfo("UUV target: (%.2f, %.2f, %.2f)",
                          self.uuv_args.wp_x, self.uuv_args.wp_y, self.uuv_args.wp_z)

            # 创建UUV控制器
            self.uuv = UnderwaterNavigation(self.uuv_args)

            # 在新线程中运行UUV控制器
            self.uuv_thread = threading.Thread(target=self.uuv.run)
            self.uuv_thread.daemon = True
            self.uuv_thread.start()

            rospy.loginfo("UUV controller started in background thread")
            return

        # 检查UUV是否完成任务
        if self.uuv.state == 3:  # WPState.DONE
            rospy.loginfo("UUV mission complete!")
            rospy.loginfo("Final position: (%.2f, %.2f, %.2f)",
                          self.current_x, self.current_y, self.current_z)

            # 等待线程结束
            if self.uuv_thread.is_alive():
                rospy.loginfo("Waiting for UUV thread to finish...")
                self.uuv_thread.join(timeout=2.0)

            self.state = 2
        else:
            # 显示进度
            if hasattr(self.uuv, 'current_pose') and self.uuv.current_pose:
                pos = self.uuv.current_pose.position
                dist = math.sqrt(
                    (self.uuv.wp_x - pos.x) ** 2 +
                    (self.uuv.wp_y - pos.y) ** 2 +
                    (self.uuv.wp_z - pos.z) ** 2
                )
                state_names = ["ALIGN_ATT", "APPROACH", "DECEL", "DONE"]
                rospy.loginfo_throttle(2.0,
                                       "UUV: pos=(%.2f, %.2f, %.2f), dist=%.2f m, state=%s",
                                       pos.x, pos.y, pos.z, dist, state_names[self.uuv.state])

    def state_uav_takeoff(self):
        """State 2: UAV takes off"""
        # Check if reached altitude
        if self.current_z >= self.uav_takeoff_height * 0.95:
            rospy.loginfo("Reached altitude z=%.2f. Returning to origin", self.current_z)
            self.state = 3
            return

        # Command takeoff
        self.uav.takeoff(self.uav_takeoff_height, duration=3.0)
        rospy.loginfo_throttle(1.0, "UAV: Taking off z=%.2f (target: %.2f)",
                               self.current_z, self.uav_takeoff_height)

    def state_uav_return(self):
        """State 3: UAV returns to origin"""
        distance = self.get_distance_traveled()

        # 检查是否到达原点附近
        if distance < 0.5:
            rospy.loginfo("Reached origin (distance: %.2f m). Preparing to land", distance)
            self.state = 4  # 切换到降落状态
            return

        # 计算相对于原点的位置
        dx = self.start_x - self.current_x
        dy = self.start_y - self.current_y

        # 命令返回原点（保持当前高度）
        self.uav.go_to_position(dx, dy, self.uav_takeoff_height, duration=5.0)
        rospy.loginfo_throttle(1.0, "UAV: Returning to origin, distance=%.2f m", distance)

    def state_uav_land(self):
        """State 4: UAV lands at origin"""
        # 检查是否已经降落（接近地面）
        if self.current_z <= 0.2:  # 距离地面20cm以内认为已降落
            rospy.loginfo("UAV landed successfully at z=%.2f m. Mission complete!", self.current_z)
            self.state = 5  # 切换到任务完成状态
            return

        # 命令降落到原点
        self.uav.land(duration=3.0)
        rospy.loginfo_throttle(1.0, "UAV: Landing... altitude=%.2f m", self.current_z)

    def run(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rate.sleep()
                continue

            if self.state == 0:
                self.state_ugv()
            elif self.state == 1:
                self.state_uuv()
                # UUV控制器内部有自己的循环，这里只需要调用一次
                # 如果任务完成，会自动切换状态
            elif self.state == 2:
                self.state_uav_takeoff()
            elif self.state == 3:
                self.state_uav_return()
            elif self.state == 4:
                self.state_uav_land()
            elif self.state == 5:
                rospy.loginfo("Mission completed!")
                break

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("combined_robot_controller")
    controller = CombinedRobotController()
    controller.run()
