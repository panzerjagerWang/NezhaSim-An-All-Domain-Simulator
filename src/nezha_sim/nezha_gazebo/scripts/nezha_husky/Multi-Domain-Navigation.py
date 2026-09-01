#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# -*- coding: utf-8 -*-
"""
Nezha Husky Multi-Domain Navigation Mission
Mission Profile:
1. UGV drives forward 10.5 meters on land
2. UGV falls into water, transitions to UUV control
3. UUV dives to underwater waypoint [12, 0, -2]
4. UUV ascends to surface (z ≈ -0.5m)
5. UAV takes off to 2 meters altitude
6. UAV returns to origin point
"""

import rospy
import math
import time
from copy import deepcopy
from geometry_msgs.msg import Pose
from UGV_controller import UGVController
from UUV_Controller import UnderwaterNavigation, parse_args
from UAV_Controller import UAVController


class MultiDomainMission:
    """Multi-domain cooperative mission controller"""

    def __init__(self):
        rospy.init_node("nezha_multi_domain_mission", anonymous=True)
        rospy.loginfo("=" * 60)
        rospy.loginfo("Nezha Husky multi-domain mission starting")
        rospy.loginfo("=" * 60)

        # Initialize each controller
        self.ugv = None
        self.uuv = None
        self.uav = None

        # Mission parameters
        self.ugv_forward_distance = 12  # UGV forward distance (meters)
        self.underwater_waypoint = [13.0, 0.0, -1.0]  # Underwater waypoint
        self.surface_depth = -1  # Water surface depth
        self.takeoff_height = 2.0  # Takeoff altitude
        self.origin = [0.0, 0.0, 2.0]  # Return-to-origin coordinates

        # State monitoring
        self.current_pose = None
        rospy.Subscriber("/nezha_husky/ground_truth/pose", Pose, self._pose_callback)

        rospy.sleep(1.0)

    def _pose_callback(self, msg):
        """Pose callback function"""
        self.current_pose = msg

    def wait_for_pose(self, timeout=10.0):
        """Wait to receive pose information"""
        rospy.loginfo("Waiting for pose information...")
        start_time = rospy.Time.now().to_sec()

        while self.current_pose is None and not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.logerr("❌ Timed out waiting for pose information")
                return False
            rospy.sleep(0.1)

        rospy.loginfo(f"✓ Current position: x={self.current_pose.position.x:.2f}, "
                      f"y={self.current_pose.position.y:.2f}, "
                      f"z={self.current_pose.position.z:.2f}")
        return True

    def phase_1_ugv_drive(self):
        """Phase 1: UGV drives forward on land to the target position [12, 0, 0]"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("Phase 1: UGV land driving")
        rospy.loginfo("=" * 60)

        # Initialize the UGV controller
        self.ugv = UGVController(namespace="/nezha_husky")

        if not self.wait_for_pose():
            return False

        # Target position
        target_x = 12.0
        target_y = 0.0

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y

        rospy.loginfo(f"Start position: x={start_x:.2f}m, y={start_y:.2f}m")
        rospy.loginfo(f"Target position: x={target_x:.2f}m, y={target_y:.2f}m")

        # Compute the distance to the target
        def get_distance_to_target():
            if self.current_pose is None:
                return float('inf')
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            return math.sqrt(dx ** 2 + dy ** 2)

        # Compute the current heading angle
        def get_current_yaw():
            if self.current_pose is None:
                return 0.0
            orientation = self.current_pose.orientation
            import tf.transformations as tf_trans
            euler = tf_trans.euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            return euler[2]

        # 导航控制参数
        max_linear_speed = 2.0
        max_angular_speed = 1.0

        rate = rospy.Rate(10)
        timeout = 60.0
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rate.sleep()
                continue

            current_time = rospy.Time.now().to_sec()

            # 超时检查
            if current_time - start_time > timeout:
                self.ugv.stop()
                rospy.loginfo("✓ UGV阶段完成（超时）")
                break

            distance = get_distance_to_target()
            current_yaw = get_current_yaw()

            # 简单判定：距离小于0.5米即认为到达
            if distance < 0.5:
                self.ugv.stop()
                rospy.loginfo(f"✓ UGV到达目标位置")
                break

            # 计算目标方向
            dx = target_x - self.current_pose.position.x
            dy = target_y - self.current_pose.position.y
            target_angle = math.atan2(dy, dx)

            # 计算角度误差
            angle_error = target_angle - current_yaw
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            # 计算控制量
            linear_speed = min(max_linear_speed, distance * 0.5)
            angular_speed = max(min(angle_error * 2.0, max_angular_speed), -max_angular_speed)

            if abs(angle_error) > 0.3:
                linear_speed *= 0.3

            self.ugv.drive_and_turn(linear_speed, angular_speed)

            # 实时显示进度
            rospy.loginfo_throttle(2, f"导航中: 距离={distance:.2f}m")

            rate.sleep()

        rospy.sleep(2.0)
        return True

    def phase_2_water_entry(self):
        """阶段2: 检测入水并切换到UUV模式"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("阶段 2: 入水检测与模式切换")
        rospy.loginfo("=" * 60)

        rospy.loginfo("等待入水...")
        rate = rospy.Rate(10)
        timeout = 20.0
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rate.sleep()
                continue

            current_z = self.current_pose.position.z

            # 检测是否入水（z < -0.1）
            if current_z < -0.1:
                rospy.loginfo(f"✓ 检测到入水！当前深度: z={current_z:.2f}m")
                break

            # 超时检查
            if rospy.Time.now().to_sec() - start_time > timeout:
                rospy.loginfo("✓ 入水检测阶段完成（超时）")
                break

            rate.sleep()

        rospy.sleep(1.0)
        return True

    def phase_3_uuv_dive(self):
        """阶段3: UUV下潜到水下航点 - 10秒后自动进入下一阶段"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("阶段 3: UUV 下潜到水下航点（10秒）")
        rospy.loginfo("=" * 60)

        # 准备UUV导航参数
        args = parse_args()

        args.wp_x = self.underwater_waypoint[0]
        args.wp_y = self.underwater_waypoint[1]
        args.wp_z = self.underwater_waypoint[2]

        # 降低推力参数
        args.K_fwd = 10.0
        args.T_fwd_max = 30.0
        args.K_pitch = 8.0
        args.K_yaw = 8.0

        args.final_tol = 0.5
        args.decel_radius = 2.0
        args.max_mission_time = 10.0  # 设置为10秒

        rospy.loginfo(f"水下目标位置: x={args.wp_x:.2f}, y={args.wp_y:.2f}, z={args.wp_z:.2f}")
        rospy.loginfo("执行时间: 10秒")

        # 执行UUV导航
        self.uuv = UnderwaterNavigation(args)
        self.uuv.run()

        rospy.loginfo("✓ UUV下潜阶段完成（10秒）")
        rospy.sleep(1.0)
        return True

    def phase_4_uuv_surface(self):
        """阶段4: UUV上浮到水面 - 10秒后自动进入下一阶段"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("阶段 4: UUV 上浮到水面（10秒）")
        rospy.loginfo("=" * 60)

        # 准备UUV导航参数
        args = parse_args()

        args.wp_x = self.underwater_waypoint[0]
        args.wp_y = 0.0
        args.wp_z = self.surface_depth

        # 上浮阶段使用温和参数
        args.K_fwd = 8.0
        args.T_fwd_max = 25.0
        args.K_pitch = 6.0
        args.K_yaw = 6.0

        args.final_tol = 0.3
        args.decel_radius = 1.5
        args.max_mission_time = 10.0  # 设置为10秒

        rospy.loginfo(f"水面目标位置: x={args.wp_x:.2f}, y={args.wp_y:.2f}, z={args.wp_z:.2f}")
        rospy.loginfo("执行时间: 10秒")

        # 执行UUV导航
        self.uuv = UnderwaterNavigation(args)
        self.uuv.run()

        rospy.loginfo("✓ UUV上浮阶段完成（10秒）")
        rospy.sleep(1.0)
        return True

    def phase_5_uav_takeoff(self):
        """阶段5: UAV起飞 - 10秒后自动进入下一阶段"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("阶段 5: UAV 起飞（10秒）")
        rospy.loginfo("=" * 60)

        # 初始化UAV控制器
        self.uav = UAVController(debug=True)
        rospy.sleep(2.0)

        if not self.wait_for_pose(timeout=20):
            return False

        # 设置目标高度
        self.uav.target_height = self.takeoff_height
        rospy.loginfo(f"目标高度: {self.takeoff_height}m")
        rospy.loginfo("执行时间: 10秒")

        from geometry_msgs.msg import Point

        # 创建起飞目标点
        takeoff_target = Point()
        takeoff_target.x = self.current_pose.position.x
        takeoff_target.y = self.current_pose.position.y
        takeoff_target.z = self.takeoff_height

        rospy.loginfo(
            f"当前高度: {self.current_pose.position.z:.2f}m, "
            f"开始上升到目标高度 {self.takeoff_height:.2f}m..."
        )

        # 发送起飞命令
        self.uav.move_to_waypoint(takeoff_target, duration=5.0)

        # 固定执行10秒
        start_time = rospy.Time.now().to_sec()
        duration = 10.0

        rate = rospy.Rate(10)
        while rospy.Time.now().to_sec() - start_time < duration:
            if self.current_pose:
                current_height = self.current_pose.position.z
                elapsed = rospy.Time.now().to_sec() - start_time

                rospy.loginfo_throttle(
                    1,
                    f"起飞中 [{elapsed:.1f}/{duration:.1f}s]: "
                    f"高度={current_height:.2f}m"
                )

                # 持续发送命令保持高度
                self.uav.move_to_waypoint(takeoff_target, duration=2.0)

            rate.sleep()

        rospy.loginfo("✓ UAV起飞阶段完成（10秒）")
        rospy.sleep(1.0)
        return True

    def phase_6_return_home(self):
        """阶段6: UAV返回原点 - 10秒后自动完成"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("阶段 6: UAV 返回原点（10秒）")
        rospy.loginfo("=" * 60)

        if self.uav is None:
            rospy.logerr("❌ UAV控制器未初始化")
            return False

        from geometry_msgs.msg import Point

        # 设置返回目标
        target = Point()
        target.x = self.origin[0]
        target.y = self.origin[1]
        target.z = self.origin[2]

        rospy.loginfo(f"返回目标: x={target.x:.2f}, "
                      f"y={target.y:.2f}, "
                      f"z={target.z:.2f}")
        rospy.loginfo("执行时间: 10秒")

        # 固定执行10秒
        start_time = rospy.Time.now().to_sec()
        duration = 10.0

        rate = rospy.Rate(10)
        while rospy.Time.now().to_sec() - start_time < duration:
            self.uav.waypoint_navigation(target)

            if self.current_pose:
                distance = math.sqrt(
                    (target.x - self.current_pose.position.x) ** 2 +
                    (target.y - self.current_pose.position.y) ** 2 +
                    (target.z - self.current_pose.position.z) ** 2
                )
                elapsed = rospy.Time.now().to_sec() - start_time
                rospy.loginfo_throttle(
                    1,
                    f"返航中 [{elapsed:.1f}/{duration:.1f}s]: "
                    f"距离原点={distance:.2f}m"
                )

            rate.sleep()

        rospy.loginfo("✓ UAV返航阶段完成（10秒）")
        rospy.sleep(1.0)

        # 着陆
        rospy.loginfo("开始着陆...")
        self.uav.land()
        rospy.loginfo("✓ UAV着陆完成")

        return True

    def run_mission(self):
        """执行完整任务流程"""
        try:
            rospy.loginfo("\n🚀 开始执行多域协同任务\n")

            # 阶段1: UGV陆地行驶
            if not self.phase_1_ugv_drive():
                rospy.logerr("❌ 阶段1失败")
                return False

            # 阶段2: 入水检测
            if not self.phase_2_water_entry():
                rospy.logerr("❌ 阶段2失败")
                return False

            # 阶段3: UUV下潜（10秒）
            if not self.phase_3_uuv_dive():
                rospy.logerr("❌ 阶段3失败")
                return False

            # 阶段4: UUV上浮（10秒）
            if not self.phase_4_uuv_surface():
                rospy.logerr("❌ 阶段4失败")
                return False

            # 阶段5: UAV起飞（10秒）
            if not self.phase_5_uav_takeoff():
                rospy.logerr("❌ 阶段5失败")
                return False

            # 阶段6: 返回原点（10秒）
            if not self.phase_6_return_home():
                rospy.logerr("❌ 阶段6失败")
                return False

            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo("🎉 任务完成！所有阶段执行成功")
            rospy.loginfo("=" * 60 + "\n")
            return True

        except rospy.ROSInterruptException:
            rospy.loginfo("任务被用户中断")
            return False
        except Exception as e:
            rospy.logerr(f"❌ 任务执行异常: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """主函数"""
    try:
        mission = MultiDomainMission()
        success = mission.run_mission()

        if success:
            rospy.loginfo("任务执行成功，节点保持运行...")
            rospy.spin()
        else:
            rospy.logerr("任务执行失败")

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
