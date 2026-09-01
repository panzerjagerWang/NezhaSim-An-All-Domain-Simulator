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
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry


class FlightController:
    def __init__(self):
        rospy.init_node('flight_controller', anonymous=True)

        # 发布器：发布轨迹命令
        self.traj_pub = rospy.Publisher('/nezha_f/command/trajectory',
                                        MultiDOFJointTrajectory,
                                        queue_size=10)

        # 订阅器：获取当前位置信息
        self.odom_sub = rospy.Subscriber('/nezha_f/ground_truth/odometry',
                                         Odometry,
                                         self.odom_callback)

        # 当前状态
        self.current_position = Vector3(0, 0, 0)
        self.current_velocity = Vector3(0, 0, 0)

        # 航点列表
        self.waypoints = []

        # 第一个航点相关
        self.first_waypoint_position = None
        self.position_error_threshold = 0.2
        self.stability_check_duration = 2.0
        self.stability_position_history = []
        self.max_history_size = 20

        # Epoch控制
        self.max_epochs = 100
        self.current_epoch = 0

        # 飞行状态标志
        self.flight_in_progress = False

        rospy.sleep(1.0)

    def odom_callback(self, msg):
        """里程计回调函数，更新当前位置"""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear

    def load_waypoints(self, csv_file):
        """从CSV文件加载航点"""
        try:
            with open(csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    waypoint = {
                        'index': int(row['index']),
                        'x': float(row['x_m']),
                        'y': float(row['y_m']),
                        'z': float(row['z_m'])
                    }
                    self.waypoints.append(waypoint)
            rospy.loginfo(f"✓ 原始加载 {len(self.waypoints)} 个航点")
            return True
        except Exception as e:
            rospy.logerr(f"✗ 加载CSV文件失败: {e}")
            return False

    def downsample_waypoints(self, target_count=20):
        """将航点数量缩减到目标数量"""
        if len(self.waypoints) <= target_count:
            rospy.loginfo(f"航点数量 ({len(self.waypoints)}) 已小于目标数量 ({target_count})，无需缩减")
            return

        original_count = len(self.waypoints)
        step = len(self.waypoints) / target_count
        selected_indices = [int(i * step) for i in range(target_count)]

        if selected_indices[-1] != len(self.waypoints) - 1:
            selected_indices[-1] = len(self.waypoints) - 1

        downsampled = [self.waypoints[i] for i in selected_indices]

        for i, wp in enumerate(downsampled):
            wp['index'] = i

        self.waypoints = downsampled

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"✓ 航点缩减完成: {original_count} → {len(self.waypoints)} 个")
        rospy.loginfo("=" * 60)

    def calculate_distance_to_first_waypoint(self):
        """计算当前位置到第一个航点的距离"""
        if self.first_waypoint_position is None:
            return float('inf')

        dx = self.current_position.x - self.first_waypoint_position.x
        dy = self.current_position.y - self.first_waypoint_position.y
        dz = self.current_position.z - self.first_waypoint_position.z

        return np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    def update_stability_history(self):
        """更新位置历史记录（用于稳定性检测）"""
        current_pos = Vector3(
            self.current_position.x,
            self.current_position.y,
            self.current_position.z
        )
        self.stability_position_history.append(current_pos)

        if len(self.stability_position_history) > self.max_history_size:
            self.stability_position_history.pop(0)

    def clear_stability_history(self):
        """清空稳定性历史记录"""
        self.stability_position_history = []

    def check_stability_at_first_waypoint(self):
        """检查无人机是否在第一个航点稳定停留"""
        if len(self.stability_position_history) < self.max_history_size:
            return False

        for pos in self.stability_position_history:
            dx = pos.x - self.first_waypoint_position.x
            dy = pos.y - self.first_waypoint_position.y
            dz = pos.z - self.first_waypoint_position.z
            distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            if distance >= self.position_error_threshold:
                return False

        return True

    def wait_for_first_waypoint(self, timeout=5.0):
        """等待无人机到达第一个航点并稳定停留"""
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"等待无人机到达第一个航点（误差阈值: {self.position_error_threshold}m）...")
        rospy.loginfo("=" * 60)

        self.clear_stability_history()

        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        last_log_time = 0
        reached_waypoint = False

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed > timeout:
                rospy.logwarn(f"等待超时（{timeout}秒），当前误差: {self.calculate_distance_to_first_waypoint():.3f}m")
                return False

            error = self.calculate_distance_to_first_waypoint()

            if elapsed >= last_log_time + 0.5:
                if not reached_waypoint:
                    rospy.loginfo(
                        f"[阶段1/2] 接近中... 当前误差: {error:.3f}m (目标: <{self.position_error_threshold}m)")
                else:
                    stability_progress = len(self.stability_position_history) / self.max_history_size * 100
                    rospy.loginfo(
                        f"[阶段2/2] 稳定性检测中... {stability_progress:.0f}% (需要稳定{self.stability_check_duration}秒)")
                last_log_time = elapsed

            if error < self.position_error_threshold:
                if not reached_waypoint:
                    reached_waypoint = True
                    rospy.loginfo("=" * 60)
                    rospy.loginfo(f"✓ 已到达第一个航点附近！误差: {error:.4f}m")
                    rospy.loginfo(
                        f"开始稳定性检测（需要连续{self.stability_check_duration}秒保持在{self.position_error_threshold}m内）...")
                    rospy.loginfo("=" * 60)

                self.update_stability_history()

                if self.check_stability_at_first_waypoint():
                    rospy.loginfo("=" * 60)
                    rospy.loginfo(f"✓ 稳定性检测通过！已在第一个航点稳定停留{self.stability_check_duration}秒")
                    rospy.loginfo(f"   最终误差: {error:.4f}m")
                    rospy.loginfo("=" * 60)
                    return True
            else:
                if reached_waypoint:
                    rospy.logwarn(
                        f"⚠ 位置偏离！误差: {error:.3f}m > {self.position_error_threshold}m，重新开始稳定性检测")
                    self.clear_stability_history()
                    reached_waypoint = False

            rate.sleep()

        return False

    def send_to_first_waypoint(self):
        """发送命令让无人机返回第一个航点"""
        if self.first_waypoint_position is None:
            rospy.logwarn("第一个航点位置未设置，无法返回")
            return

        rospy.loginfo("=" * 60)
        rospy.loginfo("发送返回第一个航点命令...")
        rospy.loginfo(
            f"目标位置: ({self.first_waypoint_position.x:.3f}, {self.first_waypoint_position.y:.3f}, {self.first_waypoint_position.z:.3f})")
        rospy.loginfo("=" * 60)

        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]

        point = MultiDOFJointTrajectoryPoint()

        transform = Transform()
        transform.translation.x = self.first_waypoint_position.x
        transform.translation.y = self.first_waypoint_position.y
        transform.translation.z = self.first_waypoint_position.z
        transform.rotation = Quaternion(0, 0, 0, 1)
        point.transforms.append(transform)

        velocity = Twist()
        velocity.linear = Vector3(0, 0, 0)
        velocity.angular = Vector3(0, 0, 0)
        point.velocities.append(velocity)

        point.time_from_start = rospy.Duration(5.0)

        traj.points.append(point)
        self.traj_pub.publish(traj)

        rospy.loginfo("返回第一个航点命令已发布")

    def create_smooth_trajectory_msg(self, waypoints, time_per_segment=2.0):
        """创建平滑的多航点轨迹消息"""
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]

        cumulative_time = 0.0

        for i, waypoint in enumerate(waypoints):
            point = MultiDOFJointTrajectoryPoint()

            transform = Transform()
            transform.translation.x = waypoint['x']
            transform.translation.y = waypoint['y']
            transform.translation.z = waypoint['z']
            transform.rotation = Quaternion(0, 0, 0, 1)
            point.transforms.append(transform)

            velocity = Twist()
            if i < len(waypoints) - 1:
                next_wp = waypoints[i + 1]
                dx = next_wp['x'] - waypoint['x']
                dy = next_wp['y'] - waypoint['y']
                dz = next_wp['z'] - waypoint['z']
                distance = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

                speed = min(distance / time_per_segment, 2.0)
                if distance > 0:
                    velocity.linear.x = (dx / distance) * speed
                    velocity.linear.y = (dy / distance) * speed
                    velocity.linear.z = (dz / distance) * speed
            else:
                velocity.linear = Vector3(0, 0, 0)

            velocity.angular = Vector3(0, 0, 0)
            point.velocities.append(velocity)

            cumulative_time += time_per_segment
            point.time_from_start = rospy.Duration(cumulative_time)

            traj.points.append(point)

        return traj, cumulative_time

    def execute_flight(self):
        """执行单次飞行"""
        if not self.waypoints:
            rospy.logerr("没有可用的航点！")
            return 0.0

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"准备飞行 {len(self.waypoints)} 个航点")
        rospy.loginfo("=" * 60)

        time_per_segment = 2.0
        traj_msg, total_time = self.create_smooth_trajectory_msg(self.waypoints, time_per_segment)

        rospy.loginfo(f"预计飞行时间: {total_time:.1f} 秒")

        self.flight_in_progress = True
        self.traj_pub.publish(traj_msg)

        rospy.loginfo("轨迹已发布，无人机开始飞行...")

        # 等待飞行完成
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()

            if elapsed >= total_time + 1.0:
                break

            rate.sleep()

        self.flight_in_progress = False
        rospy.loginfo("=" * 60)
        rospy.loginfo("航点飞行完成！")
        rospy.loginfo("=" * 60)

        return total_time

    def run_multiple_epochs(self, max_epochs=100):
        """执行多个epoch的飞行"""
        if len(self.waypoints) == 0:
            rospy.logerr("没有航点可用！")
            return

        first_wp = self.waypoints[0]
        self.first_waypoint_position = Vector3(
            first_wp['x'],
            first_wp['y'],
            first_wp['z']
        )
        rospy.loginfo(
            f"第一个航点位置已记录: ({self.first_waypoint_position.x:.3f}, {self.first_waypoint_position.y:.3f}, {self.first_waypoint_position.z:.3f})")

        self.max_epochs = max_epochs

        for epoch in range(1, max_epochs + 1):
            self.current_epoch = epoch

            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo(f"开始 Epoch {epoch}/{max_epochs}")
            rospy.loginfo("=" * 60)

            if epoch > 1:
                self.send_to_first_waypoint()
                rospy.sleep(5.0)

                if not self.wait_for_first_waypoint(timeout=5.0):
                    rospy.logwarn(f"Epoch {epoch}: 未能精确返回第一个航点，但继续执行")

                rospy.sleep(2.0)

            # 发布开始飞行信号（供数据记录模块使用）
            rospy.set_param('/flight_controller/epoch', epoch)
            rospy.set_param('/flight_controller/flight_status', 'starting')

            # 执行飞行
            flight_time = self.execute_flight()

            # 发布飞行完成信号
            rospy.set_param('/flight_controller/flight_status', 'completed')
            rospy.set_param('/flight_controller/flight_time', flight_time)

            rospy.sleep(2.0)

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo(f"所有 {max_epochs} 个Epoch完成！")
        rospy.loginfo("=" * 60)


def main():
    try:
        controller = FlightController()

        csv_file = 'waypoints.csv'
        if not controller.load_waypoints(csv_file):
            return

        controller.downsample_waypoints(target_count=20)

        rospy.loginfo("等待系统初始化...")
        rospy.sleep(2.0)

        controller.run_multiple_epochs(max_epochs=100)

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被用户中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
