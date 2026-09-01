#!/usr/bin/env python
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3


class DataRecorder:
    def __init__(self):
        rospy.init_node('data_recorder', anonymous=True)

        # 订阅器：订阅里程计获取位置和速度信息
        self.odom_sub = rospy.Subscriber('/nezha_f/ground_truth/odometry',
                                         Odometry,
                                         self.odom_callback)

        # 数据存储 - 分别存储x, y, z的位置和速度
        self.position_x = []
        self.position_y = []
        self.position_z = []
        self.velocity_x = []
        self.velocity_y = []
        self.velocity_z = []

        self.max_records = 100
        self.current_position = Vector3(0, 0, 0)
        self.current_velocity = Vector3(0, 0, 0)

        # 记录控制
        self.recording = False
        self.current_epoch = 0
        self.flight_start_time = None
        self.sampling_interval = 0.0

        # 数据记录锁，防止重复记录
        self.first_waypoint_recorded = False
        self.last_waypoint_recorded = False

        rospy.sleep(1.0)

    def odom_callback(self, msg):
        """里程计回调函数，基于时间均匀采样"""
        self.current_position = msg.pose.pose.position
        self.current_velocity = msg.twist.twist.linear

        if self.recording and self.flight_start_time is not None:
            current_time = rospy.Time.now()
            elapsed = (current_time - self.flight_start_time).to_sec()

            # 计算飞行中已采集的点数（不含第一个航点）
            flight_records = len(self.position_x) - 1
            target_flight_records = self.max_records - 2  # 98个

            # 严格限制：只采集98个点
            if flight_records >= target_flight_records:
                return

            expected_index = int(elapsed / self.sampling_interval)

            if flight_records < expected_index:
                self.position_x.append(self.current_position.x)
                self.position_y.append(self.current_position.y)
                self.position_z.append(self.current_position.z)

                self.velocity_x.append(self.current_velocity.x)
                self.velocity_y.append(self.current_velocity.y)
                self.velocity_z.append(self.current_velocity.z)

                flight_records = len(self.position_x) - 1
                if flight_records % 10 == 0:
                    rospy.loginfo(
                        f"✓ 飞行采集: {flight_records}/{target_flight_records} 个数据点 "
                        f"(总计: {len(self.position_x)}/{self.max_records - 1}, 进度: {elapsed:.1f}s)"
                    )

    def record_current_state(self):
        """手动记录当前状态（位置和速度）"""
        self.position_x.append(self.current_position.x)
        self.position_y.append(self.current_position.y)
        self.position_z.append(self.current_position.z)

        self.velocity_x.append(self.current_velocity.x)
        self.velocity_y.append(self.current_velocity.y)
        self.velocity_z.append(self.current_velocity.z)

        rospy.loginfo(f"✓ 手动记录数据点 {len(self.position_x)}/{self.max_records}")

    def start_recording(self, flight_time):
        """开始记录数据"""
        self.recording = True
        self.flight_start_time = rospy.Time.now()

        # 计算采样间隔
        target_records = self.max_records - 2  # 98个点（不含首尾）
        self.sampling_interval = flight_time / target_records

        rospy.loginfo("=" * 60)
        rospy.loginfo(f"开始数据采集")
        rospy.loginfo(f"采样间隔: {self.sampling_interval:.3f} 秒")
        rospy.loginfo(f"目标数据点: {target_records} 个")
        rospy.loginfo("=" * 60)

    def stop_recording(self):
        """停止记录数据"""
        self.recording = False
        rospy.loginfo("数据采集已停止")

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

            self.recording = True
            rate = rospy.Rate(10)
            补充计数 = 0

            while len(self.position_x) < self.max_records and not rospy.is_shutdown():
                rate.sleep()
                补充计数 += 1
                if 补充计数 % 10 == 0:
                    rospy.loginfo(f"补充采集中... {len(self.position_x)}/{self.max_records}")

            self.recording = False
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

    def save_to_txt(self, data_list, filename, mode='a'):
        """将列表数据保存到txt文件，单行格式 [a,b,...,x], 并追加到文件"""
        try:
            with open(filename, mode) as f:
                formatted_data = '[' + ','.join([f"{value:.6f}" for value in data_list]) + '],\n'
                f.write(formatted_data)
            rospy.loginfo(f"✓ {filename} - Epoch {self.current_epoch} (共 {len(data_list)} 个数据点)")
        except Exception as e:
            rospy.logerr(f"✗ 保存文件 {filename} 失败: {e}")

    def save_recorded_data(self):
        """保存记录的数据到6个不同的txt文件（追加模式）"""
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
        self.flight_start_time = None

        # 重置记录标志
        self.first_waypoint_recorded = False
        self.last_waypoint_recorded = False

        rospy.loginfo("数据已重置，准备下一个epoch...")

    def monitor_flight_controller(self):
        """监听飞行控制器的状态，自动记录数据"""
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # 检查飞行状态
            if rospy.has_param('/flight_controller/flight_status'):
                status = rospy.get_param('/flight_controller/flight_status')
                epoch = rospy.get_param('/flight_controller/epoch', 0)

                if epoch != self.current_epoch:
                    # 新的epoch开始
                    if self.current_epoch > 0:
                        # 保存上一个epoch的数据
                        self.ensure_exact_record_count()
                        self.save_recorded_data()
                        self.reset_for_next_epoch()

                    self.current_epoch = epoch
                    rospy.loginfo(f"开始记录 Epoch {epoch}")

                if status == 'starting' and not self.recording:
                    # 记录第一个航点
                    if not self.first_waypoint_recorded:
                        rospy.loginfo("记录第一个航点数据...")
                        self.record_current_state()
                        self.first_waypoint_recorded = True

                    # 获取飞行时间并开始记录
                    flight_time = rospy.get_param('/flight_controller/flight_time', 40.0)
                    self.start_recording(flight_time)

                elif status == 'completed' and self.recording:
                    # 停止记录
                    self.stop_recording()

                    # 记录最后一个航点
                    if not self.last_waypoint_recorded:
                        rospy.loginfo("记录最后一个航点数据...")
                        self.record_current_state()
                        self.last_waypoint_recorded = True

            rate.sleep()


def main():
    try:
        recorder = DataRecorder()

        rospy.loginfo("数据记录器已启动，等待飞行控制器信号...")

        recorder.monitor_flight_controller()

    except rospy.ROSInterruptException:
        rospy.loginfo("数据记录器被用户中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
