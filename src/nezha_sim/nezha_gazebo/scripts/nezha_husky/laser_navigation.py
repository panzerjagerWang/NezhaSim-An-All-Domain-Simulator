#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import open3d as o3d
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import threading


class PointCloudVisualizer:
    """点云可视化器 - 在独立线程中运行"""

    def __init__(self):
        self.vis = None
        self.pcd = None
        self.first_update = True
        self.points_buffer = None
        self.lock = threading.Lock()
        self.running = True
        self.initialized = False

        # 启动可视化线程
        self.vis_thread = threading.Thread(target=self.visualization_loop)
        self.vis_thread.daemon = True
        self.vis_thread.start()

    def callback(self, msg):
        """ROS回调 - 只负责数据提取"""
        points = []
        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            points.append([point[0], point[1], point[2]])

        if len(points) > 0:
            with self.lock:
                self.points_buffer = np.array(points)
            rospy.loginfo_throttle(10, f"接收到点云: {len(points)} 个点")

    def visualization_loop(self):
        """可视化循环 - 在独立线程中运行"""
        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name="Nezha Husky Points", width=1024, height=768)
            self.pcd = o3d.geometry.PointCloud()
            self.vis.add_geometry(self.pcd)

            opt = self.vis.get_render_option()
            opt.point_size = 2.0
            opt.background_color = np.asarray([0, 0, 0])

            self.initialized = True
            rospy.loginfo("✓ 点云可视化窗口已创建")

            while self.running:
                with self.lock:
                    if self.points_buffer is not None:
                        points_array = self.points_buffer.copy()
                        self.points_buffer = None
                    else:
                        points_array = None

                if points_array is not None and len(points_array) > 0:
                    self.pcd.points = o3d.utility.Vector3dVector(points_array)

                    colors = np.zeros((points_array.shape[0], 3))
                    z_min, z_max = points_array[:, 2].min(), points_array[:, 2].max()
                    if z_max - z_min > 1e-6:
                        z_normalized = (points_array[:, 2] - z_min) / (z_max - z_min)
                        colors[:, 0] = z_normalized
                        colors[:, 2] = 1 - z_normalized
                    else:
                        colors[:, 1] = 1.0
                    self.pcd.colors = o3d.utility.Vector3dVector(colors)

                    self.vis.update_geometry(self.pcd)
                    if self.first_update:
                        self.vis.reset_view_point(True)
                        self.first_update = False

                self.vis.poll_events()
                self.vis.update_renderer()

        except Exception as e:
            rospy.logwarn(f"点云可视化错误: {e}")
        finally:
            if self.vis:
                self.vis.destroy_window()

    def close(self):
        """关闭可视化器"""
        self.running = False
        if self.vis_thread.is_alive():
            self.vis_thread.join(timeout=2.0)


class LaserObstacleAvoidanceNavigator:
    def __init__(self):
        rospy.init_node('laser_navigation_node', anonymous=True)

        # 目标点
        self.goal_x = rospy.get_param('~goal_x', 10.0)
        self.goal_y = rospy.get_param('~goal_y', 0.0)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.0)

        # 当前位置和朝向
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self._odom_received = False
        self._laser_received = False
        self._pointcloud_received = False

        # 激光/点云数据
        self.laser_ranges = []
        self.min_obstacle_distance = float('inf')
        self.obstacle_angle = 0.0

        # 控制参数
        self.max_linear_speed = 0.8
        self.max_angular_speed = 1.2
        self.obstacle_threshold = 2.5  # 提高检测距离
        self.safe_distance = 1.8
        self.critical_distance = 1.0  # 提高临界距离
        self.avoidance_gain = 2.0
        self.min_turn_radius = 3.0

        # 点云/激光过滤参数 - 关键!
        self.min_obstacle_distance_filter = 1.0  # 忽略1m以内的点(机器人自身)
        self.max_obstacle_distance_filter = 10.0  # 忽略10m以外的点
        self.min_obstacle_z = -0.3
        self.max_obstacle_z = 1.5

        # 初始化点云可视化器
        self.visualizer = PointCloudVisualizer()

        # 发布器
        self.cmd_vel_pub = rospy.Publisher('/nezha_husky/cmd_vel', Twist, queue_size=10)
        rospy.loginfo("✓ 速度命令发布到: /nezha_husky/cmd_vel")

        # 订阅点云数据
        rospy.Subscriber('/nezha_husky/points', PointCloud2, self.pointcloud_callback)
        rospy.loginfo("✓ 订阅点云话题: /nezha_husky/points")

        # 尝试订阅激光扫描数据
        laser_topic = rospy.get_param('~laser_topic', '/nezha_husky/scan')
        try:
            rospy.wait_for_message(laser_topic, LaserScan, timeout=2.0)
            rospy.Subscriber(laser_topic, LaserScan, self.laser_callback)
            rospy.loginfo(f"✓ 订阅激光话题: {laser_topic}")
        except:
            rospy.logwarn(f"⚠️  激光话题 {laser_topic} 不可用,将使用点云数据进行避障")

        # 订阅里程计数据 - 修复!尝试多个可能的话题
        odom_topics = [
            '/nezha_husky/odometry_sensor1/odometry',  # 首选
            '/nezha_husky/odometry/filtered',
            '/nezha_husky/husky_velocity_controller/odom',
            '/nezha_husky/ground_truth/odometry'
        ]

        odom_subscribed = False
        for odom_topic in odom_topics:
            try:
                rospy.loginfo(f"尝试订阅里程计话题: {odom_topic}")
                rospy.wait_for_message(odom_topic, Odometry, timeout=2.0)
                rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
                rospy.loginfo(f"✓ 成功订阅里程计话题: {odom_topic}")
                odom_subscribed = True
                break
            except:
                rospy.logwarn(f"⚠️  话题 {odom_topic} 不可用")
                continue

        if not odom_subscribed:
            # 尝试订阅PoseStamped类型
            try:
                pose_topic = '/nezha_husky/odometry_sensor1/pose'
                rospy.loginfo(f"尝试订阅位姿话题: {pose_topic}")
                rospy.wait_for_message(pose_topic, PoseStamped, timeout=2.0)
                rospy.Subscriber(pose_topic, PoseStamped, self.pose_callback)
                rospy.loginfo(f"✓ 成功订阅位姿话题: {pose_topic}")
                odom_subscribed = True
            except:
                rospy.logerr("✗ 无法找到任何可用的里程计/位姿话题!")

        self.rate = rospy.Rate(10)

        rospy.loginfo("=" * 60)
        rospy.loginfo("激光避障导航器已初始化")
        rospy.loginfo(f"目标点: ({self.goal_x}, {self.goal_y})")
        rospy.loginfo(f"障碍物过滤: {self.min_obstacle_distance_filter}m < 距离 < {self.max_obstacle_distance_filter}m")
        rospy.loginfo(f"高度过滤: {self.min_obstacle_z}m < Z < {self.max_obstacle_z}m")
        rospy.loginfo("=" * 60)

        # 等待传感器数据
        self.wait_for_sensors()

    def pose_callback(self, msg):
        """处理PoseStamped消息"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y

        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        if not self._odom_received:
            self._odom_received = True
            rospy.loginfo(f"✓ 首次接收到位姿数据")
            rospy.loginfo(f"  位置: ({self.current_x:.2f}, {self.current_y:.2f})")
            rospy.loginfo(f"  朝向: {math.degrees(yaw):.1f}°")

    def odom_callback(self, msg):
        """处理Odometry消息"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

        if not self._odom_received:
            self._odom_received = True
            rospy.loginfo(f"✓ 首次接收到里程计数据")
            rospy.loginfo(f"  位置: ({self.current_x:.2f}, {self.current_y:.2f})")
            rospy.loginfo(f"  朝向: {math.degrees(yaw):.1f}°")

    def pointcloud_callback(self, msg):
        """处理点云数据"""
        self.visualizer.callback(msg)

        if not self._pointcloud_received:
            self._pointcloud_received = True
            rospy.loginfo("✓ 首次接收到点云数据")

        self.process_pointcloud_for_obstacle(msg)

    def process_pointcloud_for_obstacle(self, msg):
        """从点云数据中提取障碍物信息 - 改进版"""
        min_dist = float('inf')
        obstacle_x = 0.0
        obstacle_y = 0.0
        valid_points = 0
        filtered_by_distance = 0

        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            x, y, z = point[0], point[1], point[2]

            angle = math.atan2(y, x)
            dist = math.sqrt(x ** 2 + y ** 2)

            # 过滤条件
            if abs(angle) < math.pi * 2 / 3:  # 前方120度
                if dist <= self.min_obstacle_distance_filter:
                    filtered_by_distance += 1
                    continue

                if dist > self.max_obstacle_distance_filter:
                    continue

                if not (self.min_obstacle_z < z < self.max_obstacle_z):
                    continue

                valid_points += 1
                if dist < min_dist:
                    min_dist = dist
                    obstacle_x = x
                    obstacle_y = y

        if min_dist < float('inf'):
            self.min_obstacle_distance = min_dist
            self.obstacle_angle = math.atan2(obstacle_y, obstacle_x)
            if not self._laser_received:
                self._laser_received = True

            rospy.loginfo_throttle(5,
                                   f"有效障碍物点: {valid_points}, 最近距离: {min_dist:.2f}m, "
                                   f"过滤点数: {filtered_by_distance}")
        else:
            self.min_obstacle_distance = float('inf')
            self.obstacle_angle = 0.0
            rospy.loginfo_throttle(5,
                                   f"未检测到有效障碍物 (有效点: {valid_points}, 过滤: {filtered_by_distance})")

    def laser_callback(self, msg):
        """处理激光扫描数据"""
        if not self._laser_received:
            self._laser_received = True
            rospy.loginfo(f"✓ 首次接收到激光数据 (共{len(msg.ranges)}个点)")

        self.laser_ranges = msg.ranges
        front_ranges = []
        obstacle_angles = []
        num_readings = len(msg.ranges)

        start_idx = num_readings // 6
        end_idx = 5 * num_readings // 6

        for i in range(start_idx, end_idx):
            # 应用距离过滤
            if (msg.range_min < msg.ranges[i] < msg.range_max and
                    self.min_obstacle_distance_filter < msg.ranges[i] < self.max_obstacle_distance_filter):
                front_ranges.append(msg.ranges[i])
                angle = msg.angle_min + i * msg.angle_increment
                obstacle_angles.append((msg.ranges[i], angle))

        if front_ranges:
            self.min_obstacle_distance = min(front_ranges)
            min_idx = front_ranges.index(self.min_obstacle_distance)
            if min_idx < len(obstacle_angles):
                self.obstacle_angle = obstacle_angles[min_idx][1]
        else:
            self.min_obstacle_distance = float('inf')
            self.obstacle_angle = 0.0

    def wait_for_sensors(self):
        """等待传感器数据"""
        rospy.loginfo("等待传感器数据...")

        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self._odom_received and (self._laser_received or self._pointcloud_received):
                rospy.loginfo("✓ 所有必需传感器数据已接收")
                rospy.loginfo(f"  初始位置: ({self.current_x:.2f}, {self.current_y:.2f})")
                rospy.loginfo(f"  初始朝向: {math.degrees(self.current_yaw):.1f}°")
                rospy.loginfo(f"  障碍物检测: {'激光' if self._laser_received else '点云'}")
                return True

            if not self._odom_received:
                rospy.loginfo_throttle(2, "等待里程计数据...")
            if not self._laser_received and not self._pointcloud_received:
                rospy.loginfo_throttle(2, "等待障碍物检测数据...")

            rospy.sleep(0.1)

        rospy.logerr("✗ 传感器数据接收超时!")
        return False

    def calculate_distance_to_goal(self):
        """计算到目标点的距离"""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        return math.sqrt(dx ** 2 + dy ** 2)

    def calculate_angle_to_goal(self):
        """计算到目标点的角度"""
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        goal_angle = math.atan2(dy, dx)
        angle_diff = goal_angle - self.current_yaw

        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        return angle_diff

    def has_obstacle(self):
        """检测是否有障碍物"""
        return self.min_obstacle_distance < self.obstacle_threshold

    def calculate_avoidance_direction(self):
        """计算避障方向"""
        if self.min_obstacle_distance == float('inf'):
            return 0.0

        avoidance_strength = (self.obstacle_threshold - self.min_obstacle_distance) / self.obstacle_threshold
        avoidance_strength = max(0.0, min(1.0, avoidance_strength))
        avoidance_angular = -math.copysign(1.0, self.obstacle_angle) * self.avoidance_gain * avoidance_strength

        return avoidance_angular

    def navigate(self):
        """主导航循环"""
        rospy.loginfo("🚀 开始导航...")
        rospy.loginfo(f"从 ({self.current_x:.2f}, {self.current_y:.2f}) 到 ({self.goal_x:.2f}, {self.goal_y:.2f})")

        while not rospy.is_shutdown():
            distance_to_goal = self.calculate_distance_to_goal()

            if distance_to_goal < self.goal_tolerance:
                rospy.loginfo("=" * 60)
                rospy.loginfo("🎯 目标已到达!")
                rospy.loginfo(f"最终位置: ({self.current_x:.2f}, {self.current_y:.2f})")
                rospy.loginfo(f"目标位置: ({self.goal_x:.2f}, {self.goal_y:.2f})")
                rospy.loginfo(f"误差: {distance_to_goal:.2f}m")
                rospy.loginfo("=" * 60)
                self.stop_robot()
                break

            angle_to_goal = self.calculate_angle_to_goal()
            cmd = Twist()

            if self.has_obstacle():
                obstacle_severity = "🚨 CRITICAL" if self.min_obstacle_distance < self.critical_distance else "⚠️  WARNING"
                rospy.logwarn_throttle(2,
                                       f"[{obstacle_severity}] 障碍物距离: {self.min_obstacle_distance:.2f}m, "
                                       f"角度: {math.degrees(self.obstacle_angle):.1f}°")

                if self.min_obstacle_distance < self.critical_distance:
                    cmd.linear.x = 0.1
                    avoidance_turn = self.calculate_avoidance_direction()
                    cmd.angular.z = avoidance_turn * self.max_angular_speed

                elif self.min_obstacle_distance < self.safe_distance:
                    cmd.linear.x = 0.25
                    avoidance_turn = self.calculate_avoidance_direction()
                    cmd.angular.z = 0.5 * angle_to_goal + 0.5 * avoidance_turn * self.max_angular_speed

                else:
                    cmd.linear.x = 0.5
                    avoidance_turn = self.calculate_avoidance_direction()
                    cmd.angular.z = 0.7 * angle_to_goal + 0.3 * avoidance_turn * self.max_angular_speed

            else:
                if abs(angle_to_goal) > 0.3:
                    cmd.angular.z = max(-self.max_angular_speed,
                                        min(self.max_angular_speed, 2.0 * angle_to_goal))
                    cmd.linear.x = 0.4
                else:
                    speed_factor = min(1.0, distance_to_goal / self.min_turn_radius)
                    cmd.linear.x = self.max_linear_speed * speed_factor
                    cmd.angular.z = 0.5 * angle_to_goal

            cmd.linear.x = max(0.0, min(self.max_linear_speed, cmd.linear.x))
            cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, cmd.angular.z))

            self.cmd_vel_pub.publish(cmd)

            rospy.loginfo_throttle(1,
                                   f"位置: ({self.current_x:.2f}, {self.current_y:.2f}) | "
                                   f"距离: {distance_to_goal:.2f}m | 角度: {math.degrees(angle_to_goal):.1f}° | "
                                   f"障碍: {self.min_obstacle_distance:.2f}m | "
                                   f"速度: v={cmd.linear.x:.2f} ω={cmd.angular.z:.2f}")

            self.rate.sleep()

    def stop_robot(self):
        """停止机器人"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("🛑 机器人已停止")

    def shutdown(self):
        """关闭导航器"""
        self.stop_robot()
        self.visualizer.close()


def main():
    navigator = None
    try:
        navigator = LaserObstacleAvoidanceNavigator()
        if navigator._odom_received and (navigator._laser_received or navigator._pointcloud_received):
            navigator.navigate()
        else:
            rospy.logerr("传感器初始化失败,退出程序")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航被中断")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断")
    finally:
        if navigator:
            navigator.shutdown()


if __name__ == '__main__':
    main()
