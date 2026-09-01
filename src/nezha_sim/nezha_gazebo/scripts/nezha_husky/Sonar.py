#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
import rospy
import open3d as o3d
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import tf
import threading
import time


class PointCloudVisualizer:
    def __init__(self):
        rospy.loginfo("Initializing visualizer...")

        # Open3D visualization
        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(window_name="Nezha Husky 3D SLAM", width=1280, height=960)
            rospy.loginfo("✓ Open3D window created")
        except Exception as e:
            rospy.logerr(f"✗ Failed to create Open3D window: {e}")
            raise

        # Map point cloud
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        # Current scan point cloud (red)
        self.current_scan_pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.current_scan_pcd)

        # Trajectory line
        self.trajectory_line = o3d.geometry.LineSet()
        self.vis.add_geometry(self.trajectory_line)

        # UGV model
        self.ugv_model = self.create_ugv_model()
        self.vis.add_geometry(self.ugv_model)

        # Coordinate frame
        self.coordinate_frame_size = 0.5
        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=self.coordinate_frame_size,
            origin=[0, 0, 0]
        )
        self.vis.add_geometry(self.coordinate_frame)
        rospy.loginfo(f"✓ Geometries added (coordinate frame size: {self.coordinate_frame_size}m)")

        # Set rendering options
        opt = self.vis.get_render_option()
        opt.point_size = 3.0
        opt.background_color = np.asarray([0.05, 0.05, 0.1])
        opt.line_width = 3.0

        self.first_update = True
        self.msg_count = 0
        self.scan_received = False
        self.tf_received = False

        # ===== Point cloud parameters =====
        self.scan_skip_points = 1
        self.frame_skip = 1
        self.use_voxel_downsample = False
        self.voxel_size = 0.05
        self.accumulate_history = True
        self.max_history_points = 500000
        self.history_points = []
        self.history_colors = []
        self.history_timestamps = []
        self.use_time_decay = False
        self.max_point_age = 30.0

        # ===== Range filtering parameters =====
        self.max_valid_range = 5.0
        self.min_valid_range = 0.1

        # ===== Camera-follow parameters - now follow only the lookat point =====
        self.camera_follow_lookat = True  # Follow only the lookat point, do not control the camera position
        self.camera_smooth = 0.15  # Smoothing factor; smaller means smoother

        # Camera state
        self.camera_lookat_target = np.array([0.0, 0.0, 0.0])  # Target lookat point
        self.camera_lookat_current = np.array([0.0, 0.0, 0.0])  # Current lookat point (after smoothing)

        # Record the previous camera parameters to preserve the user's viewpoint
        self.last_camera_params = None

        # ===== Trajectory data =====
        self.trajectory_poses = []
        self.max_trajectory_points = 10000

        # Current pose
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = [0, 0, 0, 1]
        self.previous_position = np.array([0.0, 0.0, 0.0])

        # Lock for thread safety
        self.lock = threading.Lock()
        self.need_update = False
        self.need_camera_update = False

        # TF listener
        try:
            self.tf_listener = tf.TransformListener()
            rospy.loginfo("✓ TF listener created")
            time.sleep(1.0)
        except Exception as e:
            rospy.logerr(f"✗ Failed to create TF listener: {e}")

        self.base_frame = "base_link"
        self.world_frame = "odom"

        # 路径发布
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.world_frame

        rospy.loginfo("=" * 60)
        rospy.loginfo("3D SLAM Visualizer Initialized")
        rospy.loginfo(f"Waiting for laser scan on: /nezha_husky/sonar")
        rospy.loginfo(f"TF frames: {self.world_frame} -> {self.base_frame}")
        rospy.loginfo(f"Valid range: [{self.min_valid_range}, {self.max_valid_range}] meters")
        rospy.loginfo(f"Camera follow lookat: {self.camera_follow_lookat}")
        rospy.loginfo(f"Coordinate frame size: {self.coordinate_frame_size}m")
        rospy.loginfo("=" * 60)
        rospy.loginfo("💡 Tips: Use mouse to rotate/zoom view freely!")
        rospy.loginfo("         The view will follow the robot automatically.")

    def create_ugv_model(self):
        """创建UGV模型"""
        try:
            scale = 0.3

            body = o3d.geometry.TriangleMesh.create_box(
                width=0.6 * scale,
                height=0.4 * scale,
                depth=0.3 * scale
            )
            body.translate([-0.3 * scale, -0.2 * scale, -0.15 * scale])

            arrow = o3d.geometry.TriangleMesh.create_cone(
                radius=0.15 * scale,
                height=0.4 * scale
            )


            ugv = body + arrow
            ugv.paint_uniform_color([1.0, 0.5, 0.0])
            ugv.compute_vertex_normals()

            return ugv
        except Exception as e:
            rospy.logerr(f"Failed to create UGV model: {e}")
            return o3d.geometry.TriangleMesh()

    def get_transform(self):
        """获取从传感器坐标系到世界坐标系的变换"""
        try:
            if not self.tf_listener.canTransform(self.world_frame, self.base_frame, rospy.Time(0)):
                if not self.tf_received:
                    rospy.logwarn(f"Waiting for TF: {self.world_frame} -> {self.base_frame}")
                return None, None, None

            self.tf_listener.waitForTransform(
                self.world_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.1)
            )

            (trans, rot) = self.tf_listener.lookupTransform(
                self.world_frame,
                self.base_frame,
                rospy.Time(0)
            )

            if not self.tf_received:
                rospy.loginfo(f"✓ TF received: pos={trans}, rot={rot}")
                self.tf_received = True

            transform = tf.transformations.quaternion_matrix(rot)
            transform[0:3, 3] = trans

            return transform, trans, rot

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            if self.msg_count % 50 == 1:
                rospy.logwarn(f"TF lookup failed: {e}")
            return None, None, None

    def callback(self, msg):
        """激光扫描回调"""
        if not self.scan_received:
            rospy.loginfo(f"✓ First scan received: {len(msg.ranges)} points")
            rospy.loginfo(f"  Range: [{msg.range_min}, {msg.range_max}]")
            rospy.loginfo(f"  Angle: [{msg.angle_min}, {msg.angle_max}]")
            self.scan_received = True

        self.msg_count += 1

        if self.msg_count % self.frame_skip != 0:
            return

        points = []
        angle = msg.angle_min
        valid_count = 0
        filtered_count = 0

        for i, r in enumerate(msg.ranges):
            if i % self.scan_skip_points != 0:
                angle += msg.angle_increment
                continue

            if math.isnan(r) or math.isinf(r):
                angle += msg.angle_increment
                continue

            if r < self.min_valid_range or r > self.max_valid_range:
                filtered_count += 1
                angle += msg.angle_increment
                continue

            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0

            points.append([x, y, z])
            valid_count += 1
            angle += msg.angle_increment

        if len(points) == 0:
            if self.msg_count % 50 == 1:
                rospy.logwarn(f"No valid points in scan (filtered: {filtered_count})")
            return

        if self.msg_count == self.frame_skip:
            rospy.loginfo(f"✓ Extracted {valid_count} valid points (filtered: {filtered_count})")

        points_array = np.array(points)

        transform, trans, rot = self.get_transform()
        if transform is None:
            return

        with self.lock:
            self.previous_position = self.current_position.copy()
            self.current_position = np.array(trans)
            self.current_orientation = rot

            if self.accumulate_history:
                points_homogeneous = np.hstack([points_array, np.ones((points_array.shape[0], 1))])
                points_world = (transform @ points_homogeneous.T).T[:, :3]

                current_time = rospy.Time.now().to_sec()

                colors = self.compute_colors_by_height(points_world[:, 2])

                self.history_points.extend(points_world.tolist())
                self.history_colors.extend(colors.tolist())
                self.history_timestamps.extend([current_time] * len(points_world))

                if len(self.history_points) > self.max_history_points:
                    excess = len(self.history_points) - self.max_history_points
                    self.history_points = self.history_points[excess:]
                    self.history_colors = self.history_colors[excess:]
                    self.history_timestamps = self.history_timestamps[excess:]

                self.trajectory_poses.append(trans)
                if len(self.trajectory_poses) > self.max_trajectory_points:
                    self.trajectory_poses = self.trajectory_poses[-self.max_trajectory_points:]

                self.current_scan_points = points_world.copy()

                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = self.world_frame
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose.position.x = trans[0]
                pose_stamped.pose.position.y = trans[1]
                pose_stamped.pose.position.z = trans[2]
                pose_stamped.pose.orientation.x = rot[0]
                pose_stamped.pose.orientation.y = rot[1]
                pose_stamped.pose.orientation.z = rot[2]
                pose_stamped.pose.orientation.w = rot[3]
                self.path_msg.poses.append(pose_stamped)

                self.need_update = True
                self.need_camera_update = True

                if self.msg_count % 30 == 0:
                    rospy.loginfo(f"Map: {len(self.history_points)} points, Path: {len(self.trajectory_poses)} poses")
            else:
                self.history_points = points_array.tolist()
                self.need_update = True

    def compute_colors_by_height(self, z_values):
        """根据高度计算颜色"""
        colors = np.zeros((len(z_values), 3))

        if len(z_values) > 0:
            z_min, z_max = z_values.min(), z_values.max()
            if z_max > z_min:
                normalized = (z_values - z_min) / (z_max - z_min)
            else:
                normalized = np.ones_like(z_values) * 0.5

            colors[:, 0] = np.clip(normalized * 2, 0, 1)
            colors[:, 1] = np.clip(2 - np.abs(normalized * 2 - 1) * 2, 0, 1)
            colors[:, 2] = np.clip((1 - normalized) * 2, 0, 1)

        return colors

    def update_camera_lookat(self, robot_pos):
        """只更新相机的lookat点，保持用户的视角方向和距离"""
        if not self.camera_follow_lookat:
            return

        try:
            ctr = self.vis.get_view_control()

            # 获取当前相机参数
            cam_params = ctr.convert_to_pinhole_camera_parameters()

            # 更新目标lookat点（机器人位置稍微抬高一点）
            self.camera_lookat_target = robot_pos + np.array([0, 0, 0.3])

            # 平滑过渡lookat点
            if self.last_camera_params is None:
                # 第一次，直接设置
                self.camera_lookat_current = self.camera_lookat_target.copy()
            else:
                # 平滑插值
                self.camera_lookat_current = (
                        self.camera_smooth * self.camera_lookat_target +
                        (1 - self.camera_smooth) * self.camera_lookat_current
                )

            # 获取当前相机位置
            extrinsic = cam_params.extrinsic
            camera_pos = -extrinsic[:3, :3].T @ extrinsic[:3, 3]

            # 计算当前相机到旧lookat点的向量
            if self.last_camera_params is not None:
                old_extrinsic = self.last_camera_params.extrinsic
                old_lookat = np.array([0, 0, 0])  # 这个会被更新

                # 计算相机到lookat的距离和方向
                view_direction = self.camera_lookat_current - camera_pos
                distance = np.linalg.norm(view_direction)

                # 保持相同的距离和相对方向，但lookat点移动到新位置
                new_camera_pos = self.camera_lookat_current - (view_direction / distance) * distance

                # 更新相机位置，使其相对于新lookat点保持相同的位置关系
                # 计算位移
                lookat_delta = self.camera_lookat_current - camera_pos + (camera_pos - self.camera_lookat_current)

                # 只平移相机，保持方向
                new_extrinsic = extrinsic.copy()
                # 计算新的相机位置，使其保持相对lookat的距离和角度
                offset = camera_pos - self.camera_lookat_current + (self.camera_lookat_current - camera_pos)
                new_camera_position = self.camera_lookat_current + (camera_pos - self.camera_lookat_current)
                new_extrinsic[:3, 3] = -extrinsic[:3, :3] @ new_camera_position

                cam_params.extrinsic = new_extrinsic

            # 保存当前参数
            self.last_camera_params = cam_params

            # 应用新的相机参数
            ctr.convert_from_pinhole_camera_parameters(cam_params)

            # 使用set_lookat来更新观察中心
            ctr.set_lookat(self.camera_lookat_current.tolist())

        except Exception as e:
            if self.msg_count % 100 == 1:
                rospy.logwarn(f"Failed to update camera lookat: {e}")

    def update_visualization(self):
        """在主线程中更新可视化"""
        with self.lock:
            if not self.need_update:
                return

            map_points = np.array(self.history_points) if self.history_points else np.zeros((0, 3))
            map_colors = np.array(self.history_colors) if self.history_colors else np.zeros((0, 3))
            trajectory = np.array(self.trajectory_poses) if self.trajectory_poses else np.zeros((0, 3))
            current_scan = self.current_scan_points.copy() if hasattr(self, 'current_scan_points') else None
            current_pos = self.current_position.copy()
            current_ori = self.current_orientation
            need_cam_update = self.need_camera_update

            self.need_update = False
            self.need_camera_update = False

        if self.msg_count == self.frame_skip:
            rospy.loginfo(f"First update: {map_points.shape[0]} map points")

        # 更新地图点云
        if map_points.shape[0] > 0:
            self.pcd.points = o3d.utility.Vector3dVector(map_points)
            self.pcd.colors = o3d.utility.Vector3dVector(map_colors)
            self.vis.update_geometry(self.pcd)

        # 更新当前扫描
        if current_scan is not None and current_scan.shape[0] > 0:
            self.current_scan_pcd.points = o3d.utility.Vector3dVector(current_scan)
            scan_colors = np.tile([1, 0, 0], (current_scan.shape[0], 1))
            self.current_scan_pcd.colors = o3d.utility.Vector3dVector(scan_colors)
            self.vis.update_geometry(self.current_scan_pcd)

        # 更新轨迹线
        if trajectory.shape[0] > 1:
            lines = [[i, i + 1] for i in range(len(trajectory) - 1)]
            line_colors = [[0, 1, 0] for _ in range(len(lines))]

            self.trajectory_line.points = o3d.utility.Vector3dVector(trajectory)
            self.trajectory_line.lines = o3d.utility.Vector2iVector(lines)
            self.trajectory_line.colors = o3d.utility.Vector3dVector(line_colors)
            self.vis.update_geometry(self.trajectory_line)

        # 更新坐标轴
        try:
            self.vis.remove_geometry(self.coordinate_frame, reset_bounding_box=False)

            self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                size=self.coordinate_frame_size,
                origin=[0, 0, 0]
            )

            transform = tf.transformations.quaternion_matrix(current_ori)
            transform[0:3, 3] = current_pos
            self.coordinate_frame.transform(transform)

            self.vis.add_geometry(self.coordinate_frame, reset_bounding_box=False)
        except Exception as e:
            if self.msg_count % 100 == 1:
                rospy.logwarn(f"Failed to update coordinate frame: {e}")

        # 更新UGV模型
        try:
            self.vis.remove_geometry(self.ugv_model, reset_bounding_box=False)
            self.ugv_model = self.create_ugv_model()
            transform = tf.transformations.quaternion_matrix(current_ori)
            transform[0:3, 3] = current_pos
            self.ugv_model.transform(transform)
            self.vis.add_geometry(self.ugv_model, reset_bounding_box=False)
        except Exception as e:
            if self.msg_count % 100 == 1:
                rospy.logwarn(f"Failed to update UGV model: {e}")

        # ===== 更新相机lookat点（不改变视角方向） =====
        if need_cam_update:
            self.update_camera_lookat(current_pos)

        # 首次更新
        if self.first_update and map_points.shape[0] > 0:
            self.vis.reset_view_point(True)
            self.camera_lookat_current = current_pos.copy()
            self.camera_lookat_target = current_pos.copy()
            self.first_update = False
            rospy.loginfo("✓ View point initialized")
            rospy.loginfo("🖱️  You can now freely rotate and zoom the view!")

        # 发布路径
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)


def main():
    rospy.init_node('pointcloud_visualizer', anonymous=True)

    rospy.loginfo("Starting 3D SLAM Visualizer...")

    visualizer = PointCloudVisualizer()

    rospy.Subscriber('/nezha_husky/sonar', LaserScan, visualizer.callback, queue_size=1)

    rospy.loginfo("Subscribed to /nezha_husky/sonar")
    rospy.loginfo("Waiting for messages...")

    rate = rospy.Rate(30)
    loop_count = 0

    while not rospy.is_shutdown():
        try:
            visualizer.update_visualization()

            if not visualizer.vis.poll_events():
                rospy.loginfo("Window closed by user")
                break
            visualizer.vis.update_renderer()

            loop_count += 1
            if loop_count == 1:
                rospy.loginfo("✓ Visualization loop running")

            rate.sleep()

        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")
            import traceback
            traceback.print_exc()
            break

    visualizer.vis.destroy_window()
    rospy.loginfo("Visualizer stopped")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Fatal error: {e}")
        import traceback

        traceback.print_exc()
