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
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class PointCloudVisualizer:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="Nezha Husky Points", width=1024, height=768)
        self.pcd = o3d.geometry.PointCloud()
        self.vis.add_geometry(self.pcd)

        # Configure render options
        opt = self.vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.asarray([0, 0, 0])

        self.first_update = True

    def callback(self, msg):
        # Extract points from the PointCloud2 message
        points = []
        for point in pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")):
            points.append([point[0], point[1], point[2]])

        if len(points) == 0:
            rospy.logwarn("Received empty point cloud")
            return

        # Convert to a numpy array
        points_array = np.array(points)

        # Update the point cloud
        self.pcd.points = o3d.utility.Vector3dVector(points_array)

        # Color points by height
        colors = np.zeros((points_array.shape[0], 3))
        z_normalized = (points_array[:, 2] - points_array[:, 2].min()) / (
                    points_array[:, 2].max() - points_array[:, 2].min() + 1e-6)
        colors[:, 0] = z_normalized  # Red channel
        colors[:, 2] = 1 - z_normalized  # Blue channel
        self.pcd.colors = o3d.utility.Vector3dVector(colors)

        # Update the visualization
        self.vis.update_geometry(self.pcd)
        if self.first_update:
            self.vis.reset_view_point(True)
            self.first_update = False
        self.vis.poll_events()
        self.vis.update_renderer()

        rospy.loginfo(f"Updated point cloud with {len(points)} points")


def main():
    rospy.init_node('pointcloud_visualizer', anonymous=True)

    visualizer = PointCloudVisualizer()

    # Subscribe to the point cloud topic
    rospy.Subscriber('/nezha_husky1/points', PointCloud2, visualizer.callback)

    rospy.loginfo("Point cloud visualizer started. Waiting for data...")

    # Keep the visualization window open
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        visualizer.vis.poll_events()
        visualizer.vis.update_renderer()
        rate.sleep()

    visualizer.vis.destroy_window()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
