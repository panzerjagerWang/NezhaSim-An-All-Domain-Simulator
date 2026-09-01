#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
# 假设已初始化rospy、各种消息类型和args等
import math
import time
import numpy as np
import rospy
import argparse
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped
from UUVControllerV8 import *
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from UAVController import *
import transformations as tf
# 1. 水下部分
from copy import deepcopy

def run_underwater_navigation(target_wp, args):
    args.wp_x, args.wp_y, args.wp_z = target_wp
    nav = UnderwaterNavigation(args)
    nav.run()  # 阻塞到到点或超时

def main():
    # 假设 args 是使用 argparse 或命名空间构造好的参数
    # 1. 导航到 [2, 0, -2]
    args = parse_args()

    run_underwater_navigation([1, 0, -1], args)
    rospy.loginfo("已到达 [2, 0, -2]，准备导航到 [3, 0, 0]")

    rospy.sleep(2.0)
    # 2. 导航到 [3, 0, 0]
    run_underwater_navigation([3, 0, 0], args)
    rospy.loginfo("已到达 [3, 0, 0]，准备切换无人机模式")

    rospy.sleep(2.0)
    # 3. 切换无人机控制，导航并起飞到 [3, 0, 2]
    rospy.init_node("nezha_underwater_nav_blocking")

    uav = UAVController(debug=True)
    rospy.sleep(2.0)  # 等待订阅接收到位姿
    timeout = 20
    start_time = rospy.Time.now().to_sec()
    while uav.current_pose is None:
        if rospy.Time.now().to_sec() - start_time > timeout:
            rospy.logerr("无法获取无人机位置信息")
            return
        rospy.loginfo("等待接收无人机位置信息...")


if __name__ == '__main__':
    main()
