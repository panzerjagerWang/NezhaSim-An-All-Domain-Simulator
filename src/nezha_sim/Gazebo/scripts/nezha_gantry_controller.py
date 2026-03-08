#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
力架+无人机组合系统控制器
功能：控制力架抓取、移动和释放无人机
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math


class GantryDroneController:
    """力架无人机组合控制器"""

    def __init__(self):
        rospy.init_node('gantry_drone_controller', anonymous=True)

        # 力架控制发布器
        self.horizontal_pub = rospy.Publisher(
            '/gantry/horizontal_position_controller/command',
            Float64, queue_size=10)

        self.vertical_pub = rospy.Publisher(
            '/gantry/vertical_position_controller/command',
            Float64, queue_size=10)

        self.left_finger_pub = rospy.Publisher(
            '/gantry/left_finger_controller/command',
            Float64, queue_size=10)

        self.right_finger_pub = rospy.Publisher(
            '/gantry/right_finger_controller/command',
            Float64, queue_size=10)

        # 关节状态订阅
        self.joint_state_sub = rospy.Subscriber(
            '/gantry/joint_states',
            JointState,
            self.joint_state_callback)

        # 当前状态
        self.current_horizontal_pos = 0.0
        self.current_vertical_pos = 0.0
        self.current_left_finger_pos = 0.0
        self.current_right_finger_pos = 0.0

        # 无人机相关参数
        self.drone_initial_pos = {'x': 0.0, 'y': 0.0, 'z': 1.3}
        self.drone_target_pos = {'x': 1.2, 'y': 0.0, 'z': 1.3}
        self.drone_handle_height = 0.12  # 抓取杆相对无人机base的高度

        # 运动限制
        self.horizontal_limit = (-0.85, 0.85)
        self.vertical_limit = (0.0, 1.5)
        self.gripper_limit = (0.0, 0.08)

        # 抓取参数
        self.grip_force = 0.025  # 夹持力度（开口距离）
        self.approach_offset = 0.15  # 接近偏移量

        self.rate = rospy.Rate(10)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Gantry-Drone Controller Initialized!")
        rospy.loginfo("=" * 60)
        rospy.sleep(1)

    def joint_state_callback(self, msg):
        """关节状态回调"""
        try:
            if 'horizontal_slider_joint' in msg.name:
                idx = msg.name.index('horizontal_slider_joint')
                self.current_horizontal_pos = msg.position[idx]

            if 'vertical_lift_joint' in msg.name:
                idx = msg.name.index('vertical_lift_joint')
                self.current_vertical_pos = msg.position[idx]

            if 'left_finger_joint' in msg.name:
                idx = msg.name.index('left_finger_joint')
                self.current_left_finger_pos = msg.position[idx]

            if 'right_finger_joint' in msg.name:
                idx = msg.name.index('right_finger_joint')
                self.current_right_finger_pos = msg.position[idx]

        except Exception as e:
            rospy.logwarn(f"Joint state callback error: {e}")

    def move_horizontal(self, position, wait=True):
        """水平移动"""
        position = max(self.horizontal_limit[0], min(position, self.horizontal_limit[1]))
        self.horizontal_pub.publish(Float64(position))
        rospy.loginfo(f"→ Moving horizontal to: {position:.3f}m")
        if wait:
            self._wait_for_position('horizontal', position)

    def move_vertical(self, position, wait=True):
        """垂直移动"""
        position = max(self.vertical_limit[0], min(position, self.vertical_limit[1]))
        self.vertical_pub.publish(Float64(position))
        rospy.loginfo(f"↓ Moving vertical to: {position:.3f}m")
        if wait:
            self._wait_for_position('vertical', position)

    def control_gripper(self, open_distance, wait=True):
        """控制夹钳"""
        open_distance = max(self.gripper_limit[0], min(open_distance, self.gripper_limit[1]))
        self.left_finger_pub.publish(Float64(open_distance))
        self.right_finger_pub.publish(Float64(open_distance))

        if open_distance < 0.01:
            rospy.loginfo("✋ Closing gripper...")
        elif open_distance > 0.07:
            rospy.loginfo("✋ Opening gripper fully...")
        else:
            rospy.loginfo(f"✋ Gripper opening: {open_distance:.3f}m")

        if wait:
            self._wait_for_gripper(open_distance)

    def open_gripper(self, wait=True):
        """打开夹钳"""
        self.control_gripper(0.08, wait)

    def close_gripper(self, wait=True):
        """闭合夹钳"""
        self.control_gripper(0.0, wait)

    def grasp_drone(self, wait=True):
        """抓取无人机（适度夹持）"""
        self.control_gripper(self.grip_force, wait)

    def _wait_for_position(self, axis, target, tolerance=0.02, timeout=10.0):
        """等待到达目标位置"""
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if axis == 'horizontal':
                current = self.current_horizontal_pos
            elif axis == 'vertical':
                current = self.current_vertical_pos
            else:
                return

            if abs(current - target) < tolerance:
                rospy.loginfo(f"  ✓ {axis.capitalize()} reached target")
                return

            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn(f"  ✗ {axis.capitalize()} timeout!")
                return

            self.rate.sleep()

    def _wait_for_gripper(self, target, tolerance=0.005, timeout=5.0):
        """等待夹钳到达目标"""
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            left_error = abs(self.current_left_finger_pos - target)
            right_error = abs(self.current_right_finger_pos - target)

            if left_error < tolerance and right_error < tolerance:
                rospy.loginfo("  ✓ Gripper reached target")
                return

            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("  ✗ Gripper timeout!")
                return

            self.rate.sleep()

    def go_home(self, wait=True):
        """返回初始位置"""
        rospy.loginfo("🏠 Returning to home position...")
        self.move_vertical(0.0, wait)
        self.move_horizontal(0.0, wait)
        self.open_gripper(wait)
        rospy.loginfo("  ✓ Home position reached")

    def calculate_gripper_height_for_drone(self, drone_z):
        """
        计算夹钳需要下降的高度以抓取无人机

        参数:
            drone_z: 无人机base_link的z坐标
        返回:
            vertical_lift_joint 的目标位置
        """
        # 力架顶部高度
        gantry_top_height = 2.5  # frame_height

        # 抓取点的世界坐标高度
        handle_world_z = drone_z + self.drone_handle_height

        # 需要下降的距离
        vertical_position = gantry_top_height - handle_world_z

        return vertical_position

    def pick_and_transport_drone_demo(self):
        """完整演示：抓取无人机并运输到目标位置"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("🚁 DRONE PICK AND TRANSPORT DEMONSTRATION")
        rospy.loginfo("=" * 60 + "\n")

        try:
            # Step 1: 初始化 - 回到原点
            rospy.loginfo("📍 Step 1: Initializing...")
            self.go_home(wait=True)
            rospy.sleep(2)

            # Step 2: 移动到无人机上方
            rospy.loginfo("\n📍 Step 2: Moving above drone...")
            self.move_horizontal(self.drone_initial_pos['x'], wait=True)
            rospy.sleep(1)

            # Step 3: 打开夹钳准备抓取
            rospy.loginfo("\n📍 Step 3: Opening gripper...")
            self.open_gripper(wait=True)
            rospy.sleep(1)

            # Step 4: 下降到抓取高度（接近但不接触）
            rospy.loginfo("\n📍 Step 4: Descending to approach position...")
            approach_height = self.calculate_gripper_height_for_drone(
                self.drone_initial_pos['z']) - self.approach_offset
            self.move_vertical(approach_height, wait=True)
            rospy.sleep(2)

            # Step 5: 精确下降到抓取位置
            rospy.loginfo("\n📍 Step 5: Descending to grasp position...")
            grasp_height = self.calculate_gripper_height_for_drone(
                self.drone_initial_pos['z'])
            self.move_vertical(grasp_height, wait=True)
            rospy.sleep(2)

            # Step 6: 夹持无人机
            rospy.loginfo("\n📍 Step 6: Grasping drone...")
            self.grasp_drone(wait=True)
            rospy.sleep(3)  # 给足够时间让抓取稳定

            # Step 7: 提升无人机
            rospy.loginfo("\n📍 Step 7: Lifting drone...")
            self.move_vertical(0.5, wait=True)
            rospy.sleep(2)

            # Step 8: 水平移动到目标位置
            rospy.loginfo("\n📍 Step 8: Transporting to target position...")
            self.move_horizontal(self.drone_target_pos['x'], wait=True)
            rospy.sleep(2)

            # Step 9: 下降到目标高度
            rospy.loginfo("\n📍 Step 9: Descending to target height...")
            target_height = self.calculate_gripper_height_for_drone(
                self.drone_target_pos['z'])
            self.move_vertical(target_height, wait=True)
            rospy.sleep(2)

            # Step 10: 释放无人机
            rospy.loginfo("\n📍 Step 10: Releasing drone...")
            self.open_gripper(wait=True)
            rospy.sleep(2)

            # Step 11: 上升并返回
            rospy.loginfo("\n📍 Step 11: Retracting and returning home...")
            self.move_vertical(0.3, wait=True)
            rospy.sleep(1)
            self.go_home(wait=True)

            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo("✅ DRONE TRANSPORT DEMO COMPLETED SUCCESSFULLY!")
            rospy.loginfo("=" * 60 + "\n")

        except rospy.ROSInterruptException:
            rospy.loginfo("Demo interrupted by user")
        except Exception as e:
            rospy.logerr(f"❌ Error during demo: {e}")
            self.go_home(wait=True)

    def cyclic_transport_demo(self, cycles=3):
        """循环运输演示"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo(f"🔄 CYCLIC DRONE TRANSPORT ({cycles} cycles)")
        rospy.loginfo("=" * 60 + "\n")

        for i in range(cycles):
            rospy.loginfo(f"\n{'=' * 60}")
            rospy.loginfo(f"Cycle {i + 1}/{cycles}")
            rospy.loginfo(f"{'=' * 60}\n")

            # 从起点到终点
            self.pick_and_transport_drone_demo()
            rospy.sleep(3)

            # 交换起点和终点
            self.drone_initial_pos, self.drone_target_pos = \
                self.drone_target_pos, self.drone_initial_pos

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("✅ CYCLIC TRANSPORT COMPLETED!")
        rospy.loginfo("=" * 60 + "\n")


def print_menu():
    """打印菜单"""
    print("\n" + "=" * 60)
    print("🚁 GANTRY-DRONE CONTROLLER MENU")
    print("=" * 60)
    print("\n Available Demonstrations:")
    print("  1. Pick and Transport Drone (Single)")
    print("  2. Cyclic Transport (Multiple Rounds)")
    print("  3. Go to Home Position")
    print("  4. Test Gripper")
    print("  5. Manual Control")
    print("  0. Exit")
    print("=" * 60)


def main():
    """主函数"""
    try:
        controller = GantryDroneController()
        rospy.sleep(2)

        while not rospy.is_shutdown():
            print_menu()

            try:
                choice = input("\nEnter your choice (0-5): ").strip()

                if choice == '0':
                    rospy.loginfo("Exiting...")
                    break

                elif choice == '1':
                    # 单次抓取运输演示
                    controller.pick_and_transport_drone_demo()

                elif choice == '2':
                    # 循环运输演示
                    cycles = input("Enter number of cycles (default 3): ").strip()
                    cycles = int(cycles) if cycles else 3
                    controller.cyclic_transport_demo(cycles)

                elif choice == '3':
                    # 回到初始位置
                    controller.go_home(wait=True)

                elif choice == '4':
                    # 测试夹爪
                    rospy.loginfo("Testing gripper...")
                    controller.open_gripper(wait=True)
                    rospy.sleep(1)
                    controller.close_gripper(wait=True)
                    rospy.sleep(1)
                    controller.open_gripper(wait=True)

                elif choice == '5':
                    # 手动控制模式
                    rospy.loginfo("Manual control mode - implement as needed")
                    # 这里可以添加手动控制的代码

                else:
                    rospy.logwarn("Invalid choice! Please enter 0-5.")

            except ValueError as e:
                rospy.logwarn(f"Invalid input: {e}")
            except KeyboardInterrupt:
                rospy.loginfo("\nInterrupted by user")
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS shutdown")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")
    finally:
        rospy.loginfo("Controller shutdown")


if __name__ == '__main__':
    main()