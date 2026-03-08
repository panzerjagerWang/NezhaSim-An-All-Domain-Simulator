#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import tty
import termios
import rospy
from std_msgs.msg import Float64

HELP = """
Gantry velocity teleop (namespace: /gantry)

键位:
  水平滑块:      A 向 -X        D 向 +X        S 停止水平
  垂直升降:      W 向下(+)      X 向上(-)      E 停止垂直
  左指:          Z 向外(+)      C 向内(-)      V 停止左指
  右指:          , 向外(+)      . 向内(-)      / 停止右指
  全部急停:      空格
  增/减速度档:   + 增加10%      - 减少10%      0 重置档位
  退出:          Q

提示:
- 这是“速度控制”，按键即发布一个速度值，松开不会自动停，请用对应“停止键”或空格急停。
- 默认速度(可改): 水平 0.10 m/s, 垂直 0.05 m/s, 手指 0.02 m/s
"""

class GantryVelTeleop(object):
    def __init__(self, ns='/gantry'):
        self.ns = ns
        rospy.init_node('gantry_velocity_teleop', anonymous=True)

        self.pub_h = rospy.Publisher(ns + '/horizontal_velocity_controller/command', Float64, queue_size=1)
        self.pub_v = rospy.Publisher(ns + '/vertical_velocity_controller/command', Float64, queue_size=1)
        self.pub_l = rospy.Publisher(ns + '/left_finger_velocity_controller/command',  Float64, queue_size=1)
        self.pub_r = rospy.Publisher(ns + '/right_finger_velocity_controller/command', Float64, queue_size=1)

        # 基础速度
        self.base_h = 0.10  # m/s (X)
        self.base_v = 0.05  # m/s (Z, 轴定义 0 0 -1，因此“向下”为正速度)
        self.base_f = 0.02  # m/s (finger)

        self.scale = 1.0

        rospy.sleep(0.5)
        print(HELP)

    def publish(self, ph=None, pv=None, pl=None, pr=None):
        # 只对非 None 的通道发布
        if ph is not None: self.pub_h.publish(Float64(ph))
        if pv is not None: self.pub_v.publish(Float64(pv))
        if pl is not None: self.pub_l.publish(Float64(pl))
        if pr is not None: self.pub_r.publish(Float64(pr))

    def stop_all(self):
        self.publish(ph=0.0, pv=0.0, pl=0.0, pr=0.0)

    # 便捷 API：也可在其他脚本 import 使用
    def move_horizontal(self, vel):  # +: +X, -: -X
        self.publish(ph=vel)

    def move_vertical(self, vel):    # 注意：+ 为向下，- 为向上（因为 joint axis 是 0 0 -1）
        self.publish(pv=vel)

    def left_finger(self, vel):      # + 为向外，- 为向内
        self.publish(pl=vel)

    def right_finger(self, vel):     # + 为向外，- 为向内
        self.publish(pr=vel)

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    teleop = GantryVelTeleop(ns='/gantry')
    rate = rospy.Rate(50)

    h = teleop.base_h
    v = teleop.base_v
    f = teleop.base_f

    while not rospy.is_shutdown():
        ch = getch()

        if ch in ('q', 'Q'):
            teleop.stop_all()
            print("退出并已停止所有关节")
            break

        elif ch == ' ':
            teleop.stop_all()
            print("急停：水平/垂直/左右指均已置 0")

        # 档位调节
        elif ch in ('+', '='):
            teleop.scale = min(2.0, teleop.scale * 1.1)
            print("速度比例: x{:.2f}".format(teleop.scale))
        elif ch in ('-', '_'):
            teleop.scale = max(0.1, teleop.scale / 1.1)
            print("速度比例: x{:.2f}".format(teleop.scale))
        elif ch == '0':
            teleop.scale = 1.0
            print("速度比例重置: x1.00")

        # 水平 X
        elif ch in ('a', 'A'):
            teleop.move_horizontal(-h * teleop.scale)
            print("水平: -X {:.3f} m/s".format(-h * teleop.scale))
        elif ch in ('d', 'D'):
            teleop.move_horizontal(+h * teleop.scale)
            print("水平: +X {:.3f} m/s".format(+h * teleop.scale))
        elif ch in ('s', 'S'):
            teleop.move_horizontal(0.0)
            print("水平停止")

        # 垂直 Z（轴向 0 0 -1：给正是向下，给负是向上）
        elif ch in ('w', 'W'):
            teleop.move_vertical(+v * teleop.scale)   # 向下
            print("垂直: 向下 {:.3f} m/s".format(+v * teleop.scale))
        elif ch in ('x', 'X'):
            teleop.move_vertical(-v * teleop.scale)   # 向上
            print("垂直: 向上 {:.3f} m/s".format(-v * teleop.scale))
        elif ch in ('e', 'E'):
            teleop.move_vertical(0.0)
            print("垂直停止")

        # 左指
        elif ch in ('z', 'Z'):
            teleop.left_finger(+f * teleop.scale)     # 向外
            print("左指: 向外 {:.3f} m/s".format(+f * teleop.scale))
        elif ch in ('c', 'C'):
            teleop.left_finger(-f * teleop.scale)     # 向内
            print("左指: 向内 {:.3f} m/s".format(-f * teleop.scale))
        elif ch in ('v', 'V'):
            teleop.left_finger(0.0)
            print("左指停止")

        # 右指
        elif ch == ',':
            teleop.right_finger(+f * teleop.scale)    # 向外
            print("右指: 向外 {:.3f} m/s".format(+f * teleop.scale))
        elif ch == '.':
            teleop.right_finger(-f * teleop.scale)    # 向内
            print("右指: 向内 {:.3f} m/s".format(-f * teleop.scale))
        elif ch == '/':
            teleop.right_finger(0.0)
            print("右指停止")

        else:
            # 忽略其他键
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
