#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from dynamic_reconfigure.msg import Config, ConfigDescription
from threading import Lock
from dataclasses import dataclass

# 定义服务用于动态设定目标角度
from std_srvs.srv import Trigger, TriggerResponse
from rospy import Service
from rospy import ServiceException

# 自定义简单服务以同时设置 yaw/pitch 目标
from rospy import Service
from rospy.msg import AnyMsg
import argparse

from rospy import ServiceProxy
from rospy import ServiceException
from rospy import ROSException

# 简单自定义服务消息（无依赖新包的做法：用两个独立服务或用参数）
# 这里采用两个标准服务来分别设置 yaw/pitch，另提供一个同时设置的简易话题方式
# 为了方便，这里实现一个ROS param监听+服务组合的方案：

@dataclass
class Targets:
    yaw: float
    pitch: float

class SpotlightControllerNode(object):
    def __init__(self, rate_hz, init_yaw, init_pitch):
        # 话题名（与你提供的完全一致）
        self.ns = "/rexrov/spotlight"
        self.yaw_cmd_topic = self.ns + "/base_yaw_position/command"
        self.yaw_state_topic = self.ns + "/base_yaw_position/state"
        self.yaw_pid_desc_topic = self.ns + "/base_yaw_position/pid/parameter_descriptions"
        self.yaw_pid_updates_topic = self.ns + "/base_yaw_position/pid/parameter_updates"

        self.pitch_cmd_topic = self.ns + "/head_pitch_position/command"
        self.pitch_state_topic = self.ns + "/head_pitch_position/state"
        self.pitch_pid_desc_topic = self.ns + "/head_pitch_position/pid/parameter_descriptions"
        self.pitch_pid_updates_topic = self.ns + "/head_pitch_position/pid/parameter_updates"

        self.lock = Lock()
        self.targets = Targets(yaw=init_yaw, pitch=init_pitch)
        self.rate = rospy.Rate(rate_hz)

        # Publisher
        self.pub_yaw = rospy.Publisher(self.yaw_cmd_topic, Float64, queue_size=10)
        self.pub_pitch = rospy.Publisher(self.pitch_cmd_topic, Float64, queue_size=10)

        # Subscriber
        self.sub_yaw_state = rospy.Subscriber(self.yaw_state_topic, JointControllerState, self.cb_yaw_state, queue_size=10)
        self.sub_pitch_state = rospy.Subscriber(self.pitch_state_topic, JointControllerState, self.cb_pitch_state, queue_size=10)

        # 可选：订阅 PID 动态参数话题，仅用于打印
        self.sub_yaw_pid_desc = rospy.Subscriber(self.yaw_pid_desc_topic, ConfigDescription, self.cb_pid_desc, queue_size=1)
        self.sub_yaw_pid_updates = rospy.Subscriber(self.yaw_pid_updates_topic, Config, self.cb_pid_updates, queue_size=10)
        self.sub_pitch_pid_desc = rospy.Subscriber(self.pitch_pid_desc_topic, ConfigDescription, self.cb_pid_desc, queue_size=1)
        self.sub_pitch_pid_updates = rospy.Subscriber(self.pitch_pid_updates_topic, Config, self.cb_pid_updates, queue_size=10)

        # 服务：分别设置 yaw/pitch 目标；以及一次性同时设置
        self.srv_set_yaw = rospy.Service("~set_yaw", Float64Srv, self.handle_set_yaw)
        self.srv_set_pitch = rospy.Service("~set_pitch", Float64Srv, self.handle_set_pitch)
        self.srv_set_targets = rospy.Service("~set_targets", YawPitchSrv, self.handle_set_targets)

        rospy.loginfo("SpotlightControllerNode ready. Publishing to:")
        rospy.loginfo("  %s", self.yaw_cmd_topic)
        rospy.loginfo("  %s", self.pitch_cmd_topic)

    def cb_yaw_state(self, msg):
        rospy.logdebug("Yaw state | setpoint=%.3f, process=%.3f, error=%.3f, cmd=%.3f",
                       msg.set_point, msg.process_value, msg.error, msg.command)

    def cb_pitch_state(self, msg):
        rospy.logdebug("Pitch state | setpoint=%.3f, process=%.3f, error=%.3f, cmd=%.3f",
                       msg.set_point, msg.process_value, msg.error, msg.command)

    def cb_pid_desc(self, msg):
        # 仅首次到来时打印一次
        rospy.loginfo_once("Received PID parameter descriptions.")

    def cb_pid_updates(self, msg):
        # 打印更新（可注释掉以减少输出）
        names = [p.name for p in msg.doubles] + [p.name for p in msg.ints] + [p.name for p in msg.bools]
        rospy.logdebug("PID params updated: %s", names)

    def handle_set_yaw(self, req):
        with self.lock:
            self.targets.yaw = req.data
        rospy.loginfo("Set yaw target to %.3f rad", req.data)
        return Float64SrvResponse(success=True, message="Yaw target updated.")

    def handle_set_pitch(self, req):
        with self.lock:
            self.targets.pitch = req.data
        rospy.loginfo("Set pitch target to %.3f rad", req.data)
        return Float64SrvResponse(success=True, message="Pitch target updated.")

    def handle_set_targets(self, req):
        with self.lock:
            self.targets.yaw = req.yaw
            self.targets.pitch = req.pitch
        rospy.loginfo("Set targets -> yaw: %.3f, pitch: %.3f", req.yaw, req.pitch)
        return YawPitchSrvResponse(success=True, message="Targets updated.")

    def spin(self):
        # 等待连接
        rospy.loginfo("Waiting for subscribers to connect...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while (self.pub_yaw.get_num_connections() == 0 or self.pub_pitch.get_num_connections() == 0) and not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logwarn("No subscribers connected to command topics yet. Continuing anyway.")
                break
            rospy.sleep(0.1)

        # 主循环：周期性发布目标角
        while not rospy.is_shutdown():
            with self.lock:
                yaw = self.targets.yaw
                pitch = self.targets.pitch
            self.pub_yaw.publish(Float64(yaw))
            self.pub_pitch.publish(Float64(pitch))
            self.rate.sleep()


# 为了不引入自定义srv文件，这里临时定义简单服务类型：
# 采用 rospy.Service 的“动态类型”会很麻烦，这里提供最简单方案：
# 1) 使用两个标准话题接收目标值：/spotlight_control/yaw_cmd 和 /spotlight_control/pitch_cmd
# 2) 避免自定义srv，去掉上面服务，改为订阅这两个命令话题
# 为确保脚本可直接运行，下面给出一个不依赖自定义srv的精简版本：

class SpotlightControllerSimple(object):
    def __init__(self, rate_hz, init_yaw, init_pitch):
        self.ns = "/rexrov/spotlight"
        self.yaw_cmd_topic = self.ns + "/base_yaw_position/command"
        self.yaw_state_topic = self.ns + "/base_yaw_position/state"
        self.pitch_cmd_topic = self.ns + "/head_pitch_position/command"
        self.pitch_state_topic = self.ns + "/head_pitch_position/state"

        self.control_ns = "~"  # 私有命名空间
        self.user_yaw_cmd_topic = self.control_ns + "yaw_cmd"
        self.user_pitch_cmd_topic = self.control_ns + "pitch_cmd"

        self.lock = Lock()
        self.targets = Targets(yaw=init_yaw, pitch=init_pitch)
        self.rate = rospy.Rate(rate_hz)

        self.pub_yaw = rospy.Publisher(self.yaw_cmd_topic, Float64, queue_size=10)
        self.pub_pitch = rospy.Publisher(self.pitch_cmd_topic, Float64, queue_size=10)

        self.sub_yaw_state = rospy.Subscriber(self.yaw_state_topic, JointControllerState, self.cb_yaw_state, queue_size=10)
        self.sub_pitch_state = rospy.Subscriber(self.pitch_state_topic, JointControllerState, self.cb_pitch_state, queue_size=10)

        # 订阅用户命令话题，以 Float64 更新目标
        self.sub_user_yaw = rospy.Subscriber(self.user_yaw_cmd_topic, Float64, self.cb_user_yaw, queue_size=10)
        self.sub_user_pitch = rospy.Subscriber(self.user_pitch_cmd_topic, Float64, self.cb_user_pitch, queue_size=10)
        self.scan_enabled = rospy.get_param("~scan_enabled", True)
        self.yaw_min = rospy.get_param("~yaw_min", -1.0)
        self.yaw_max = rospy.get_param("~yaw_max", 1.0)
        self.yaw_speed = rospy.get_param("~yaw_speed", 0.3)  # rad/s

        # pitch：固定或小幅扫描（二选一，amp=0 表示固定）
        self.pitch_fixed = rospy.get_param("~pitch_fixed", 0.0)
        self.pitch_scan_amp = rospy.get_param("~pitch_scan_amp", 0.0)  # rad
        self.pitch_speed = rospy.get_param("~pitch_speed", 0.2)  # rad/s

        # 运行时状态
        self.scan_dir = 1
        with self.lock:
            self.yaw_curr = float(self.targets.yaw)
            self.pitch_curr = float(self.targets.pitch)
        self._last_time = rospy.Time.now()
        rospy.loginfo("Simple controller started. Publish Float64 to:")
        rospy.loginfo("  %s (yaw target)", rospy.resolve_name(self.user_yaw_cmd_topic))
        rospy.loginfo("  %s (pitch target)", rospy.resolve_name(self.user_pitch_cmd_topic))

    def cb_yaw_state(self, msg):
        rospy.logdebug("Yaw state | set=%.3f, pv=%.3f, err=%.3f, cmd=%.3f",
                       msg.set_point, msg.process_value, msg.error, msg.command)

    def cb_pitch_state(self, msg):
        rospy.logdebug("Pitch state | set=%.3f, pv=%.3f, err=%.3f, cmd=%.3f",
                       msg.set_point, msg.process_value, msg.error, msg.command)

    def cb_user_yaw(self, msg):
        val = float(msg.data)
        with self.lock:
            self.targets.yaw = val
            # 将当前扫描位置贴合到新值，并裁剪到范围内
            self.yaw_curr = max(self.yaw_min, min(self.yaw_max, val))
        rospy.loginfo("Updated yaw target to %.3f rad via topic (sync scan center)", self.yaw_curr)

    def cb_user_pitch(self, msg):
        val = float(msg.data)
        with self.lock:
            self.targets.pitch = val
            self.pitch_fixed = val  # 把外部给的值作为新的固定角/扫描中心
            self.pitch_curr = val
        rospy.loginfo("Updated pitch target to %.3f rad via topic (as fixed/center)", self.pitch_curr)

    def spin(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self._last_time).to_sec()
            if dt <= 0.0:
                dt = 1.0 / max(self.rate.sleep_dur.to_sec(), 1e-6)  # 兜底
            self._last_time = now

            with self.lock:
                if self.scan_enabled:
                    # --- Yaw 扫描（三角波） ---
                    self.yaw_curr += self.scan_dir * self.yaw_speed * dt
                    if self.yaw_curr > self.yaw_max:
                        self.yaw_curr = self.yaw_max
                        self.scan_dir = -1
                    elif self.yaw_curr < self.yaw_min:
                        self.yaw_curr = self.yaw_min
                        self.scan_dir = 1

                    yaw_cmd = self.yaw_curr

                    # --- Pitch 扫描（可选，正弦） ---
                    if self.pitch_scan_amp > 1e-6:
                        # 以 pitch_fixed 为中心做正弦扫：p = center + A * sin(ωt)
                        # 这里用内部累计角实现：ω = pitch_speed / A 等效不是角频率。
                        # 更直观：用时间驱动正弦相位
                        t = rospy.get_time()
                        pitch_cmd = self.pitch_fixed + self.pitch_scan_amp * __import__("math").sin(
                            self.pitch_speed * t)
                        self.pitch_curr = pitch_cmd
                    else:
                        pitch_cmd = self.pitch_fixed
                        self.pitch_curr = pitch_cmd
                else:
                    # 手动模式：跟随 targets
                    yaw_cmd = float(self.targets.yaw)
                    pitch_cmd = float(self.targets.pitch)
                    self.yaw_curr = yaw_cmd
                    self.pitch_curr = pitch_cmd

            # 发布命令
            self.pub_yaw.publish(Float64(yaw_cmd))
            self.pub_pitch.publish(Float64(pitch_cmd))

            self.rate.sleep()


def parse_args():
    parser = argparse.ArgumentParser(description="Control spotlight yaw/pitch position controllers.")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate (Hz)")
    parser.add_argument("--yaw", type=float, default=0.0, help="Initial yaw target (rad)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Initial pitch target (rad)")
    parser.add_argument("--mode", choices=["simple"], default="simple",
                        help="Use 'simple' mode without custom services.")

    # scan-related CLI options
    parser.add_argument("--scan-enabled", type=lambda v: v.lower() in ("1","true","yes","y"), default=True,
                        help="Enable scanning (true/false). If omitted, use ROS param or default.")
    parser.add_argument("--yaw-min", type=float, default=None, help="Yaw lower bound (rad)")
    parser.add_argument("--yaw-max", type=float, default=None, help="Yaw upper bound (rad)")
    parser.add_argument("--yaw-speed", type=float, default=None, help="Yaw scan speed (rad/s)")
    parser.add_argument("--pitch-fixed", type=float, default=None, help="Fixed pitch or scan center (rad)")
    parser.add_argument("--pitch-scan-amp", type=float, default=45, help="Pitch scan amplitude (rad)")
    parser.add_argument("--pitch-speed", type=float, default=None, help="Pitch scan angular speed for sine (rad/s)")
    return parser.parse_args(rospy.myargv()[1:])


if __name__ == "__main__":
    rospy.init_node("spotlight_control", anonymous=False)
    args = parse_args()

    # 将命令行覆盖写入到私有参数（仅当提供时）
    # SpotlightControllerSimple 使用的是 ~ 命名空间读取参数，这里同步写入。
    if args.scan_enabled is not None:
        rospy.set_param("~scan_enabled", bool(args.scan_enabled))
    if args.yaw_min is not None:
        rospy.set_param("~yaw_min", args.yaw_min)
    if args.yaw_max is not None:
        rospy.set_param("~yaw_max", args.yaw_max)
    if args.yaw_speed is not None:
        rospy.set_param("~yaw_speed", args.yaw_speed)
    if args.pitch_fixed is not None:
        rospy.set_param("~pitch_fixed", args.pitch_fixed)
    if args.pitch_scan_amp is not None:
        rospy.set_param("~pitch_scan_amp", args.pitch_scan_amp)
    if args.pitch_speed is not None:
        rospy.set_param("~pitch_speed", args.pitch_speed)

    if args.mode == "simple":
        node = SpotlightControllerSimple(rate_hz=args.rate, init_yaw=args.yaw, init_pitch=args.pitch)
    else:
        node = SpotlightControllerSimple(rate_hz=args.rate, init_yaw=args.yaw, init_pitch=args.pitch)

    node.spin()
