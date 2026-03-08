#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

DEFAULT_GANTRY_MODEL = "nezha_gantry"
DEFAULT_GANTRY_LINK  = "vertical_lift_arm"
DEFAULT_UAV_MODEL    = "nezha_rocket"
DEFAULT_UAV_LINK     = "nezha_rocket/base_link"  # 注意：这里用纯链接名


def set_model_pose(model_name, x, y, z, roll=0, pitch=0, yaw=0):
    """设置模型的位置和姿态"""
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # 将欧拉角转换为四元数
        from tf.transformations import quaternion_from_euler
        quat = quaternion_from_euler(roll, pitch, yaw)

        state = ModelState()
        state.model_name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]

        resp = set_state(state)
        if resp.success:
            rospy.loginfo("成功设置 %s 姿态为笔直", model_name)
            return True
        else:
            rospy.logerr("设置姿态失败: %s", resp.status_message)
            return False
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", str(e))
        return False


def call_attach(gantry_model, gantry_link, uav_model, uav_link, attach=True):
    service_name = "/link_attacher_node/attach" if attach else "/link_attacher_node/detach"
    rospy.loginfo("等待服务: %s", service_name)
    rospy.wait_for_service(service_name, timeout=rospy.get_param("~wait_timeout", 20.0))
    proxy = rospy.ServiceProxy(service_name, Attach)

    req = AttachRequest()
    req.model_name_1 = gantry_model
    req.link_name_1 = gantry_link
    req.model_name_2 = uav_model
    req.link_name_2 = uav_link

    action = "attach" if attach else "detach"
    rospy.loginfo("调用 %s: [%s/%s] <-> [%s/%s]", action, gantry_model, gantry_link, uav_model, uav_link)
    try:
        resp = proxy(req)
        if resp.ok:
            rospy.loginfo("%s 成功。", action.capitalize())
            return True
        else:
            rospy.logerr("%s 失败，服务返回 ok=false。", action.capitalize())
            return False
    except rospy.ServiceException as e:
        rospy.logerr("服务调用异常: %s", str(e))
        return False


def main():
    rospy.init_node("attach_after_spawn", anonymous=False)

    import argparse
    parser = argparse.ArgumentParser(description="使用 gazebo_ros_link_attacher 将两模型链接固定/分离")
    parser.add_argument("--gantry_model", default=DEFAULT_GANTRY_MODEL)
    parser.add_argument("--gantry_link", default=DEFAULT_GANTRY_LINK)
    parser.add_argument("--uav_model", default=DEFAULT_UAV_MODEL)
    parser.add_argument("--uav_link", default=DEFAULT_UAV_LINK)
    parser.add_argument("--detach", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    gantry_model = rospy.get_param("~gantry_model", args.gantry_model)
    gantry_link = rospy.get_param("~gantry_link", args.gantry_link)
    uav_model = rospy.get_param("~uav_model", args.uav_model)
    uav_link = rospy.get_param("~uav_link", args.uav_link)
    do_detach = rospy.get_param("~detach", args.detach)

    # 兼容 "model/link" 误传
    if "/" in uav_model and "/" not in uav_link:
        parts = uav_model.split("/")
        if len(parts) == 2:
            rospy.logwarn("检测到 uav_model 包含 '/', 自动拆分为: %s", uav_model)
            uav_model, uav_link = parts[0], parts[1]

    # 等待 Gazebo 模型出现
    wait_models = rospy.get_param("~wait_models", True)
    wait_time = rospy.get_param("~spawn_wait", 15.0)
    if wait_models:
        import time
        from gazebo_msgs.msg import ModelStates
        got_models = set()

        def cb(msg):
            got_models.update(msg.name)

        sub = rospy.Subscriber("/gazebo/model_states", ModelStates, cb, queue_size=1)
        rospy.loginfo("等待模型加载: %s, %s (超时 %.1fs)", gantry_model, uav_model, wait_time)
        deadline = time.time() + wait_time
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if gantry_model in got_models and uav_model in got_models:
                break
            rate.sleep()
        sub.unregister()

    # ===== 新增：连接前重置 UAV 姿态 =====
    if not do_detach:
        rospy.loginfo("重置 UAV 姿态为笔直...")
        # 获取当前位置（或使用固定位置）
        uav_x = rospy.get_param("~uav_x", 0.0)
        uav_y = rospy.get_param("~uav_y", 0.0)
        uav_z = rospy.get_param("~uav_z", 1.80)

        # 设置为水平姿态 (roll=0, pitch=0, yaw=0)
        set_model_pose(uav_model, uav_x, uav_y, uav_z,
                       roll=0.0, pitch=0.0, yaw=0.0)
        rospy.sleep(0.5)  # 等待姿态稳定
    # =====================================

    ok = call_attach(
        gantry_model=gantry_model,
        gantry_link=gantry_link or "vertical_lift_arm",
        uav_model=uav_model,
        uav_link=uav_link,
        attach=(not do_detach)
    )

    sys.exit(0 if ok else 1)

def main():
    rospy.init_node("attach_after_spawn", anonymous=False)

    import argparse
    parser = argparse.ArgumentParser(description="使用 gazebo_ros_link_attacher 将两模型链接固定/分离")
    parser.add_argument("--gantry_model", default=DEFAULT_GANTRY_MODEL)
    parser.add_argument("--gantry_link",  default=DEFAULT_GANTRY_LINK)
    parser.add_argument("--uav_model",    default=DEFAULT_UAV_MODEL)
    parser.add_argument("--uav_link",     default=DEFAULT_UAV_LINK)
    parser.add_argument("--detach", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    gantry_model = rospy.get_param("~gantry_model", args.gantry_model)
    gantry_link  = rospy.get_param("~gantry_link",  args.gantry_link)
    uav_model    = rospy.get_param("~uav_model",    args.uav_model)
    uav_link     = rospy.get_param("~uav_link",     args.uav_link)
    do_detach    = rospy.get_param("~detach",       args.detach)

    # 兼容 "model/link" 误传
    if "/" in uav_model and "/" not in uav_link:
        parts = uav_model.split("/")
        if len(parts) == 2:
            rospy.logwarn("检测到 uav_model 包含 '/', 自动拆分为: %s", uav_model)
            uav_model, uav_link = parts[0], parts[1]

    # 等待 Gazebo 模型出现（可选）
    wait_models = rospy.get_param("~wait_models", True)
    wait_time   = rospy.get_param("~spawn_wait", 15.0)
    if wait_models:
        import time
        from gazebo_msgs.msg import ModelStates
        got_models = set()
        def cb(msg):
            got_models.update(msg.name)
        sub = rospy.Subscriber("/gazebo/model_states", ModelStates, cb, queue_size=1)
        rospy.loginfo("等待模型加载: %s, %s (超时 %.1fs)", gantry_model, uav_model, wait_time)
        deadline = time.time() + wait_time
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if gantry_model in got_models and uav_model in got_models:
                break
            rate.sleep()
        sub.unregister()

    ok = call_attach(
        gantry_model=gantry_model,
        gantry_link=gantry_link or "vertical_lift_arm",  # 防空值，简单可靠
        uav_model=uav_model,
        uav_link=uav_link,
        attach=(not do_detach)
    )

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
