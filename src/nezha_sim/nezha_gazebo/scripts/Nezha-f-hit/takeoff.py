#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, ParamValue
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, ParamSet

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def set_battery_params():
    try:
        rospy.wait_for_service('/mavros/param/set', timeout=5)
        set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
        params = {
            'BAT_CRIT_THR':    0.0,
            'BAT_LOW_THR':     0.0,
            'BAT_EMERGEN_THR': 0.0,
            'COM_LOW_BAT_ACT': 0.0,
        }
        for name, val in params.items():
            try:
                res = set_param(param_id=name, value=ParamValue(integer=0, real=val))
                rospy.loginfo(f"{'OK' if res.success else 'FAIL'}: {name} = {val}")
            except Exception as e:
                rospy.logwarn(f"Could not set {name}: {e}")
    except Exception as e:
        rospy.logwarn(f"Param service unavailable: {e}")

def main():
    rospy.init_node('offboard_node', anonymous=True)   # ← only ONE init_node

    rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    # Wait for FCU connection
    rospy.loginfo("Waiting for FCU connection...")
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()
    rospy.loginfo("FCU Connected!")

    # Try to set battery params after connection
    set_battery_params()

    # Define target pose
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 2.0
    pose.pose.orientation.w = 1.0

    # Stream setpoints for 5 seconds before switching mode
    rospy.loginfo("Streaming initial setpoints for 5 seconds...")
    for i in range(100):
        if rospy.is_shutdown():
            break
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    rospy.loginfo("Starting main control loop...")
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if set_mode_client.call(offb_set_mode).mode_sent:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()

        elif not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            if arming_client.call(arm_cmd).success:
                rospy.loginfo("Vehicle armed! Taking off to 2 meters...")
            last_req = rospy.Time.now()

        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
