#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
"""
Force-set PX4 parameters using raw MAVLink PARAM_SET
with retries and verification.
"""
import rospy
import time
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet, ParamGet

PARAMS_TO_SET = {
    "EKF2_GPS_CHECK" : ("int",   0),
    "CBRK_GPSFAIL"   : ("int",   240024),
    "CBRK_IO_SAFETY" : ("int",   22027),
    "COM_ARM_WO_GPS" : ("int",   1),
    "NAV_RCL_ACT"    : ("int",   0),
    "NAV_DLL_ACT"    : ("int",   0),
}

def set_param(set_srv, get_srv, name, dtype, value, retries=5):
    val = ParamValue()
    if dtype == "int":
        val.integer = int(value)
        val.real    = 0.0
    else:
        val.real    = float(value)
        val.integer = 0

    for attempt in range(retries):
        try:
            rospy.loginfo(f"[{attempt+1}/{retries}] Setting {name} = {value}")
            resp = set_srv(param_id=name, value=val)

            if resp.success:
                # Verify it was actually written
                time.sleep(0.3)
                get_resp = get_srv(param_id=name)
                actual = get_resp.value.integer if dtype == "int" else get_resp.value.real
                if actual == value:
                    rospy.loginfo(f"  ✅ {name} confirmed = {actual}")
                    return True
                else:
                    rospy.logwarn(f"  ⚠️  {name} read back {actual}, expected {value}")
            else:
                rospy.logwarn(f"  ❌ Set rejected on attempt {attempt+1}")

        except Exception as e:
            rospy.logwarn(f"  ❌ Exception: {e}")

        time.sleep(1.0)

    rospy.logerr(f"  🔴 FAILED to set {name} after {retries} attempts")
    return False


def main():
    rospy.init_node("px4_param_setter")

    rospy.loginfo("Waiting for MAVROS param services...")
    rospy.wait_for_service("/mavros/param/set", timeout=10.0)
    rospy.wait_for_service("/mavros/param/get", timeout=10.0)

    set_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)
    get_srv = rospy.ServiceProxy("/mavros/param/get", ParamGet)

    rospy.loginfo("Connected! Setting parameters...\n")
    time.sleep(1.0)

    results = {}
    for name, (dtype, value) in PARAMS_TO_SET.items():
        results[name] = set_param(set_srv, get_srv, name, dtype, value)
        time.sleep(0.5)

    # ── Summary ───────────────────────────────────────────────────────────────
    rospy.loginfo("\n========== PARAMETER SET SUMMARY ==========")
    all_ok = True
    for name, ok in results.items():
        status = "✅" if ok else "❌"
        rospy.loginfo(f"  {status} {name}")
        if not ok:
            all_ok = False

    if all_ok:
        rospy.loginfo("\n✅ All parameters set! Rebooting PX4...")
        # Trigger reboot
        from mavros_msgs.srv import CommandLong
        rospy.wait_for_service("/mavros/cmd/command")
        cmd = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        cmd(command=246, param1=1)   # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    else:
        rospy.logerr("\n❌ Some parameters failed — use QGroundControl or mavshell instead!")

if __name__ == "__main__":
    main()
