#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import math

def main():
    rospy.init_node('poke_spotlight')
    pub_yaw = rospy.Publisher('/spotlight/base_yaw_position/command', Float64, queue_size=1)
    pub_pitch = rospy.Publisher('/spotlight/head_pitch_position/command', Float64, queue_size=1)
    rospy.sleep(1.0)

    r = rospy.Rate(1.0)  # 1 Hz
    t = 0
    while not rospy.is_shutdown():
        yaw = 0.5 * math.sin(t * 0.5)
        pitch = 0.3 * math.sin(t * 0.7)
        pub_yaw.publish(Float64(yaw))
        pub_pitch.publish(Float64(pitch))
        t += 1.0
        r.sleep()

if __name__ == '__main__':
    main()
