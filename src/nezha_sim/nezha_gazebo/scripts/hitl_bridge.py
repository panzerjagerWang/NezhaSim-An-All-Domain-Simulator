#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
import math
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from mav_msgs.msg import Actuators
from mavros_msgs.msg import HilSensor, HilGPS, HilActuatorControls


class HitlBridge:
    def __init__(self):
        rospy.init_node("hitl_bridge")

        self.mag_n = rospy.get_param("~mag_n", 0.20535)
        self.mag_e = rospy.get_param("~mag_e", 0.00278)
        self.mag_d = rospy.get_param("~mag_d", 0.43104)

        self.motor_max = rospy.get_param("~motor_max", 838.0)
        self.imu_min_interval = rospy.get_param("~imu_min_interval", 0.02)

        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0

        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0

        self.last_imu_stamp = None
        self.gps_ready = False
        self.odom_ready = False

        self.hil_pub = rospy.Publisher("/mavros/hil/imu_ned", HilSensor, queue_size=20)
        self.gps_pub = rospy.Publisher("/mavros/hil/gps", HilGPS, queue_size=10)
        self.motor_pub = rospy.Publisher("/nezha_f_mav/command/motor_speed", Actuators, queue_size=10)

        # ✅ 直接订阅 Gazebo 输出的真实 GPS 数据
        rospy.Subscriber("/nezha_f_mav/gps", NavSatFix, self.gps_cb, queue_size=10)
        rospy.Subscriber("/nezha_f_mav/odometry_sensor1/odometry", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/nezha_f_mav/imu", Imu, self.imu_cb, queue_size=50)
        rospy.Subscriber("/mavros/hil/actuator_controls", HilActuatorControls, self.actuator_cb, queue_size=10)

        rospy.Timer(rospy.Duration(0.1), self.publish_gps)

        rospy.loginfo("HITL Bridge started successfully!")
        rospy.spin()

    def gps_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude
        self.gps_ready = True

    def odom_cb(self, msg):
        self.vel_x = msg.twist.twist.linear.x
        self.vel_y = msg.twist.twist.linear.y
        self.vel_z = msg.twist.twist.linear.z
        self.odom_ready = True

    def imu_cb(self, msg):
        # 🚀 关键修改：直接获取当前系统时间，不用 msg 里的时间戳！
        stamp_sec = rospy.Time.now().to_sec()

        if self.last_imu_stamp is not None:
            if (stamp_sec - self.last_imu_stamp) < self.imu_min_interval:
                return

        is_first_frame = (self.last_imu_stamp is None)
        self.last_imu_stamp = stamp_sec

        acc_x = msg.linear_acceleration.x
        acc_y = -msg.linear_acceleration.y
        acc_z = -msg.linear_acceleration.z

        gyro_x = msg.angular_velocity.x
        gyro_y = -msg.angular_velocity.y
        gyro_z = -msg.angular_velocity.z

        hil = HilSensor()
        hil.header.stamp = rospy.Time()
        hil.header.frame_id = "base_link"

        hil.acc.x = acc_x
        hil.acc.y = acc_y
        hil.acc.z = acc_z

        hil.gyro.x = gyro_x
        hil.gyro.y = gyro_y
        hil.gyro.z = gyro_z

        hil.mag.x = self.mag_n
        hil.mag.y = self.mag_e
        hil.mag.z = self.mag_d

        try:
            # 注意这里改成了 1013.25
            hil.abs_pressure = 1013.25 * ((1.0 - 2.25577e-5 * max(0.0, self.alt)) ** 5.25588)
        except Exception:
            hil.abs_pressure = 1013.25

        hil.diff_pressure = 0.0
        hil.pressure_alt = self.alt
        hil.temperature = 25.0

        hil.fields_updated = 7167

        self.hil_pub.publish(hil)

    def publish_gps(self, _event):
        if not self.gps_ready or not self.odom_ready:
            return

        vn = self.vel_y
        ve = self.vel_x
        vd = -self.vel_z

        speed_2d = math.sqrt(vn * vn + ve * ve)
        speed_3d = math.sqrt(vn * vn + ve * ve + vd * vd)

        gps = HilGPS()
        gps.header.stamp = rospy.Time()
        gps.header.frame_id = "base_link"
        gps.fix_type = 3
        gps.geo.latitude = self.lat
        gps.geo.longitude = self.lon
        gps.geo.altitude = self.alt
        gps.eph = 1
        gps.epv = 1
        gps.vel = int(speed_3d * 100.0)
        gps.vn = int(vn * 100.0)
        gps.ve = int(ve * 100.0)
        gps.vd = int(vd * 100.0)

        if speed_2d > 0.2:
            cog_deg = math.degrees(math.atan2(ve, vn))
            if cog_deg < 0.0:
                cog_deg += 360.0
            gps.cog = int(cog_deg * 100.0)
        else:
            gps.cog = 0

        gps.satellites_visible = 12
        self.gps_pub.publish(gps)

    def actuator_cb(self, msg):
        motor_cmd = Actuators()
        motor_cmd.header.stamp = rospy.Time.now()

        omegas = []
        for i in range(4):
            ctrl = max(0.0, min(1.0, float(msg.controls[i])))
            omega = math.sqrt(ctrl) * self.motor_max
            omegas.append(omega)

        motor_cmd.angular_velocities = omegas
        self.motor_pub.publish(motor_cmd)


if __name__ == "__main__":
    try:
        HitlBridge()
    except rospy.ROSInterruptException:
        pass