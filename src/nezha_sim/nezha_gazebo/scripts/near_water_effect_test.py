#!/usr/bin/env python3
#
# Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>
# Shanghai Jiao Tong University, The Nezha Lab
# Key Laboratory of Polar Ecosystem and Climate Change
# State Key Laboratory of Submarine Geoscience
#
import math
import rospy
from std_msgs.msg import Float64
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

def clamp(x, a, b):
    return max(a, min(b, x))

class NearWaterEffectTester:
    def __init__(self):
        # Model and reference frame
        self.model_name = rospy.get_param("~model_name", "nezha_rocket")
        self.reference_frame = rospy.get_param("~reference_frame", "world")

        # Water surface and propeller radius
        self.water_surface_z = rospy.get_param("~water_surface_z", 0.0)
        # Rotor radius; if your system can obtain it from a topic, change this to a subscription or service call.
        self.rotor_radius = rospy.get_param("~rotor_radius", 0.11)  # meters, override with the actual value

        # Test range of h/radius: [0,4]
        self.h_over_r_low = rospy.get_param("~h_over_r_low", 0.0)
        self.h_over_r_high = rospy.get_param("~h_over_r_high", 4.0)
        self.h_over_r_tol = rospy.get_param("~h_over_r_tolerance", 0.02)  # tolerance

        # Control speed and rate
        self.speed = rospy.get_param("~speed", 0.1)         # m/s
        self.pub_rate = rospy.get_param("~rate", 20.0)      # Hz

        # z offset of the body top / probe point (use this to shift the reference point to the propeller disk center or blade tip)
        self.z_probe_offset = rospy.get_param("~z_probe_offset", 0.0)

        # Publisher (negative is upward, positive is downward)
        self.pub = rospy.Publisher("/gantry/vertical_velocity_controller/command", Float64, queue_size=10)

        # Gazebo GetModelState
        rospy.loginfo("Waiting for /gazebo/get_model_state ...")
        rospy.wait_for_service("/gazebo/get_model_state")
        self.cli_model = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        # Initial target: first scan upward to h_over_r_high
        self.target_h_over_r = self.h_over_r_high

        # Validity check
        if self.h_over_r_low < 0.0 or self.h_over_r_high > 4.0:
            rospy.logwarn("h_over_r range should be within [0,4]. Clamping to [0,4].")
            self.h_over_r_low = clamp(self.h_over_r_low, 0.0, 4.0)
            self.h_over_r_high = clamp(self.h_over_r_high, 0.0, 4.0)
        if self.h_over_r_low > self.h_over_r_high:
            self.h_over_r_low, self.h_over_r_high = self.h_over_r_high, self.h_over_r_low

        rospy.loginfo("NearWaterEffectTester ready: model=%s, radius=%.3f m, h/r scan=[%.2f, %.2f], speed=%.2f m/s",
                      self.model_name, self.rotor_radius, self.h_over_r_low, self.h_over_r_high, self.speed)

    def get_model_z(self):
        try:
            req = GetModelStateRequest(model_name=self.model_name, relative_entity_name=self.reference_frame)
            resp = self.cli_model(req)
            if not resp.success:
                rospy.logwarn_throttle(2.0, "GetModelState failed for %s", self.model_name)
                return None
            return resp.pose.position.z
        except Exception as e:
            rospy.logwarn_throttle(2.0, "Exception calling GetModelState: %s", str(e))
            return None

    def measure_h_over_r(self, z_model):
        # Reference point height (z_probe_offset can shift it to the propeller disk plane or blade tip)
        z_probe = z_model + self.z_probe_offset
        h = z_probe - self.water_surface_z
        # The near-water definition is usually only valid above water; we still clamp to [0, 4] here for convenient scanning
        if self.rotor_radius <= 1e-6:
            rospy.logwarn_throttle(2.0, "Rotor radius too small or not set. Using 0.11 by default.")
            radius = 0.11
        else:
            radius = self.rotor_radius
        h_over_r = clamp(h / radius, 0.0, 4.0)
        return h, h_over_r

    def compute_command_from_h_over_r(self, h_over_r_now):
        dz = self.target_h_over_r - h_over_r_now
        # Switch target once reached
        if abs(dz) <= self.h_over_r_tol:
            if math.isclose(self.target_h_over_r, self.h_over_r_high, abs_tol=1e-9):
                self.target_h_over_r = self.h_over_r_low
            else:
                self.target_h_over_r = self.h_over_r_high
            rospy.loginfo("Reached h/r target, switch to %.3f", self.target_h_over_r)
            dz = self.target_h_over_r - h_over_r_now

        # Larger h_over_r => need to rise (z increases) => negative velocity
        cmd_speed = -self.speed if dz > 0.0 else self.speed

        # Proportional scaling in the near zone to avoid overshoot
        band = 10.0 * self.h_over_r_tol
        if abs(dz) < band:
            scale = max(0.1, abs(dz) / band)
            cmd_speed *= scale

        return cmd_speed, dz

    def spin(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            z_model = self.get_model_z()
            if z_model is None:
                rate.sleep()
                continue

            h, h_over_r = self.measure_h_over_r(z_model)
            cmd, dz = self.compute_command_from_h_over_r(h_over_r)

            self.pub.publish(Float64(data=cmd))
            rospy.loginfo_throttle(
                1.0,
                "z=%.3f, h=%.3f, h/r=%.3f -> target=%.3f, dz=%.3f, cmd=%.3f",
                z_model, h, h_over_r, self.target_h_over_r, dz, cmd
            )
            rate.sleep()

def main():
    rospy.init_node("near_water_effect_tester")
    node = NearWaterEffectTester()
    node.spin()

if __name__ == "__main__":
    main()
