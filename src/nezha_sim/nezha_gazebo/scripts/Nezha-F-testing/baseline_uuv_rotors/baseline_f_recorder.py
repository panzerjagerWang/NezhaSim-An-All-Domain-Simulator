#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XX-F Scenario-C BASELINE recorder  (UUV_simulator + RotorS, no NSE / no TR)
===========================================================================
Comparison baseline for Test Scenario C of the paper (XX-F HAUV water-to-air
transition trajectory tracking). It flies the SAME water-to-air waypoint path
as the original `flight_controllerv3.py`, under the SAME wave field, but on the
``nezha_f_bl`` robot whose physics come from the *upstream* engines:

  * Hydro : genuine UUV_simulator plugin (uuv_underwater_object_ros_plugin,
            Fossen) -- NO Phase Switch Console and NO Transmedium-Resistance
            (TR) drag model.
  * Rotors: genuine RotorS motor model -- NO Near-Surface-Effect (NSE)
            k_T/k_Q modulation.

The vehicle REALLY flies the trajectory: the lee position controller tracks
``/nezha_f_bl/command/trajectory`` (genuine RotorS trajectory tracking), so the
recorded position/velocity reflect true tracking without NSE/TR.

Output (matches the original Scenario-C recorder, data_recorder.py):
  position_x.txt position_y.txt position_z.txt
  velocity_x.txt velocity_y.txt velocity_z.txt
    -- one "[v0,v1,...,v99]," line appended per epoch (100 samples).
Plus, per epoch, a rosbag for replay:
  baseline_f_run_<epoch>_<ts>.bag  (UUV wrenches, is_submerged, RotorS rotor
  speeds, command/trajectory, ground-truth odometry, /tf).

Usage:
    rosrun ... baseline_f_recorder.py          # 10 epochs
    rosrun ... baseline_f_recorder.py 1        # quick 1-epoch test

Launch first:
    roslaunch nezha_gazebo nezha_f_baseline.launch gui:=false paused:=false
"""

import os
import sys
import math
import time
import signal
import subprocess

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Vector3, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

NS = "nezha_f_bl"
Z_OFFSET = 0.5                  # same +0.5 m trajectory offset as flight_controllerv3
MAX_RECORDS = 100              # samples per epoch (first + 98 + last)
MAX_SPEED = 5.0               # original pacing (matches the reference/XXSim time base)
MIN_SEG_T = 0.3
END_HOLD_S = 12.0             # hold the final waypoint so the vehicle settles at target alt

HERE = os.path.dirname(os.path.abspath(__file__))
WAYPOINTS_CANDIDATES = [
    os.path.join(HERE, "waypoints.csv"),
    os.path.normpath(os.path.join(HERE, "..", "waypoints.csv")),
]
OUT_DIR = os.path.abspath(os.environ.get("NEZHA_F_OUTPUT_DIR", HERE))
os.makedirs(OUT_DIR, exist_ok=True)

BAG_TOPICS = [
    "/debug/forces/" + NS + "/base_link/restoring",
    "/debug/forces/" + NS + "/base_link/damping",
    "/debug/forces/" + NS + "/base_link/added_mass",
    "/debug/forces/" + NS + "/base_link/added_coriolis",
    "/" + NS + "/is_submerged",
    "/" + NS + "/motor_speed/0",
    "/" + NS + "/motor_speed/1",
    "/" + NS + "/motor_speed/2",
    "/" + NS + "/motor_speed/3",
    "/" + NS + "/command/motor_speed",
    "/" + NS + "/command/trajectory",
    "/" + NS + "/ground_truth/odometry",
    "/tf",
]


def load_waypoints():
    import csv
    path = next((p for p in WAYPOINTS_CANDIDATES if os.path.isfile(p)), None)
    if path is None:
        raise RuntimeError("waypoints.csv not found in %s" % WAYPOINTS_CANDIDATES)
    rospy.loginfo("[baseline-F] waypoints: %s", path)
    wps = []
    for r in csv.DictReader(open(path)):
        wps.append({"x": float(r["x_m"]), "y": float(r["y_m"]), "z": float(r["z_m"])})
    return wps


class BaselineF(object):
    def __init__(self):
        rospy.init_node("baseline_f_recorder", anonymous=True)
        self.cur_p = Vector3(0, 0, 0)
        self.cur_v = Vector3(0, 0, 0)

        self.traj_pub = rospy.Publisher("/" + NS + "/command/trajectory",
                                        MultiDOFJointTrajectory, queue_size=10)
        rospy.Subscriber("/" + NS + "/ground_truth/odometry", Odometry, self._odom_cb,
                         queue_size=1)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        self.wps = load_waypoints()
        # Start the test ABOVE water: drop the leading underwater waypoints and
        # begin at the first waypoint whose z is at/above the water line.
        _start = next((i for i, w in enumerate(self.wps) if w["z"] >= 0.0), 0)
        if _start > 0:
            rospy.loginfo("[baseline-F] starting above water at waypoint idx %d "
                          "(z_m=%.2f); dropped %d underwater waypoints",
                          _start, self.wps[_start]["z"], _start)
            self.wps = self.wps[_start:]
        rospy.loginfo("[baseline-F] %d waypoints (z %.2f..%.2f, +%.1f offset)",
                      len(self.wps), min(w["z"] for w in self.wps),
                      max(w["z"] for w in self.wps), Z_OFFSET)

        # per-epoch recording state
        self._reset_buffers()
        self.recording = False
        self.flight_start = None
        self.sampling_interval = 1.0

        rospy.sleep(1.0)

    def _reset_buffers(self):
        self.px, self.py, self.pz = [], [], []
        self.vx, self.vy, self.vz = [], [], []

    def _odom_cb(self, msg):
        self.cur_p = msg.pose.pose.position
        self.cur_v = msg.twist.twist.linear
        if not self.recording or self.flight_start is None:
            return
        elapsed = (rospy.Time.now() - self.flight_start).to_sec()
        flight_records = len(self.px) - 1            # excludes the first waypoint
        if flight_records >= MAX_RECORDS - 2:        # cap at 98 in-flight samples
            return
        if flight_records < int(elapsed / self.sampling_interval):
            self._append_current()

    def _append_current(self):
        self.px.append(self.cur_p.x); self.py.append(self.cur_p.y); self.pz.append(self.cur_p.z)
        self.vx.append(self.cur_v.x); self.vy.append(self.cur_v.y); self.vz.append(self.cur_v.z)

    # -- trajectory (identical structure to flight_controllerv3) --------------
    def build_trajectory(self):
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]
        cum = 0.0
        for i, wp in enumerate(self.wps):
            pt = MultiDOFJointTrajectoryPoint()
            tf = Transform()
            tf.translation.x = wp["x"]
            tf.translation.y = wp["y"]
            tf.translation.z = wp["z"] + Z_OFFSET
            tf.rotation = Quaternion(0, 0, 0, 1)
            pt.transforms.append(tf)
            vel = Twist()
            if i < len(self.wps) - 1:
                nx = self.wps[i + 1]
                dx, dy, dz = nx["x"] - wp["x"], nx["y"] - wp["y"], nx["z"] - wp["z"]
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                seg_t = max(dist / MAX_SPEED, MIN_SEG_T)
                if dist > 0:
                    sp = dist / seg_t
                    vel.linear.x = dx / dist * sp
                    vel.linear.y = dy / dist * sp
                    vel.linear.z = dz / dist * sp
                cum += seg_t
            else:
                vel.linear = Vector3(0, 0, 0)
            vel.angular = Vector3(0, 0, 0)
            pt.velocities.append(vel)
            pt.time_from_start = rospy.Duration(cum)
            traj.points.append(pt)
        return traj, cum

    def _publish_hold(self, x, y, z):
        """Command the lee controller to hold a single setpoint (so it does not
        keep flying back to the previous trajectory's last waypoint)."""
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = "world"
        traj.joint_names = ["base_link"]
        pt = MultiDOFJointTrajectoryPoint()
        tf = Transform()
        tf.translation.x = x
        tf.translation.y = y
        tf.translation.z = z
        tf.rotation = Quaternion(0, 0, 0, 1)
        pt.transforms.append(tf)
        vel = Twist()
        vel.linear = Vector3(0, 0, 0)
        vel.angular = Vector3(0, 0, 0)
        pt.velocities.append(vel)
        pt.time_from_start = rospy.Duration(0.0)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

    def teleport_to_start(self):
        """Reset the vehicle to the FIRST waypoint and make sure it is actually
        there before the run starts. We both (a) command the controller to hold
        wp0 and (b) teleport via set_model_state, then poll ground-truth until
        the body is within tolerance of wp0 (so every epoch begins at the same
        true 'rebirth' location = waypoints.csv[0])."""
        wp0 = self.wps[0]
        tx, ty, tz = wp0["x"], wp0["y"], wp0["z"] + Z_OFFSET
        ms = ModelState()
        ms.model_name = NS
        ms.reference_frame = "world"
        ms.pose.position.x = tx
        ms.pose.position.y = ty
        ms.pose.position.z = tz
        ms.pose.orientation.w = 1.0          # twist left at zero

        rate = rospy.Rate(20)
        deadline = rospy.Time.now() + rospy.Duration(20.0)
        reached = 0
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self._publish_hold(tx, ty, tz)   # controller target = wp0
            try:
                self.set_state(ms)           # hard teleport boost
            except rospy.ServiceException:
                pass
            d = math.sqrt((self.cur_p.x - tx) ** 2 +
                          (self.cur_p.y - ty) ** 2 +
                          (self.cur_p.z - tz) ** 2)
            if d < 0.25:
                reached += 1
                if reached >= 15:            # stable at wp0 for ~0.75 s
                    break
            else:
                reached = 0
            rate.sleep()

        d = math.sqrt((self.cur_p.x - tx) ** 2 +
                      (self.cur_p.y - ty) ** 2 +
                      (self.cur_p.z - tz) ** 2)
        rospy.loginfo("[baseline-F] reset to wp0 (%.3f, %.3f, %.3f); body at "
                      "(%.3f, %.3f, %.3f), err=%.3f m",
                      tx, ty, tz, self.cur_p.x, self.cur_p.y, self.cur_p.z, d)
        rospy.sleep(0.3)

    def save_txt(self, data, fname, mode):
        with open(os.path.join(OUT_DIR, fname), mode) as f:
            f.write("[" + ",".join("%.6f" % v for v in data) + "],\n")

    def save_epoch(self, epoch):
        mode = "w" if epoch == 1 else "a"
        # guarantee exactly MAX_RECORDS samples (pad with last / trim)
        for arr in (self.px, self.py, self.pz, self.vx, self.vy, self.vz):
            while len(arr) < MAX_RECORDS:
                arr.append(arr[-1] if arr else 0.0)
            del arr[MAX_RECORDS:]
        self.save_txt(self.px, "position_x.txt", mode)
        self.save_txt(self.py, "position_y.txt", mode)
        self.save_txt(self.pz, "position_z.txt", mode)
        self.save_txt(self.vx, "velocity_x.txt", mode)
        self.save_txt(self.vy, "velocity_y.txt", mode)
        self.save_txt(self.vz, "velocity_z.txt", mode)
        rospy.loginfo("[baseline-F] epoch %d saved (%d samples/file)", epoch, MAX_RECORDS)

    # -- one epoch -----------------------------------------------------------
    def run_epoch(self, epoch):
        self._reset_buffers()
        self.recording = False
        self.flight_start = None

        rospy.loginfo("=" * 60)
        rospy.loginfo("[baseline-F] EPOCH %d", epoch)
        self.teleport_to_start()

        ts = time.time()
        bag_path = os.path.join(OUT_DIR, "baseline_f_run_%d_%.4f.bag" % (epoch, ts))
        bag = subprocess.Popen(["rosbag", "record", "-O", bag_path] + BAG_TOPICS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               preexec_fn=os.setsid)
        rospy.sleep(1.0)

        traj, total_t = self.build_trajectory()
        rospy.loginfo("[baseline-F] trajectory: %d pts, ~%.1fs", len(traj.points), total_t)

        # one last hard reset so the first recorded sample is exactly wp0
        wp0 = self.wps[0]
        _ms = ModelState()
        _ms.model_name = NS
        _ms.reference_frame = "world"
        _ms.pose.position.x = wp0["x"]
        _ms.pose.position.y = wp0["y"]
        _ms.pose.position.z = wp0["z"] + Z_OFFSET
        _ms.pose.orientation.w = 1.0
        try:
            self.set_state(_ms)
        except rospy.ServiceException:
            pass
        rospy.sleep(0.1)

        self._append_current()                       # first waypoint
        self.sampling_interval = total_t / float(MAX_RECORDS - 2)
        self.flight_start = rospy.Time.now()
        self.recording = True
        self.traj_pub.publish(traj)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.flight_start).to_sec()
            if elapsed >= total_t + 1.0:
                break
            rate.sleep()

        # Hold the final waypoint so the vehicle settles at the target altitude
        # (the genuine RotorS lags the climb; give it time to catch up).
        last = self.wps[-1]
        hold = MultiDOFJointTrajectory()
        hold.header.stamp = rospy.Time.now()
        hold.header.frame_id = "world"
        hold.joint_names = ["base_link"]
        hp = MultiDOFJointTrajectoryPoint()
        htf = Transform()
        htf.translation.x = last["x"]
        htf.translation.y = last["y"]
        htf.translation.z = last["z"] + Z_OFFSET
        htf.rotation = Quaternion(0, 0, 0, 1)
        hp.transforms.append(htf)
        hv = Twist(); hv.linear = Vector3(0, 0, 0); hv.angular = Vector3(0, 0, 0)
        hp.velocities.append(hv)
        hp.time_from_start = rospy.Duration(0.0)
        hold.points.append(hp)
        self.traj_pub.publish(hold)
        rospy.sleep(END_HOLD_S)

        self._append_current()                       # last waypoint (settled)
        self.recording = False

        try:
            os.killpg(os.getpgid(bag.pid), signal.SIGINT)
            bag.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(bag.pid), signal.SIGKILL)
            except Exception:
                pass

        self.save_epoch(epoch)
        rospy.loginfo("[baseline-F] epoch %d: final pos (%.2f, %.2f, %.2f)",
                      epoch, self.cur_p.x, self.cur_p.y, self.cur_p.z)
        rospy.sleep(1.0)

    def run(self, n_epochs):
        for e in range(1, n_epochs + 1):
            self.run_epoch(e)
        rospy.loginfo("[baseline-F] ALL %d epochs complete.", n_epochs)


def main():
    n = 10
    if len(sys.argv) > 1:
        try:
            n = int(sys.argv[1])
        except ValueError:
            pass
    BaselineF().run(n)


if __name__ == "__main__":
    main()
