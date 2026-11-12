#!/usr/bin/env python3
"""Fly the Nezha-F Scenario C trajectory under one protocol, whichever the arm.

One script drives NezhaSim and the baseline, so both arms share the waypoints, the
pacing, the start state, the wave field, the controller and the sampling.

Protocol:
  - all 100 waypoints of the field record;
  - start at waypoint 0, the true start of the record, at z0 = -0.394 m;
  - uniform pacing, T = path length / mean field speed = 14.449 / 0.7634 = 18.93 s;
  - 100 samples of the ground-truth odometry, uniform in simulation time over [0, T].

Usage: run_scc.py --ns nezha_f|nezha_f_bl --n N --out DIR --waypoints CSV [--T 18.93] [--z0 -0.394]
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Quaternion, Transform, Twist, Vector3
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

ap = argparse.ArgumentParser()
ap.add_argument("--ns", required=True)
ap.add_argument("--n", type=int, required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--waypoints", required=True)
ap.add_argument("--T", type=float, default=18.93)
ap.add_argument("--z0", type=float, default=-0.394)
args = ap.parse_args()

SAMPLES = 100
wps = [(float(r["x_m"]), float(r["y_m"]), float(r["z_m"])) for r in csv.DictReader(open(args.waypoints))]
seg = args.T / (len(wps) - 1)
dt = args.T / (SAMPLES - 1)


class Run:
    def __init__(self):
        self.p = self.v = None
        self.last_odom = 0.0
        self.motor = 0.0
        self.kt = None
        self.sim = None
        self.buf = None
        self.t0 = None
        self.rec_motor = []
        self.rec_kt = []
        self.rec_tilt = []
        self.tilt = 0.0
        rospy.Subscriber(f"/{args.ns}/ground_truth/odometry", Odometry, self.on_odom, queue_size=1)
        rospy.Subscriber(f"/{args.ns}/command/motor_speed", Actuators, self.on_motor, queue_size=1)
        rospy.Subscriber(f"/{args.ns}/motor_0/kt_gain", Float32, self.on_kt, queue_size=1)
        rospy.Subscriber("/clock", Clock, self.on_clock, queue_size=1)
        self.pub = rospy.Publisher(f"/{args.ns}/command/trajectory", MultiDOFJointTrajectory, queue_size=10)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=60)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        rospy.sleep(1.0)

    def on_motor(self, m):
        self.motor = sum(abs(a) for a in m.angular_velocities) / max(len(m.angular_velocities), 1)

    def on_kt(self, m):
        self.kt = m.data

    def on_clock(self, m):
        self.sim = m.clock.to_sec()

    def on_odom(self, m):
        self.p, self.v = m.pose.pose.position, m.twist.twist.linear
        q = m.pose.pose.orientation
        self.tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))))
        self.last_odom = time.time()
        if self.buf is not None and len(self.buf) < SAMPLES:
            if (rospy.Time.now() - self.t0).to_sec() >= len(self.buf) * dt:
                self.buf.append([self.p.x, self.p.y, self.p.z, self.v.x, self.v.y, self.v.z])
                self.rec_motor.append(self.motor)
                self.rec_tilt.append(self.tilt)
                self.rec_kt.append(self.kt)

    def hold(self, x, y, z):
        tr = MultiDOFJointTrajectory()
        tr.header.stamp = rospy.Time.now()
        tr.joint_names = ["base_link"]
        pt = MultiDOFJointTrajectoryPoint()
        pt.transforms.append(Transform(Vector3(x, y, z), Quaternion(0, 0, 0, 1)))
        pt.velocities.append(Twist())
        pt.time_from_start = rospy.Duration(0.0)
        tr.points.append(pt)
        self.pub.publish(tr)

    def teleport(self):
        ms = ModelState()
        ms.model_name = args.ns
        ms.reference_frame = "world"
        ms.pose.position.x, ms.pose.position.y, ms.pose.position.z = wps[0][0], wps[0][1], args.z0
        ms.pose.orientation.w = 1.0
        self.set_state(ms)

    def reset(self):
        x, y, z = wps[0]
        stable, end = 0, time.time() + 20.0
        while not rospy.is_shutdown() and time.time() < end:
            self.hold(x, y, z)
            self.teleport()
            stable = stable + 1 if self.p and math.dist((self.p.x, self.p.y, self.p.z), (x, y, args.z0)) < 0.25 else 0
            if stable >= 15:
                break
            rospy.sleep(0.05)
        self.hold(x, y, z)
        rospy.sleep(1.0)
        self.teleport()
        rospy.sleep(0.1)

    def trajectory(self):
        tr = MultiDOFJointTrajectory()
        tr.header.stamp = rospy.Time.now()
        tr.joint_names = ["base_link"]
        for i, w in enumerate(wps):
            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms.append(Transform(Vector3(*w), Quaternion(0, 0, 0, 1)))
            vel = Twist()
            if i < len(wps) - 1:
                n = wps[i + 1]
                vel.linear = Vector3((n[0] - w[0]) / seg, (n[1] - w[1]) / seg, (n[2] - w[2]) / seg)
            pt.velocities.append(vel)
            pt.time_from_start = rospy.Duration(i * seg)
            tr.points.append(pt)
        return tr

    def fly(self):
        self.reset()
        start = [self.p.x, self.p.y, self.p.z]
        self.rec_motor, self.rec_kt, self.rec_tilt = [], [], []
        self.t0 = rospy.Time.now()
        self.buf = []
        sim0, wall0 = self.sim, time.time()
        self.pub.publish(self.trajectory())
        while not rospy.is_shutdown() and (rospy.Time.now() - self.t0).to_sec() < args.T + 0.5:
            if time.time() - self.last_odom > 10.0:
                return None
            rospy.sleep(0.05)
        rtf = (self.sim - sim0) / (time.time() - wall0) if self.sim is not None and sim0 is not None else None
        buf, self.buf = self.buf, None
        if len(buf) < SAMPLES:
            return None
        tail = self.rec_motor[-20:]
        kts = [k for k in self.rec_kt if k is not None]
        cols = list(zip(*buf))
        return dict(px=cols[0], py=cols[1], pz=cols[2], vx=cols[3], vy=cols[4], vz=cols[5],
                    start=start, rtf=rtf, motor_tail=sum(tail) / len(tail), tilt=self.rec_tilt,
                    kt_min=min(kts) if kts else None, kt_max=max(kts) if kts else None)


rospy.init_node("run_scc", anonymous=True)
os.makedirs(args.out, exist_ok=True)
run = Run()
path = os.path.join(args.out, "runs.jsonl")
while not rospy.is_shutdown():
    done = sum(1 for _ in open(path)) if os.path.exists(path) else 0
    if done >= args.n:
        break
    r = run.fly()
    if r is None:
        print("run lost, the wrapper restarts Gazebo", flush=True)
        sys.exit(3)
    with open(path, "a") as f:
        f.write(json.dumps(r) + "\n")
    print(f"{args.ns} run {done + 1}/{args.n} start {r['start'][2]:.2f} peak z {max(r['pz']):.2f} rtf {r['rtf']:.2f} motor {r['motor_tail']:.0f} max tilt {max(r['tilt']):.0f}", flush=True)
