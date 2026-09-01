#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XX-Husky Scenario-A BASELINE recorder  (UUV_simulator + RotorS)
================================================================
This is the comparison baseline for Test Scenario A of the paper. It runs
the SAME land -> water -> air mission as the XXSim controller, but on the
``nezha_husky_bl`` robot, whose physics come from the *upstream* engines:

  * Underwater  : genuine UUV_simulator hydrodynamics plugin
                  (uuv_underwater_object_ros_plugin, Fossen model, Table-II
                  parameters), deliberately UNGATED -> no Phase Switch Console.
  * Aerial      : genuine RotorS motor model (librotors_gazebo_motor_model),
                  no Near-Surface-Effect k_T/k_Q modulation. The body is
                  *teleported* along the aerial trajectory while the RotorS
                  rotors spin (hover command), exactly as requested.

The whole experiment uses the SAME wave field (``nezha_ocean_waves``) as the
XXSim run, so any wrench difference isolates the simulator core.

For every trial we record, in the same column layout as the XXSim controller's
``trajectory_data_with_forces_<ts>.csv``:
    timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw,
    buoyancy_*(=UUV restoring), damping_*(=UUV damping),
    wave_*(=0, UUV has no wave coupling),
    added_mass_*(=UUV added_mass), coriolis_*(=UUV added_coriolis),
    surface_z, z_over_l, phase_name, robot_type
and a matching rosbag (forces + rotor speeds + odometry) for replay.

Usage:
    rosrun ... baseline_recorder.py            # 10 trials
    rosrun ... baseline_recorder.py 1          # 1 trial (quick test)

Launch first:
    roslaunch nezha_gazebo nezha_husky_baseline.launch gui:=false paused:=false
"""

import os
import sys
import math
import time
import signal
import subprocess

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion, quaternion_from_euler

try:
    from mav_msgs.msg import Actuators
    HAVE_ACTUATORS = True
except Exception:
    HAVE_ACTUATORS = False

# ----------------------------------------------------------------------------
NS = "nezha_husky_bl"
HULL_L = 2.5                       # reference length used for z_over_l (= wave scale)
SAMPLE_DIST = 0.4                 # metres between recorded samples (distance based)
TICK_HZ = 50.0                   # teleport update rate
SPEED = {"UGV": 1.5, "UUV": 1.0, "UAV": 1.5}   # m/s used to drive each phase
HOVER_OMEGA = 600.0              # rad/s hover command for the RotorS rotors (air phase)

HERE = os.path.dirname(os.path.abspath(__file__))
REF_CSV_CANDIDATES = [
    os.path.join(HERE, "smoothed_trajectory_data.csv"),
    os.path.normpath(os.path.join(HERE, "..", "..", "smoothed_trajectory_data_modified.csv")),
]
OUT_DIR = HERE
BAG_TOPICS = [
    "/" + NS + "/ground_truth/odometry",
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
    "/tf",
]

CSV_HEADER = ("timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw,"
              "buoyancy_x,buoyancy_y,buoyancy_z,"
              "damping_x,damping_y,damping_z,"
              "wave_x,wave_y,wave_z,"
              "added_mass_x,added_mass_y,added_mass_z,"
              "coriolis_x,coriolis_y,coriolis_z,"
              "surface_z,z_over_l,phase_name,robot_type")


# ----------------------------------------------------------------------------
def load_reference():
    import csv
    path = next((p for p in REF_CSV_CANDIDATES if os.path.isfile(p)), None)
    if path is None:
        raise RuntimeError("reference trajectory CSV not found in %s" % REF_CSV_CANDIDATES)
    rospy.loginfo("[baseline] reference trajectory: %s", path)
    pts = []
    for r in csv.DictReader(open(path)):
        pts.append((float(r["x"]), float(r["y"]), float(r["z"]), r["vehicle_type"].strip()))
    return pts


def split_phases(pts):
    """Return list of (phase_name, [ (x,y,z), ... ]) preserving order."""
    phases, cur, cur_t = [], [], None
    for x, y, z, t in pts:
        if t != cur_t:
            if cur:
                phases.append((cur_t, cur))
            cur, cur_t = [], t
        cur.append((x, y, z))
    if cur:
        phases.append((cur_t, cur))
    return phases


def densify(poly, max_seg):
    """Insert points so consecutive samples are <= max_seg apart."""
    if len(poly) < 2:
        return list(poly)
    out = [poly[0]]
    for i in range(len(poly) - 1):
        a, b = poly[i], poly[i + 1]
        d = math.dist(a, b)
        n = max(1, int(math.ceil(d / max_seg)))
        for k in range(1, n + 1):
            t = k / float(n)
            out.append((a[0] + t * (b[0] - a[0]),
                        a[1] + t * (b[1] - a[1]),
                        a[2] + t * (b[2] - a[2])))
    return out


# ----------------------------------------------------------------------------
class Latest(object):
    """Holds the most-recent value from a topic."""
    def __init__(self):
        self.odom = None
        self.restoring = (0.0, 0.0, 0.0)
        self.damping = (0.0, 0.0, 0.0)
        self.added_mass = (0.0, 0.0, 0.0)
        self.coriolis = (0.0, 0.0, 0.0)
        self.is_submerged = False


def wrench_cb(msg, store_attr, latest):
    f = msg.wrench.force
    setattr(latest, store_attr, (f.x, f.y, f.z))


class BaselineRunner(object):
    def __init__(self):
        rospy.init_node("husky_baseline_recorder", anonymous=True)
        self.latest = Latest()

        rospy.loginfo("[baseline] waiting for /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30)
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.Subscriber("/" + NS + "/ground_truth/odometry", Odometry,
                         lambda m: setattr(self.latest, "odom", m), queue_size=1)
        rospy.Subscriber("/debug/forces/" + NS + "/base_link/restoring", WrenchStamped,
                         lambda m: wrench_cb(m, "restoring", self.latest), queue_size=1)
        rospy.Subscriber("/debug/forces/" + NS + "/base_link/damping", WrenchStamped,
                         lambda m: wrench_cb(m, "damping", self.latest), queue_size=1)
        rospy.Subscriber("/debug/forces/" + NS + "/base_link/added_mass", WrenchStamped,
                         lambda m: wrench_cb(m, "added_mass", self.latest), queue_size=1)
        rospy.Subscriber("/debug/forces/" + NS + "/base_link/added_coriolis", WrenchStamped,
                         lambda m: wrench_cb(m, "coriolis", self.latest), queue_size=1)
        rospy.Subscriber("/" + NS + "/is_submerged", Bool,
                         lambda m: setattr(self.latest, "is_submerged", bool(m.data)), queue_size=1)

        self.motor_pub = None
        if HAVE_ACTUATORS:
            self.motor_pub = rospy.Publisher("/" + NS + "/command/motor_speed",
                                             Actuators, queue_size=1)

        self.phases = split_phases(load_reference())
        rospy.loginfo("[baseline] phases: %s", [(t, len(p)) for t, p in self.phases])

    # -- teleport one step ---------------------------------------------------
    def teleport(self, x, y, z, yaw, vx, vy, vz):
        ms = ModelState()
        ms.model_name = NS
        ms.reference_frame = "world"
        ms.pose.position.x = x
        ms.pose.position.y = y
        ms.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        ms.pose.orientation.x, ms.pose.orientation.y = q[0], q[1]
        ms.pose.orientation.z, ms.pose.orientation.w = q[2], q[3]
        ms.twist.linear.x, ms.twist.linear.y, ms.twist.linear.z = vx, vy, vz
        try:
            self.set_state(ms)
        except rospy.ServiceException as e:
            rospy.logwarn_throttle(2.0, "set_model_state failed: %s", e)

    def send_hover(self, on):
        if self.motor_pub is None:
            return
        msg = Actuators()
        w = HOVER_OMEGA if on else 0.0
        msg.angular_velocities = [w, w, w, w]
        self.motor_pub.publish(msg)

    def sample_row(self, phase, robot_type):
        l = self.latest
        if l.odom is None:
            return None
        p = l.odom.pose.pose.position
        o = l.odom.pose.pose.orientation
        tw = l.odom.twist.twist.linear
        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        sub = l.is_submerged
        # surface is the flat lake plane at z = 0 (wave amplitude = 0 in this field)
        surface_z = 0.0
        z_over_l = p.z / HULL_L
        cols = [
            "%.6f" % rospy.Time.now().to_sec(),
            "%.6f" % p.x, "%.6f" % p.y, "%.6f" % p.z,
            "%.6f" % tw.x, "%.6f" % tw.y, "%.6f" % tw.z,
            "%.6f" % math.degrees(roll), "%.6f" % math.degrees(pitch), "%.6f" % math.degrees(yaw),
            "%.6f" % l.restoring[0], "%.6f" % l.restoring[1], "%.6f" % l.restoring[2],
            "%.6f" % l.damping[0], "%.6f" % l.damping[1], "%.6f" % l.damping[2],
            "0.0", "0.0", "0.0",                                   # wave_* : UUV has no wave force
            "%.6f" % l.added_mass[0], "%.6f" % l.added_mass[1], "%.6f" % l.added_mass[2],
            "%.6f" % l.coriolis[0], "%.6f" % l.coriolis[1], "%.6f" % l.coriolis[2],
            "%.6f" % surface_z, "%.6f" % z_over_l,
            ("SUBMERGED" if sub else "AIR"),                        # UUV's (latched) view of phase
            robot_type,
        ]
        return ",".join(cols)

    # -- run one full mission ------------------------------------------------
    def run_trial(self, idx):
        rows = []
        dt = 1.0 / TICK_HZ
        rate = rospy.Rate(TICK_HZ)

        # reset to the very first waypoint
        first_t, first_poly = self.phases[0]
        fx, fy, fz = first_poly[0]
        for _ in range(10):
            self.teleport(fx, fy, fz, 0.0, 0.0, 0.0, 0.0)
            rospy.sleep(0.05)
        rospy.sleep(0.5)

        for phase_name, poly in self.phases:
            robot_type = phase_name
            dense = densify(poly, 0.25)
            speed = SPEED.get(phase_name, 1.0)
            air = (phase_name == "UAV")
            acc = SAMPLE_DIST          # force a sample at the first point of each phase
            prev = dense[0]
            for i in range(len(dense) - 1):
                a, b = dense[i], dense[i + 1]
                seg = math.dist(a, b)
                if seg < 1e-6:
                    continue
                ux = (b[0] - a[0]) / seg
                uy = (b[1] - a[1]) / seg
                uz = (b[2] - a[2]) / seg
                yaw = math.atan2(b[1] - a[1], b[0] - a[0])
                steps = max(1, int(math.ceil(seg / (speed * dt))))
                for s in range(1, steps + 1):
                    if rospy.is_shutdown():
                        return rows
                    t = s / float(steps)
                    x = a[0] + t * (b[0] - a[0])
                    y = a[1] + t * (b[1] - a[1])
                    z = a[2] + t * (b[2] - a[2])
                    self.teleport(x, y, z, yaw, speed * ux, speed * uy, speed * uz)
                    if air:
                        self.send_hover(True)        # RotorS rotors spinning during the aerial leg
                    acc += math.dist((x, y, z), prev)
                    prev = (x, y, z)
                    if acc >= SAMPLE_DIST:
                        acc = 0.0
                        row = self.sample_row(phase_name, robot_type)
                        if row:
                            rows.append(row)
                    rate.sleep()
            if air:
                self.send_hover(False)
            rospy.loginfo("[baseline] trial %d: finished %s (%d rows so far)",
                          idx, phase_name, len(rows))
        return rows

    # -- orchestrate ---------------------------------------------------------
    def run(self, n_trials):
        for idx in range(1, n_trials + 1):
            ts = time.time()
            bag_path = os.path.join(OUT_DIR, "baseline_run_%d_%.4f.bag" % (idx, ts))
            rospy.loginfo("=" * 60)
            rospy.loginfo("[baseline] TRIAL %d/%d  ->  %s", idx, n_trials, bag_path)
            bag = subprocess.Popen(
                ["rosbag", "record", "-O", bag_path] + BAG_TOPICS,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid)
            rospy.sleep(1.0)

            rows = self.run_trial(idx)

            # stop the bag cleanly
            try:
                os.killpg(os.getpgid(bag.pid), signal.SIGINT)
                bag.wait(timeout=10)
            except Exception:
                try:
                    os.killpg(os.getpgid(bag.pid), signal.SIGKILL)
                except Exception:
                    pass

            csv_path = os.path.join(OUT_DIR, "trajectory_data_with_forces_%.7f.csv" % time.time())
            with open(csv_path, "w") as f:
                f.write(CSV_HEADER + "\n")
                f.write("\n".join(rows) + "\n")
            rospy.loginfo("[baseline] trial %d saved %d rows -> %s", idx, len(rows), csv_path)
            rospy.sleep(1.0)

        rospy.loginfo("[baseline] ALL %d trials complete.", n_trials)


def main():
    n = 10
    if len(sys.argv) > 1:
        try:
            n = int(sys.argv[1])
        except ValueError:
            pass
    runner = BaselineRunner()
    runner.run(n)


if __name__ == "__main__":
    main()
