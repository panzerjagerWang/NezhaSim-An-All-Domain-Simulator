#!/usr/bin/env python3
"""Run one closed-loop nezha_mini water-exit trial and log it.

The initial condition is drawn from a seed, so both arms (PSC and PSC-RS) see
the same 30 initial conditions. The trial is logged to <out>/<arm>_<seed>/.

Usage: run_trial.py --arm psc|psc_rs --seed N --out DIR [--port 11411]
"""
import argparse
import csv
import json
import os
import signal
import subprocess
import threading
import time

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--arm", choices=["psc", "psc_rs"], required=True)
ap.add_argument("--seed", type=int, required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--port", type=int, default=11411)
ap.add_argument("--world", default="nezha_lake_waves.world")
ap.add_argument("--duration", type=float, default=28.0)   # s after the first command
ap.add_argument("--z_target", type=float, default=1.5)    # m above mean water level
ap.add_argument("--profile", action="store_true")
args = ap.parse_args()

# Initial condition: horizontal position (sets the local wave phase), depth of
# the body center and command time. Identical for both arms of one seed.
rng = np.random.default_rng(1000 + args.seed)
x0, y0 = (float(v) for v in rng.uniform(-3.0, 3.0, 2))
z0 = float(rng.uniform(-0.8, -0.3))
t_cmd = float(rng.uniform(4.0, 8.0))

# Mission after t_cmd: exit and hover at z_target for 10 s, descend to
# z_dive for 8 s (water entry, floating), exit and hover again for 10 s.
Z_DIVE = -0.3
def setpoint_z(t_rel, z_target):
    if t_rel < 10.0:
        return z_target
    if t_rel < 18.0:
        return Z_DIVE
    return z_target

run_dir = os.path.join(args.out, f"{args.arm}_{args.seed:02d}")
os.makedirs(run_dir, exist_ok=True)
os.environ["ROS_MASTER_URI"] = f"http://localhost:{args.port}"
os.environ["GAZEBO_MASTER_URI"] = f"http://localhost:{args.port + 100}"
if args.profile:
    prof_dir = os.path.join(run_dir, "profile")
    os.makedirs(prof_dir, exist_ok=True)
    os.environ["NEZHA_PROFILE_DIR"] = prof_dir

log = open(os.path.join(run_dir, "launch.log"), "w")
cmd = ["roslaunch", "-p", str(args.port), "--wait", "nezha_gazebo", "nezha_mini_psc_trial.launch",
       f"world_file:=$(find nezha_gazebo)/worlds/{args.world}",
       f"rigid_switching:={'true' if args.arm == 'psc_rs' else 'false'}",
       f"x:={x0:.4f}", f"y:={y0:.4f}", f"z:={z0:.4f}"]
roscore = subprocess.Popen(["roscore", "-p", str(args.port)], stdout=log, stderr=log,
                           preexec_fn=os.setsid)
launch = subprocess.Popen(cmd, stdout=log, stderr=log, preexec_fn=os.setsid)


def shutdown():
    for p in (launch, roscore):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
    t_end = time.time() + 15
    while time.time() < t_end and (launch.poll() is None or roscore.poll() is None):
        time.sleep(0.2)
    for p in (launch, roscore):
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    log.close()


meta = dict(arm=args.arm, seed=args.seed, x0=x0, y0=y0, z0=z0, t_cmd=t_cmd,
            z_target=args.z_target, z_dive=Z_DIVE, duration=args.duration, world=args.world,
            status="started")
try:
    import rospy
    from geometry_msgs.msg import PoseStamped
    from gazebo_msgs.srv import GetModelState
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Empty
    from nezha_plugins.srv import GetPhaseSample, HydrodynamicsForces

    time.sleep(3.0)
    rospy.init_node("psc_trial_runner", anonymous=True, disable_signals=True)
    rospy.wait_for_service("/gazebo/unpause_physics", timeout=120)
    get_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    t0 = time.time()
    while time.time() - t0 < 120:
        try:
            if get_state("nezha_mini", "world").success:
                break
        except rospy.ServiceException:
            pass
        time.sleep(0.5)
    else:
        raise RuntimeError("nezha_mini was not spawned")
    rospy.wait_for_service("/nezha_mini/get_hydrodynamics_forces", timeout=60)
    rospy.wait_for_service("/nezha_mini/transmedia/get_phase_sample", timeout=60)
    pub = rospy.Publisher("/nezha_mini/command/pose", PoseStamped, queue_size=10)
    t0 = time.time()
    while pub.get_num_connections() == 0 and time.time() - t0 < 60:
        time.sleep(0.2)
    time.sleep(1.0)

    sim = {"t": 0.0}
    odom_rows, mt_rows, phase_rows = [], [], []

    def on_clock(msg):
        sim["t"] = msg.clock.to_sec()

    def on_odom(m):
        p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
        odom_rows.append((m.header.stamp.to_sec(), p.x, p.y, p.z, v.x, v.y, v.z,
                          q.x, q.y, q.z, q.w))

    rospy.Subscriber("/clock", Clock, on_clock, queue_size=100)
    rospy.Subscriber("/nezha_mini/ground_truth/odometry", Odometry, on_odom, queue_size=1000)
    mt = rospy.ServiceProxy("/nezha_mini/get_hydrodynamics_forces", HydrodynamicsForces,
                            persistent=True)
    phase = rospy.ServiceProxy("/nezha_mini/transmedia/get_phase_sample", GetPhaseSample,
                               persistent=True)
    t_stop = t_cmd + args.duration
    stop = threading.Event()

    def sample_mt():
        while not stop.is_set():
            try:
                r = mt()
                mt_rows.append((r.sim_time, r.buoyancy_z, r.damping_x, r.damping_y,
                                r.damping_z, r.wave_z, r.heave_z, r.submersion_ratio))
            except Exception:
                pass
            time.sleep(0.005)

    def sample_phase():
        while not stop.is_set():
            try:
                r = phase()
                phase_rows.append((sim["t"], r.surface_z, r.z_over_l, r.phase_name))
            except Exception:
                pass
            time.sleep(0.05)

    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()
    wall_start, sim_start = time.time(), sim["t"]
    threads = [threading.Thread(target=sample_mt, daemon=True),
               threading.Thread(target=sample_phase, daemon=True)]
    for th in threads:
        th.start()

    sp = PoseStamped()
    sp.header.frame_id = "world"
    sp.pose.position.x, sp.pose.position.y, sp.pose.position.z = x0, y0, args.z_target
    sp.pose.orientation.w = 1.0
    wall_deadline = time.time() + 3 * (t_stop + 10)
    while sim["t"] < t_stop and time.time() < wall_deadline:
        if sim["t"] >= t_cmd:
            sp.pose.position.z = setpoint_z(sim["t"] - t_cmd, args.z_target)
            sp.header.stamp = rospy.Time.now()
            pub.publish(sp)
        time.sleep(0.05)
    stop.set()
    for th in threads:
        th.join(timeout=2)
    meta["status"] = "done" if sim["t"] >= t_stop else "wall_timeout"
    meta["sim_end"] = sim["t"]
    meta["wall_elapsed"] = time.time() - wall_start
    meta["rtf"] = (sim["t"] - sim_start) / meta["wall_elapsed"]

    with open(os.path.join(run_dir, "odom.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw"])
        w.writerows(odom_rows)
    with open(os.path.join(run_dir, "mt.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "buoy_z", "damp_x", "damp_y", "damp_z", "wave_z", "heave_z", "w"])
        w.writerows(mt_rows)
    with open(os.path.join(run_dir, "phase.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "surface_z", "z_over_l", "phase"])
        w.writerows(phase_rows)
except Exception as exc:  # recorded, the trial is then rerun or reported
    meta["status"] = f"error: {exc!r}"
finally:
    shutdown()
    with open(os.path.join(run_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
    print(json.dumps(meta))
