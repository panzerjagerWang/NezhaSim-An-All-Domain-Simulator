#!/usr/bin/env python3
# ============================================================================
#  NezhaSim — Hardware-in-the-Loop (HITL) control DEMO  (nezha_mini / nezha_f)
#
#  Author: Jiaqing "Lance" Wang <jiaqing.wang@sjtu.edu.cn>  — The Nezha Lab, SJTU
#
#  This single file acts as a minimal Ground Control Station for a PX4 flight
#  controller running HITL against NezhaSim, and demonstrates a full flight:
#      connect -> prime -> wait for healthy sensors+GPS -> STABILIZED ->
#      arm (zero throttle) -> take off from water -> hold altitude -> land ->
#      disarm.
#  It works for any HITL multirotor in the project (nezha_mini_hil, nezha_f_mav,
#  ...). Pick the model with --model.
#
#  ---------------------------------------------------------------------------
#  QUICK START
#  ---------------------------------------------------------------------------
#    # 1. (one time) on the FC via QGroundControl: SYS_HITL=1, a quad airframe,
#    #    COM_RC_IN_MODE=1, CBRK_SUPPLY_CHK=894281, CBRK_USB_CHK=197848, and
#    #    DISABLE QGC serial/USB autoconnect (critical — see DEBUG #1).
#    # 2. plug in the Pixhawk by USB, then:
#    cd ~/Nezha_ws && source devel/setup.bash
#    roslaunch nezha_gazebo nezha_mini_hil.launch gui:=false      # ocean world
#    # 3. in another terminal:
#    python3 src/nezha_sim/nezha_gazebo/scripts/hitl_demo.py --model nezha_mini_hil
#
#    Other useful invocations:
#      --check-only                 just connect + report sensor/GPS health
#      --model nezha_f_mav          fly the nezha_f instead
#      --alt 8 --hover-secs 12      target 8 m, hover 12 s
#
#  How it works: in HITL the gazebo plugin (libgazebo_mavlink_interface.so) owns
#  the FC serial port and forwards the FC's MAVLink to UDP 14550 (the "QGC"
#  link). This script binds 14550 and sprays heartbeats to gazebo's UDP sockets
#  so the plugin learns our address and PX4 starts streaming — i.e. we ARE the
#  GCS. (QGroundControl can do the same thing; only run one GCS at a time.)
#
# ============================================================================
#  ############################  HOW TO DEBUG HITL  ##########################
# ============================================================================
#  Symptom -> cause -> fix.  Run with --check-only first; it prints heartbeat,
#  GPS fix and which sensors are unhealthy.
#
#  0. SANITY: `lsof /dev/ttyACM0` must show ONLY `gzserver`. `ls /dev/ttyACM0`
#     must exist (user in `dialout`). Gazebo console must print
#     "Connecting to PX4 SITL using serial" then "Opened serial device".
#
#  1. *** QGroundControl stealing the serial port *** (the #1 gotcha)
#     If QGC's serial/USB autoconnect is ON it grabs /dev/ttyACM0 and fights the
#     gazebo plugin -> gazebo prints "Error opening serial device: ... busy", OR
#     sensors read unhealthy / "No GPS Lock" for EVERY model.
#     FIX: QGC -> Application Settings -> AutoConnect -> uncheck Serial/USB.
#          QGC then connects over the UDP link the plugin forwards (udp:14550).
#     NOTE: `pkill -x QGroundControl` does NOT kill the AppImage — use
#          `pkill -9 -f QGroundControl.AppImage`. Verify with `lsof /dev/ttyACM0`.
#
#  2. "connects but won't arm / no HIL / state never leaves 8"
#     PX4 not in HITL or no GCS connected. Check SYS_HITL=1. PX4's USB telemetry
#     only starts once a GCS heartbeats — this script (or QGC) provides that.
#
#  3. "Accel/Gyro/Compass/Baro 0 missing"  (all sensors unhealthy, GPS fix=0)
#     PX4 isn't receiving HIL_SENSOR. Either (a) the model uses RotorS sensor
#     plugins (gz_sensor_msgs::*) instead of the PX4 ones (sensor_msgs::msgs::*)
#     — gazebo transport silently won't link mismatched types; the HITL model
#     must use the px4_hitl macro (common/UAV/hitl_snippets.xacro), or (b) QGC is
#     holding the serial (see #1).
#
#  4. "MAG #0 failed: TIMEOUT" / "BARO #0 failed: TIMEOUT"
#     Gazebo real-time factor < 1, so sim-timestamped sensors lag the FC clock.
#     HITL is wall-clock (no lockstep) so the sim MUST keep up. Use the coarse
#     real-time worlds (nezha_hitl / nezha_hitl_ocean, ~0.9 RTF) and/or raise the
#     sensor pubRate. Check RTF: rosservice call /gazebo/get_world_properties.
#
#  5. Arms + "takeoff detected" but the sim vehicle never moves
#     Missing <control_channels> in the mavlink_interface plugin -> actuator
#     0..1 maps to ~0 rad/s (no thrust). The px4_hitl macro sets input_scaling.
#     Also the HITL rotor motor model must be the PX4 libgazebo_motor_model.so
#     (mav_msgs::msgs::CommandMotorSpeed), not the nezha/RotorS one.
#
#  6. "could not enter ALTITUDE/POSCTL mode" on the water
#     On the bobbing ocean waterline the EKF height is too noisy. Fly in
#     STABILIZED (this demo's default): attitude + manual throttle, no altitude
#     estimate needed.
#
#  7. "Arming denied: high throttle"
#     STABILIZED won't arm unless the throttle stick is at minimum. This demo
#     streams throttle=0 to arm, then ramps up. (RC/joystick: throttle down.)
#
#  8. "Preflight Fail: Flight termination active"  (won't recover on relaunch)
#     PX4 latched flight-termination (often from a too-aggressive climb). A
#     MAVLink reboot does NOT clear it — physically UNPLUG/REPLUG the Pixhawk USB
#     (it is USB-powered; a battery/reset cycle won't do it). Confirm a real
#     re-enumeration with: dmesg | grep 'usb 1-10'  (fresh disconnect/reconnect).
#     Keep climbs gentle (throttle <= ~700) to avoid tripping it.
#
#  9. "Device or resource busy" opening serial
#     A leftover process holds the port. pkill -9 -f QGroundControl.AppImage ;
#     pkill -x gzserver ; confirm `lsof /dev/ttyACM0` empty ; relaunch.
# ============================================================================

import argparse
import re
import socket
import subprocess
import threading
import time

from pymavlink import mavutil

# PX4 main-mode numbers used with MAV_CMD_DO_SET_MODE (param2)
PX4_MAIN_MANUAL, PX4_MAIN_ALTCTL, PX4_MAIN_POSCTL = 1, 2, 3
PX4_MAIN_STABILIZED = 7
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED            # 128
HIL_FLAG = mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED              # 32
SENSOR_BITS = {1 << 0: "GYRO", 1 << 1: "ACCEL", 1 << 2: "MAG",
               1 << 3: "BARO", 1 << 5: "GPS"}


def gazebo_udp_ports():
    """UDP ports gzserver is bound to — the mavlink_interface QGC/SDK sockets
    are among them. We spray heartbeats to all of them to bootstrap the link."""
    try:
        out = subprocess.run(["ss", "-unlp"], capture_output=True, text=True).stdout
    except FileNotFoundError:
        out = ""
    ports = set()
    for line in out.splitlines():
        if "gz" in line:
            m = re.search(r"0\.0\.0\.0:(\d+)", line)
            if m:
                ports.add(int(m.group(1)))
    return sorted(ports)


class HitlGcs:
    """Minimal GCS over the gazebo mavlink_interface's UDP 14550 forward."""

    def __init__(self, qgc_port=14550):
        self.ports = gazebo_udp_ports()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", qgc_port))
        self.sock.settimeout(0.1)
        self.mav = mavutil.mavlink.MAVLink(None)
        self.mav.srcSystem, self.mav.srcComponent = 255, 190
        self.peer = None
        self.throttle = 0          # MANUAL_CONTROL z: 0..1000 (0 = idle)
        self.run = True
        self.lock = threading.Lock()
        # latest vehicle state from heartbeats / sys_status
        self.main_mode = None
        self.armed = False
        self.state = None
        self.gps = None            # (fix_type, sats)
        self.unhealthy = None      # list of unhealthy sensor names
        self.got_hb = False
        self._seen_text = set()
        threading.Thread(target=self._streamer, daemon=True).start()
        threading.Thread(target=self._reader, daemon=True).start()

    # ---- outbound -----------------------------------------------------------
    def _send(self, buf):
        for p in self.ports:                       # spray = bootstrap priming
            try:
                self.sock.sendto(buf, ("127.0.0.1", p))
            except OSError:
                pass
        if self.peer:                              # and to the learned peer
            try:
                self.sock.sendto(buf, self.peer)
            except OSError:
                pass

    def _hb(self):
        return self.mav.heartbeat_encode(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0).pack(self.mav)

    def _manual(self):
        # x,y,z(throttle),r ; buttons. We only drive throttle (z) here.
        return self.mav.manual_control_encode(1, 0, 0, int(self.throttle), 0, 0).pack(self.mav)

    def _streamer(self):
        # 20 Hz GCS heartbeat + manual control keeps PX4 happy (no datalink/
        # manual-control-loss failsafe) and primes its USB stream.
        while self.run:
            with self.lock:
                self._send(self._hb())
                self._send(self._manual())
            time.sleep(0.05)

    def cmd_long(self, cmd, *params):
        params = list(params) + [0] * (7 - len(params))
        with self.lock:
            self._send(self.mav.command_long_encode(1, 1, cmd, 0, *params).pack(self.mav))

    # ---- inbound ------------------------------------------------------------
    def _reader(self):
        while self.run:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            self.peer = addr
            try:
                msgs = self.mav.parse_buffer(data) or []
            except Exception:
                msgs = []
            for m in msgs:
                t = m.get_type()
                if t == "HEARTBEAT" and m.get_srcComponent() == 1:
                    self.got_hb = True
                    self.main_mode = (m.custom_mode >> 16) & 0xFF
                    self.armed = bool(m.base_mode & ARMED_FLAG)
                    self.state = m.system_status
                elif t == "GPS_RAW_INT":
                    self.gps = (m.fix_type, m.satellites_visible)
                elif t == "SYS_STATUS":
                    en, h = m.onboard_control_sensors_enabled, m.onboard_control_sensors_health
                    self.unhealthy = [n for b, n in SENSOR_BITS.items() if (en & b) and not (h & b)]
                elif t == "STATUSTEXT":
                    txt = m.text if isinstance(m.text, str) else m.text.decode(errors="ignore")
                    if txt not in self._seen_text:
                        self._seen_text.add(txt)
                        print(f"   PX4[{m.severity}]: {txt.strip()}")

    # ---- high-level ---------------------------------------------------------
    def wait_heartbeat(self, timeout=40):
        end = time.time() + timeout
        while time.time() < end and not self.got_hb:
            time.sleep(0.2)
        return self.got_hb

    def request_streams(self):
        self.cmd_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 33, 100000)   # local pos
        self.cmd_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 24, 200000)   # gps
        self.cmd_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 1, 500000)    # sys_status

    def wait_healthy(self, timeout=90):
        """Healthy = 3D GPS fix and no unhealthy enabled sensors."""
        self.request_streams()
        end = time.time() + timeout
        while time.time() < end:
            fix = self.gps[0] if self.gps else 0
            if fix >= 3 and self.unhealthy == []:
                return True
            time.sleep(1.0)
        return False

    def set_mode(self, main_mode, timeout=12):
        end = time.time() + timeout
        while time.time() < end:
            self.cmd_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, main_mode, 0)
            for _ in range(5):
                time.sleep(0.1)
                if self.main_mode == main_mode:
                    return True
        time.sleep(1.5)
        return self.main_mode == main_mode

    def arm(self, timeout=8):
        end = time.time() + timeout
        while time.time() < end:
            self.cmd_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0)
            time.sleep(0.4)
            if self.armed:
                return True
        return self.armed

    def disarm(self):
        # normal disarm first; then force (param2=21196) as a fallback — on the
        # ocean the vehicle bobs on waves so PX4's land-detector may refuse a
        # normal disarm ("Not landed"). Force is safe here (HITL, at waterline).
        for _ in range(4):
            self.cmd_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0)
            time.sleep(0.25)
            if not self.armed:
                return
        for _ in range(6):
            self.cmd_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 21196)
            time.sleep(0.25)
            if not self.armed:
                return

    def close(self):
        self.run = False
        time.sleep(0.2)
        self.sock.close()


# --- optional gazebo ground-truth height (for the demo's altitude feedback) ---
def make_height_fn(model):
    try:
        import rospy
        from gazebo_msgs.srv import GetModelState
        rospy.init_node("hitl_demo", anonymous=True, disable_signals=True)
        rospy.wait_for_service("/gazebo/get_model_state", timeout=8)
        srv = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        def height():
            try:
                return srv(model, "").pose.position.z
            except Exception:
                return None
        return height
    except Exception as e:
        print(f"[demo] gazebo height feedback unavailable ({e}); using timed climb")
        return None


def fly_demo(gcs, height, target_alt, hover_secs):
    """STABILIZED takeoff -> altitude hold (via gazebo z feedback) -> land."""
    print(f"[demo] entering STABILIZED ...")
    if not gcs.set_mode(PX4_MAIN_STABILIZED):
        print("[demo] ABORT: could not enter STABILIZED (see DEBUG #6/#8)")
        return False
    gcs.throttle = 0                      # minimum throttle so arming is allowed
    print("[demo] arming (throttle at minimum) ...")
    if not gcs.arm():
        print("[demo] ABORT: arm denied (see DEBUG #7/#8)")
        return False
    print(f"[demo] ARMED in mode={gcs.main_mode}. Taking off to ~{target_alt} m")

    # Proportional altitude hold on gazebo ground-truth z (STABILIZED throttle
    # = direct thrust). HOVER_BASE is ~hover throttle for these light HITL
    # quads; KP nudges around it. Clamped to gentle limits so a climb can't run
    # away (a full-throttle runaway can trip flight-termination).
    HOVER_BASE, KP, TMIN, TMAX = 380, 35, 150, 560
    last_z, last_t = None, time.time()
    t_end = time.time() + 8 + hover_secs
    while time.time() < t_end:
        z = height() if height else None
        if z is None:                     # no feedback: gentle open-loop climb
            gcs.throttle = 470
        else:
            # add a little damping on vertical rate to avoid overshoot
            now = time.time()
            vz = 0.0 if last_z is None else (z - last_z) / max(1e-3, now - last_t)
            last_z, last_t = z, now
            gcs.throttle = int(max(TMIN, min(TMAX,
                              HOVER_BASE + KP * (target_alt - z) - 40 * vz)))
        zs = f"{z:+.2f}" if z is not None else "n/a"
        print(f"[demo]  z={zs} m  throttle={gcs.throttle}  armed={gcs.armed} mode={gcs.main_mode}")
        time.sleep(1.0)

    print("[demo] landing ...")
    for _ in range(60):
        z = height() if height else None
        if z is not None and z < 0.3:
            break
        gcs.throttle = 180                # well below hover -> descend
        time.sleep(0.5)
    gcs.throttle = 0
    time.sleep(3)                         # let PX4's land-detector confirm 'landed'
    print("[demo] disarming ...")
    gcs.disarm()
    print("[demo] done.")
    return True


def main():
    ap = argparse.ArgumentParser(description="NezhaSim HITL control demo")
    ap.add_argument("--model", default="nezha_mini_hil",
                    help="gazebo model name (nezha_mini_hil, nezha_f_mav, ...)")
    ap.add_argument("--alt", type=float, default=5.0, help="target altitude (m)")
    ap.add_argument("--hover-secs", type=float, default=8.0)
    ap.add_argument("--check-only", action="store_true",
                    help="only connect and report health, do not fly")
    args = ap.parse_args()

    print(f"[demo] gazebo UDP ports: {gazebo_udp_ports()}")
    gcs = HitlGcs()
    try:
        print("[demo] waiting for PX4 heartbeat (priming the link) ...")
        if not gcs.wait_heartbeat():
            print("[demo] NO HEARTBEAT — is gazebo HITL running and is QGC off the "
                  "serial port? See DEBUG #0/#1.")
            return
        print(f"[demo] connected. HITL flag will show in mode; waiting for healthy sensors+GPS ...")
        healthy = gcs.wait_healthy()
        print(f"[demo] GPS={gcs.gps}  unhealthy_sensors={gcs.unhealthy}  "
              f"armed={gcs.armed}  state={gcs.state}")
        if not healthy:
            print("[demo] NOT healthy. See DEBUG #3 (HIL_SENSOR) / #4 (RTF) / #8 "
                  "(flight-termination needs USB power-cycle).")
            return
        print("[demo] HEALTHY (3D GPS + all sensors).")
        if args.check_only:
            return
        height = make_height_fn(args.model)
        fly_demo(gcs, height, args.alt, args.hover_secs)
    except KeyboardInterrupt:
        print("\n[demo] interrupted — disarming")
        gcs.throttle = 0
        gcs.disarm()
    finally:
        gcs.close()


if __name__ == "__main__":
    main()
