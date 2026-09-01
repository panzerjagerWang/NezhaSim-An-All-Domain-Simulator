# NezhaSim — Hardware-in-the-Loop (HITL) with PX4

This guide explains how to run NezhaSim with a **real PX4 flight controller in the
loop** (tested with a USB Pixhawk 6C), and how to convert any multirotor robot in
the project to HITL.

In HITL, PX4 firmware runs on the **real hardware**. Gazebo simulates the vehicle
and its sensors; the simulated sensor readings are streamed to the FC as
`HIL_SENSOR`/`HIL_GPS`, and the FC's `HIL_ACTUATOR_CONTROLS` drive the simulated
motors. You fly it from QGroundControl (or RC) exactly like a real vehicle.

---

## 1. Architecture

We use the **native PX4 path**: the Gazebo `libgazebo_mavlink_interface.so`
plugin opens the FC serial port directly and speaks the HIL protocol.

```
 Gazebo (sim physics + PX4 sensor plugins)
   │  HIL_SENSOR / HIL_GPS          ▲  HIL_ACTUATOR_CONTROLS
   ▼                                │
 libgazebo_mavlink_interface.so  ── serial /dev/ttyACM0 @921600 ──►  PX4 (Pixhawk 6C)
   │  forwards GCS traffic
   ▼  UDP 14550
 QGroundControl  (arm / mode / fly)
```

There is **no MAVROS and no bridge script** in this path. (The old
`hitl_bridge.py` / `nezha_f_mav.launch` mixed a second, conflicting design — don't
use them for HITL.)

---

## 2. Prerequisites

- PX4 firmware flashed on the FC (v1.14.x used here). Source/binary in `~/下载/PX4-Autopilot`.
- QGroundControl (`~/QGroundControl.AppImage`).
- User in the `dialout` group (for `/dev/ttyACM0`).
- Workspace built: `catkin build` (the PX4 gazebo plugins `libgazebo_{imu,gps,
  magnetometer,barometer,groundtruth,motor,mavlink_interface}_plugin.so` must be
  in `devel/lib`).

---

## 3. One-time flight-controller setup (in QGroundControl)

Connect the FC by USB to QGC, set these, then **reboot the FC**:

| Param | Value | Why |
|---|---|---|
| `SYS_HITL` | `1` | Enable HITL mode |
| Airframe | HIL Quadcopter X (or any quad) | 4-rotor output |
| `COM_RC_IN_MODE` | `1` or `4` | Allow arming without physical RC |
| `CBRK_SUPPLY_CHK` | `894281` | No power module |
| `CBRK_USB_CHK` | `197848` | Allow arming over USB |
| `CBRK_FLIGHTTERM` | `121212` | Disable flight-termination (else aggressive climbs / mag-timeouts latch it and you must physically re-plug USB) |
| `COM_ARM_MAG_ANG` | `-1` | Don't block arming on HIL-mag vs EKF yaw mismatch |

Then **QGC → Application Settings → AutoConnect → uncheck Serial/USB**. This is
critical: otherwise QGC grabs `/dev/ttyACM0` and the Gazebo plugin can't open it.
QGC will instead connect over the UDP link the plugin forwards (`udp://:14550`).

---

## 4. Running HITL

Per-robot launches (each defaults to the lightweight real-time world `nezha_hitl`):

```bash
cd ~/Nezha_ws && source devel/setup.bash

# 1. Power the FC over USB (QGC will NOT auto-connect to serial).
# 2. Launch the sim onto the FC:
roslaunch nezha_gazebo nezha_f_hitl.launch        # nezha_f_mav      (VERIFIED)
#   roslaunch nezha_gazebo nezha_mini_hil.launch    # nezha_mini_hil  (VERIFIED)
#   roslaunch nezha_gazebo nezha_husky_hil.launch   # nezha_husky_hil
# 3. Open QGroundControl. It connects over UDP, shows the vehicle + "HITL".
# 4. Arm and take off in QGC (Altitude mode needs no GPS; Position/Auto need GPS).
```

Watch the Gazebo console for `Connecting to PX4 SITL using serial` →
`Opened serial device /dev/ttyACM0`. `lsof /dev/ttyACM0` should show only
`gzserver`.

**A GCS must be connected for HITL to run.** PX4's USB telemetry (and the HIL
stream) only activates once it receives GCS heartbeats — the Gazebo plugin is a
simulator client, not a GCS. So always have QGroundControl (or the scripted GCS
in §7) running.

Headless / scripted flight without QGC — the all-in-one demo (verified on both
models; has a full in-file "HOW TO DEBUG HITL" guide):
```bash
roslaunch nezha_gazebo nezha_mini_hil.launch gui:=false        # or nezha_f_hitl.launch
python3 src/nezha_sim/nezha_gazebo/scripts/hitl_demo.py --model nezha_mini_hil
#   --model nezha_f_mav    --alt 5 --hover-secs 8    --check-only
# Does: connect -> wait healthy (3D GPS) -> STABILIZED -> arm -> take off from
# water -> proportional altitude-hold -> land -> disarm.
```

---

## 5. How to switch ANY multirotor to HITL  ← "where do I change?"

Everything HITL-specific is gated on the launch/xacro arg
**`enable_mavlink_interface`** (true in the `*_hitl.launch` files). Three places:

**(a) The robot's `UAV/UAV.xacro`** — swap the RotorS sensors/controller for the
shared PX4 HITL macro:

```xml
<xacro:include filename="$(find nezha_description)/common/UAV/hitl_snippets.xacro"/>

<!-- non-HITL: keep the RotorS controller + IMU -->
<xacro:unless value="$(arg enable_mavlink_interface)">
  <xacro:controller_plugin_macro namespace="${namespace}" imu_sub_topic="imu"/>
  <xacro:default_imu namespace="${namespace}" parent_link="${namespace}/base_link"/>
</xacro:unless>

<!-- HITL: PX4 sensor suite + GPS + mavlink_interface (opens the serial port) -->
<xacro:if value="$(arg enable_mavlink_interface)">
  <xacro:px4_hitl namespace="${namespace}" rotor_count="4"/>
</xacro:if>
```

**(b) `common/UAV/UAV_plugin.xacro`** — already done once for all robots: the
`vertical_rotor` macro uses the PX4 `libgazebo_motor_model.so` when
`enable_mavlink_interface` is true, else the nezha motor model.

**(c) A launch file** — copy `nezha_f_hitl.launch`, change `mav_name`, and keep
`enable_mavlink_interface:=true` + `world_name:=nezha_hitl` + `use_sim_time:=false`.

> **HITL is opt-in.** `spawn_mav.launch` defaults `enable_mavlink_interface` to
> **false**, so every *plain* (non-HITL) launch spawns the RotorS xacro. Only the
> dedicated `*_hitl` / `*_hil` / `*_mav` launches pass `:=true`. Give each HITL
> robot its own model name (`nezha_f_mav`, `nezha_mini_hil`, `nezha_husky_hil`)
> rather than toggling the plain model — the dedicated clone also omits the
> RotorS ground-truth IMU, which otherwise collides with the PX4 IMU and kills
> the HIL sensor feed.

That's it. The shared macro `px4_hitl` (in `common/UAV/hitl_snippets.xacro`)
contains the IMU/mag/baro/groundtruth/GPS plugins and the `mavlink_interface`
with the `<control_channels>` that scale actuator outputs to rotor speed.

`px4_hitl` parameters (defaults shown): `rotor_count:=4`,
`serial_device:=/dev/ttyACM0`, `baud_rate:=921600`, and the reference
`ref_lat/ref_lon/ref_alt` + `ref_mag_*` (Zurich; keep consistent with the world's
`<spherical_coordinates>`).

---

## 6. Why these pieces matter (the non-obvious gotchas)

These cost real debugging time — they are the difference between "connects but
won't arm" and a clean flight:

1. **Sensor/motor protobuf types must match.** The default Nezha/RotorS plugins
   publish `gz_sensor_msgs::*` / `gz_mav_msgs::CommandMotorSpeed`; PX4's
   `mavlink_interface` consumes `sensor_msgs::msgs::*` /
   `mav_msgs::msgs::CommandMotorSpeed`. Gazebo transport silently never connects
   mismatched types. HITL therefore uses the **PX4** sensor + motor plugins, not
   the RotorS/nezha ones — never both on the same topic.

2. **`<control_channels>` are mandatory.** Without them `input_scaling=0`, so
   actuator commands map to ~0 rad/s and the vehicle never lifts (PX4 may still
   report "takeoff detected"). The macro sets `input_scaling=1000`,
   `zero_position_armed=100`.

3. **Real-time factor ≈ 1.0.** HITL is wall-clock (no lockstep), so if Gazebo
   runs slow the sim-timestamped sensors lag the FC's clock and PX4 times out
   mag/baro. The `nezha_hitl.world` uses no water + coarse physics (5 ms step,
   200 Hz, 20 solver iters) to hold RTF ~1.0. Heavy/transmedia models in the lake
   world drop to ~0.4 → sensor timeouts.

4. **One owner of the serial port.** Only the Gazebo plugin may open
   `/dev/ttyACM0`. Disable QGC serial auto-connect; don't run MAVROS on serial.

5. **GPS** needs the world's `<spherical_coordinates>` (origin) and a
   `gps`-named joint+sensor (the macro provides `gps0`). Position/Auto modes need
   it; Altitude/Stabilized fly without.

---

## 7. Test / debug utilities (`nezha_gazebo/scripts/`)

- `hitl_gcs_probe.py` — acts as a minimal GCS on UDP 14550: primes the FC and
  prints state, GPS fix, and per-sensor health. First thing to run to see *why*
  it won't arm.
- `hitl_flight_test.py` — no-GPS autonomous flight: streams `MANUAL_CONTROL` +
  Altitude mode, arms, climbs, lands. Verify the climb with Gazebo's
  `get_model_state` on the model's z.

Quick health check:
```bash
python3 src/nezha_sim/nezha_gazebo/scripts/hitl_gcs_probe.py 15
# -> got_px4_heartbeat=True  GPS(fix,sats)=(3,10)  unhealthy_sensors=[]   == good
```

---

## 8. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| Plugin can't open `/dev/ttyACM0` | QGC serial autoconnect on, or MAVROS on serial. Disable both. |
| Connects but won't arm / "no HIL" | `SYS_HITL≠1`, or no GCS connected (PX4 not streaming). |
| `Accel/Gyro/Compass/Baro missing` | PX4 not getting `HIL_SENSOR` — RotorS sensors instead of PX4 ones, or topic mismatch. |
| `MAG/BARO #0 failed: TIMEOUT` | RTF < 1. Use `nezha_hitl.world`; raise sensor `pubRate`. |
| Arms + "takeoff detected" but sim doesn't move | Missing/incorrect `<control_channels>` (no thrust). |
| Failsafe immediately after arm | No control source (RC/joystick) or no GPS for the mode. Use QGC virtual joystick + Altitude mode, or add GPS for Position/Auto. |
| No telemetry on a fresh boot | PX4 needs a GCS heartbeat to start streaming — connect QGC or run `hitl_gcs_probe.py`. |
| ALL sensors unhealthy + "No GPS Lock" in QGC, for *every* model | **#1 cause: QGroundControl's serial autoconnect grabbed `/dev/ttyACM0`** and is fighting the gazebo plugin. Disable QGC serial autoconnect; verify `lsof /dev/ttyACM0` shows ONLY gzserver. (`pkill -x QGroundControl` does NOT kill the AppImage — use `pkill -9 -f QGroundControl.AppImage`.) |
| Sensors unhealthy that a relaunch won't fix (and QGC isn't holding the port) | FC HIL session wedged — usually after an over-aggressive scripted flight tripped PX4 flight-termination (latches). **Physically power-cycle the Pixhawk USB.** Avoid throttle >~750 in `hitl_flight_test.py`. |
| `Error opening serial device: open: Device or resource busy` | Another process holds `/dev/ttyACM0` (lingering QGC/gzserver). `pkill -9 -f QGroundControl.AppImage`, `pkill -x gzserver`, confirm `lsof /dev/ttyACM0` empty, relaunch. |

---

## Status

Both default to the **HAUV ocean world** `nezha_hitl_ocean.world` (includes
`nezha_ocean_waves` + lake bottom + GPS spherical coords, coarse physics
RTF ~0.9). Pass `world_name:=nezha_hitl` for the lightweight no-water aerial world.

- **`nezha_f_mav`** (`nezha_f_hitl.launch`) — fully verified in the **ocean
  world**: healthy sensors + 3D GPS fix, armed, took off from the water surface,
  climbed to 26.5 m.
- **`nezha_mini_hil`** (`nezha_mini_hil.launch`) — DEDICATED clean HITL model
  (cloned from nezha_mini, namespace `nezha_mini_hil`; clone-namespace bug fixed).
  Fully verified in the **ocean world**: healthy sensors + 3D GPS, armed in
  STABILIZED, took off from the water, climbed to 12.9 m, descended, landed,
  disarmed.

**Water-takeoff flight tips (light HAUV on waves):** on the bobbing waterline the
EKF height is too noisy for ALTITUDE/POSCTL — use **STABILIZED** mode. STABILIZED
arming requires the **throttle at minimum**, so (with a joystick/RC or scripted
MANUAL_CONTROL) hold throttle down to arm, then raise it to climb. Keep climbs
gentle — a runaway full-throttle climb can trip PX4 flight-termination (which
then needs a physical USB power-cycle to clear).
- **`nezha_husky_hil`** (`nezha_husky_hil.launch`) — DEDICATED clean HITL model
  (cloned from nezha_husky, namespace `nezha_husky_hil`; ground-truth IMU omitted
  in HITL). Flies the transmedia husky in UAV/multirotor mode. Xacro validated in
  both modes; not yet flown on hardware. For ground-rover HITL use PX4 rover
  firmware (a different airframe).

> The earlier in-place toggle launches (`nezha_mini_hitl.launch`,
> `nezha_husky_hitl.launch`) that HITL-ized the *plain* `nezha_mini` / `nezha_husky`
> models have been removed. HITL now always uses a dedicated `*_hil` / `*_mav`
> model so the plain launches stay pure simulation.

### Cloning a robot to a dedicated `*_hil` model — gotcha
When you clone a robot dir for a dedicated HITL model, the namespace appears
**hardcoded** (not `${namespace}`) in `UUV/UUV_baselink.xacro` and
`plugins.xacro`. You MUST update those to the new namespace, or base_link is
built under the old namespace while the rotors/PX4 plugins use the new one →
spawn fails with *"parent link ... not found."*

### Verifying after an FC power-cycle
```bash
roslaunch nezha_gazebo nezha_mini_hil.launch gui:=false
python3 src/nezha_sim/nezha_gazebo/scripts/hitl_gcs_probe.py 15
# healthy == GPS(fix,sats)=(3,N)  unhealthy_sensors=[]
python3 src/nezha_sim/nezha_gazebo/scripts/hitl_flight_test.py   # should climb ~2-3 m
```
Or simply open QGroundControl: a GPS lock + green sensor indicators confirm it.
