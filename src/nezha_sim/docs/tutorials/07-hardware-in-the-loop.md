# Tutorial 7 — Hardware-in-the-loop (advanced)

> **This tutorial needs real hardware** — a USB-connected **PX4 Pixhawk 6C**
> flight controller. If you just want to learn the simulator, you can stop after
> Tutorial 6.

**Goal:** drive a *real* PX4 flight controller from the simulation. Gazebo feeds
the Pixhawk simulated IMU/mag/baro/GPS over the USB serial port, the firmware
runs its real control loops, and its motor commands fly the simulated vehicle —
**hardware-in-the-loop (HITL)**.

The complete reference (architecture, FC parameters, debugging) is
[`HITL_SETUP.md`](../../HITL_SETUP.md). This tutorial is the short version.

---

## 1. How HITL differs from the sim launches

Two rules make the whole system make sense:

1. **HITL is opt-in.** Plain launches (`nezha_f.launch`, the husky/mini sim
   launches) run pure simulation — no flight controller is ever opened. Only the
   dedicated HITL launches pass `enable_mavlink_interface:=true`.
2. **Each HITL robot is a separately-named model**, never the plain model with a
   flag toggled:

   | Sim model | HITL model | HITL launch |
   |-----------|------------|-------------|
   | `nezha_f`     | `nezha_f_mav`     | `nezha_f_hitl.launch` |
   | `nezha_mini`  | `nezha_mini_hil`  | `nezha_mini_hil.launch` |
   | `nezha_husky` | `nezha_husky_hil` | `nezha_husky_hil.launch` |

   The dedicated clone omits the RotorS ground-truth IMU, which would otherwise
   collide with the PX4 IMU and silently kill the HIL sensor feed.

## 2. Flight-controller prerequisites (once, via QGroundControl)

Set these on the Pixhawk and reboot it:

| Parameter | Value | Why |
|-----------|-------|-----|
| `SYS_HITL` | `1` | enable HITL |
| `COM_RC_IN_MODE` | `1` or `4` | no physical RC required |
| `CBRK_SUPPLY_CHK` | `894281` | bypass power-module check (no battery) |
| `CBRK_USB_CHK` | `197848` | allow arming on USB |

And in **QGroundControl ▸ Application Settings ▸ AutoConnect**, **uncheck Serial**
so QGC doesn't grab `/dev/ttyACM0` and fight the Gazebo plugin for the port. See
`HITL_SETUP.md` for the full list (including the flight-termination breakers).

## 3. Run it

```bash
# 1. Power the Pixhawk over USB.
# 2. Launch the sim onto the FC (verified models):
roslaunch nezha_gazebo nezha_f_hitl.launch        # nezha_f_mav
#   roslaunch nezha_gazebo nezha_mini_hil.launch    # nezha_mini_hil
#   roslaunch nezha_gazebo nezha_husky_hil.launch   # nezha_husky_hil (xacro-validated)
```

Watch the Gazebo console for `Connecting to PX4 SITL using serial` →
`Opened serial device /dev/ttyACM0`. Confirm only `gzserver` holds the port:

```bash
lsof /dev/ttyACM0     # should list gzserver and nothing else
```

## 4. Fly it without QGC (scripted)

A GCS must send heartbeats for PX4's HIL stream to start. The all-in-one demo
does that and runs a full take-off → hover → land:

```bash
python3 src/nezha_sim/nezha_gazebo/scripts/hitl_demo.py --model nezha_f_mav
#   --model nezha_mini_hil    --alt 5 --hover-secs 8    --check-only
```

`--check-only` just verifies sensor health + GPS lock without flying — the best
first test. The script has an in-file **"HOW TO DEBUG HITL"** block covering the
common symptoms.

## 5. When something won't arm or shows "all sensors unhealthy"

Nearly always one of:

- **QGC grabbed the serial port** — `pkill -9 -f QGroundControl.AppImage`, and
  make sure serial autoconnect is off.
- **`SYS_HITL` not set / FC not rebooted** after setting it.
- **Flight-termination latched** (from a previous runaway) — needs a **physical
  USB unplug/replug**; a MAVLink reboot won't clear it. Setting
  `CBRK_FLIGHTTERM=121212` stops it recurring.

The full troubleshooting matrix is in [`HITL_SETUP.md`](../../HITL_SETUP.md).

---

That's the tour. From here, dig into the [architecture](../../README.md) and the
plugin source under `nezha_plugins/` to see how the physics is actually computed.
