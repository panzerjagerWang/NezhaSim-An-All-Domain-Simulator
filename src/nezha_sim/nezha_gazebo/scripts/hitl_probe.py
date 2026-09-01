#!/usr/bin/env python3
"""
HITL link probe — acts as a minimal GCS on the gazebo mavlink_interface's
QGC UDP forward (udp:14550). Confirms the PX4 hardware is alive over the
HIL link and reports HITL / arming state. Read-only (sends nothing that
changes vehicle state). Usage: python3 hitl_probe.py [seconds]
"""
import sys
import time
from pymavlink import mavutil

HIL_FLAG = mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED          # 32
ARMED_FLAG = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED        # 128
CUSTOM_FLAG = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

deadline = time.time() + (float(sys.argv[1]) if len(sys.argv) > 1 else 12.0)

print("[probe] binding udpin:0.0.0.0:14550 (acting as GCS) ...")
m = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

got_hb = False
seen_status = set()
while time.time() < deadline:
    msg = m.recv_match(blocking=True, timeout=1.0)
    if msg is None:
        continue
    t = msg.get_type()
    if t == "HEARTBEAT" and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1:
        if not got_hb:
            print(f"[probe] HEARTBEAT from PX4  sys={msg.get_srcSystem()} "
                  f"autopilot={msg.autopilot} type={msg.type}")
            got_hb = True
        hil = bool(msg.base_mode & HIL_FLAG)
        armed = bool(msg.base_mode & ARMED_FLAG)
        print(f"[probe] base_mode=0x{msg.base_mode:02x}  HITL={hil}  ARMED={armed}  "
              f"custom_mode={msg.custom_mode}  system_status={msg.system_status}")
    elif t == "STATUSTEXT":
        txt = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
        if txt not in seen_status:
            seen_status.add(txt)
            print(f"[probe] STATUSTEXT[{msg.severity}]: {txt}")

if not got_hb:
    print("[probe] *** NO PX4 HEARTBEAT on 14550 — FC not talking over HIL link ***")
    sys.exit(2)
print("[probe] done.")
