#!/usr/bin/env python3
"""Bootstrapping GCS for the gazebo mavlink_interface QGC forward.
Sprays a GCS heartbeat to candidate plugin UDP ports so the plugin (a) forwards
our heartbeat to PX4 over serial (priming it) and (b) learns our address and
forwards PX4 telemetry back to us. Read-only: reports state/health, no arming."""
import socket, time, sys, subprocess, re
from pymavlink import mavutil

def gz_ports():
    out = subprocess.run(["ss","-unlp"], capture_output=True, text=True).stdout
    ports=set()
    for line in out.splitlines():
        if "gz" in line:
            m=re.search(r"0\.0\.0\.0:(\d+)", line)
            if m: ports.add(int(m.group(1)))
    return sorted(ports)

SENSOR_BITS={1<<0:"GYRO",1<<1:"ACCEL",1<<2:"MAG",1<<3:"BARO",1<<5:"GPS",1<<20:"PREARM"}
secs=float(sys.argv[1]) if len(sys.argv)>1 else 15
cands=gz_ports(); print(f"[gcs] candidate plugin ports: {cands}")

s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(("0.0.0.0",14550)); s.settimeout(0.3)
mav=mavutil.mavlink.MAVLink(None); mav.srcSystem=255; mav.srcComponent=190

def hb_bytes():
    return mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0).pack(mav)

end=time.time()+secs; last_spray=0; peer=None
states=set(); seen=set(); gps=None; unhealthy=None; got=False
while time.time()<end:
    if time.time()-last_spray>0.5:
        last_spray=time.time()
        pkt=hb_bytes()
        for p in cands:
            try: s.sendto(pkt,("127.0.0.1",p))
            except OSError: pass
        if peer:
            try: s.sendto(pkt,peer)
            except OSError: pass
    try: data,addr=s.recvfrom(2048)
    except socket.timeout: continue
    peer=addr
    try: msgs=mav.parse_buffer(data) or []
    except Exception: msgs=[]
    for m in msgs:
        t=m.get_type()
        if t=="HEARTBEAT" and m.get_srcComponent()==1:
            got=True
            states.add((m.system_status,bool(m.base_mode&128),bool(m.base_mode&32)))
        elif t=="STATUSTEXT":
            txt=m.text if isinstance(m.text,str) else m.text.decode(errors="ignore")
            if txt not in seen: seen.add(txt); print(f"[gcs] TEXT[{m.severity}]: {txt}")
        elif t=="GPS_RAW_INT": gps=(m.fix_type,m.satellites_visible)
        elif t=="SYS_STATUS":
            en=m.onboard_control_sensors_enabled; h=m.onboard_control_sensors_health
            unhealthy=[n for b,n in SENSOR_BITS.items() if (en&b) and not(h&b)]
print(f"[gcs] got_px4_heartbeat={got} peer={peer}")
print(f"[gcs] heartbeat(state,armed,HITL)={states}")
print(f"[gcs] GPS(fix,sats)={gps}  unhealthy_sensors={unhealthy}")
