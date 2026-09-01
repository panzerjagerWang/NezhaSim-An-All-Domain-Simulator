#!/usr/bin/env python3
"""Autonomous HITL flight test WITHOUT GPS:
 - streams GCS heartbeats + MANUAL_CONTROL (virtual joystick) so PX4 has a
   control source (no RC needed),
 - switches to ALTITUDE mode (baro+attitude, no GPS),
 - arms, holds, then commands climb throttle.
Run /tmp/odom_watch.py separately to see the sim vehicle's height respond."""
import socket, time, threading, subprocess, re
from pymavlink import mavutil

def gz_ports():
    out=subprocess.run(["ss","-unlp"],capture_output=True,text=True).stdout
    return sorted({int(m.group(1)) for line in out.splitlines() if "gz" in line
                   for m in [re.search(r"0\.0\.0\.0:(\d+)",line)] if m})
cands=gz_ports()
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind(("0.0.0.0",14550)); s.settimeout(0.1)
mav=mavutil.mavlink.MAVLink(None); mav.srcSystem=255; mav.srcComponent=190
peer=[None]; throttle=[500]; run=[True]; lock=threading.Lock()

def enc(msg): return msg.pack(mav)
def hb(): return enc(mav.heartbeat_encode(mavutil.mavlink.MAV_TYPE_GCS,mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,0))
def manual(): # x,y,z(throttle 0-1000),r ; buttons
    return enc(mav.manual_control_encode(1,0,0,throttle[0],0,0))
def sendall(b):
    for p in cands:
        try: s.sendto(b,("127.0.0.1",p))
        except OSError: pass
    if peer[0]:
        try: s.sendto(b,peer[0])
        except OSError: pass

def streamer():
    while run[0]:
        with lock:
            sendall(hb()); sendall(manual())
        time.sleep(0.05)   # 20 Hz
threading.Thread(target=streamer,daemon=True).start()

def cmd(c,*p):
    p=list(p)+[0]*(7-len(p))
    with lock: sendall(enc(mav.command_long_encode(1,1,c,0,*p)))

def status_pump(dur):
    end=time.time()+dur
    while time.time()<end:
        try: data,addr=s.recvfrom(2048)
        except socket.timeout: continue
        peer[0]=addr
        try: msgs=mav.parse_buffer(data) or []
        except Exception: msgs=[]
        for m in msgs:
            t=m.get_type()
            if t=="STATUSTEXT":
                tx=m.text if isinstance(m.text,str) else m.text.decode(errors="ignore")
                print(f"  [{m.severity}] {tx}")
            elif t=="COMMAND_ACK":
                print(f"  ACK cmd={m.command} result={m.result}")
            elif t=="HEARTBEAT" and m.get_srcComponent()==1:
                armed=bool(m.base_mode&128); mm=(m.custom_mode>>16)&0xff
                print(f"  HB armed={armed} main_mode={mm} state={m.system_status}")

print("== prime (4s) =="); status_pump(4)
print("== set ALTITUDE mode (main_mode=2) ==")
cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE,1,2,0); status_pump(3)
print("== ARM ==");
cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,1,0); status_pump(4)
print("== climb throttle (z=700) for 8s =="); throttle[0]=700; status_pump(8)
print("== hold (z=500) 4s =="); throttle[0]=500; status_pump(4)
print("== land: throttle down (z=350) 6s =="); throttle[0]=350; status_pump(6)
print("== DISARM =="); cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0); status_pump(3)
run[0]=False; time.sleep(0.2); print("done")
