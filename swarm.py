import math, time, threading
from pymavlink import mavutil

LEADER = 1
FOLLOWERS = [2, 3, 4, 5]
TAKEOFF_ALT = 10
pattern = "line"          # line, v, circle, square

m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
m.wait_heartbeat()
print("Connected")

def set_guided(s):
    m.mav.set_mode_send(s, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)

def arm(s):
    m.mav.command_long_send(s, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

def takeoff(s, alt):
    m.mav.command_long_send(s, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)

def offsets(i, t):
    """(forward, right) in metres, in the leader's frame. i = follower index 0..3"""
    if pattern == "line":
        return (-8 * (i + 1), 0)
    if pattern == "v":
        return [(-8, -8), (-8, 8), (-16, -16), (-16, 16)][i]
    if pattern == "square":
        return [(10, 10), (10, -10), (-10, 10), (-10, -10)][i]
    if pattern == "circle":
        a = 2 * math.pi * i / 4 + 0.2 * t
        return (15 * math.cos(a), 15 * math.sin(a))
    return (-8 * (i + 1), 0)

def ask_pattern():
    global pattern
    while True:
        p = input("pattern (line/v/circle/square): ").strip()
        if p in ("line", "v", "circle", "square"):
            pattern = p

# Launch everything
for s in [LEADER] + FOLLOWERS:
    set_guided(s)
time.sleep(1)
for s in [LEADER] + FOLLOWERS:
    arm(s)
time.sleep(3)
for s in [LEADER] + FOLLOWERS:
    takeoff(s, TAKEOFF_ALT)
print("Taking off, waiting 15 s...")
time.sleep(15)

threading.Thread(target=ask_pattern, daemon=True).start()

lead = None
last_send = 0
t0 = time.time()
while True:
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg and msg.get_srcSystem() == LEADER:
        lead = msg
    if lead is None or time.time() - last_send < 0.2:
        continue
    last_send = time.time()

    lat = lead.lat / 1e7
    lon = lead.lon / 1e7
    alt = lead.relative_alt / 1000.0
    hdg = math.radians(lead.hdg / 100.0) if lead.hdg != 65535 else 0.0

    for i, s in enumerate(FOLLOWERS):
        f, r = offsets(i, time.time() - t0)
        north = f * math.cos(hdg) - r * math.sin(hdg)
        east = f * math.sin(hdg) + r * math.cos(hdg)
        tlat = lat + north / 111320.0
        tlon = lon + east / (111320.0 * math.cos(math.radians(lat)))
        talt = alt + 2 * (i + 1)          # 2 m vertical stagger for safety
        m.mav.set_position_target_global_int_send(
            0, s, 1,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0x0DF8,                        # position only
            int(tlat * 1e7), int(tlon * 1e7), talt,
            0, 0, 0, 0, 0, 0, 0, 0)
