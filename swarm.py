import math, time, threading, argparse
from pymavlink import mavutil

ap = argparse.ArgumentParser()
ap.add_argument("-n", type=int, default=5, help="total drones (leader + followers)")
ap.add_argument("--alt", type=float, default=10, help="takeoff altitude (m)")
ap.add_argument("--spacing", type=float, default=8, help="base spacing (m)")
args = ap.parse_args()

LEADER = 1
FOLLOWERS = list(range(2, args.n + 1))
K = len(FOLLOWERS)
spacing = args.spacing
pattern = "line"
running = True

m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
m.wait_heartbeat()
print(f"Connected. Leader=1, followers={FOLLOWERS}")

# ---------- helpers ----------
def set_mode(s, mode_id):                      # 4 = GUIDED, 6 = RTL, 9 = LAND
    m.mav.set_mode_send(s, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

def collect_acks(cmd, ids, timeout=4):
    ok, end = set(), time.time() + timeout
    while time.time() < end and len(ok) < len(ids):
        a = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=0.5)
        if a and a.command == cmd and a.result == 0:
            ok.add(a.get_srcSystem())
    return ok

def send_to_all(cmd, params, ids, retries=5):
    """Send a COMMAND_LONG to each drone and retry until all acknowledge."""
    pending = set(ids)
    for attempt in range(retries):
        for s in pending:
            m.mav.command_long_send(s, 1, cmd, 0, *params)
        pending -= collect_acks(cmd, pending)
        if not pending:
            return True
        print(f"  retry {attempt+1}: no ACK from {sorted(pending)}")
    print(f"  FAILED for {sorted(pending)}")
    return False

# ---------- formations ----------
def offsets(i, k, t):
    """(forward, right) in metres in the leader's frame. i = 0..k-1"""
    sp = spacing
    if pattern == "line":                       # single file behind
        return (-sp * (i + 1), 0)
    if pattern == "v":                          # wedge, alternating left/right
        row = i // 2 + 1
        side = -1 if i % 2 == 0 else 1
        return (-sp * row, side * sp * row)
    if pattern == "grid":                       # rows of 3 behind the leader
        cols = 3
        row = i // cols + 1
        col = i % cols - (cols - 1) / 2
        return (-sp * row, sp * col)
    if pattern == "circle":                     # rotating ring, radius grows with k
        r = max(15, sp * k / (2 * math.pi) * 1.5)
        a = 2 * math.pi * i / k + 0.2 * t
        return (r * math.cos(a), r * math.sin(a))
    if pattern == "square":                     # followers spread around a square
        h = max(10, sp * k / 8)
        u = (i / k) * 4
        side, f = int(u), u - int(u)
        if side == 0: return (h, -h + 2 * h * f)
        if side == 1: return (h - 2 * h * f, h)
        if side == 2: return (-h, h - 2 * h * f)
        return (-h + 2 * h * f, -h)
    return (-sp * (i + 1), 0)

PATTERNS = ("line", "v", "grid", "circle", "square")

def console():
    global pattern, spacing, running
    while True:
        cmd = input(f"[{pattern}] pattern | 'spacing 12' | 'rtl' > ").strip().split()
        if not cmd: continue
        if cmd[0] in PATTERNS:
            pattern = cmd[0]
        elif cmd[0] == "spacing" and len(cmd) == 2:
            spacing = float(cmd[1])
        elif cmd[0] == "rtl":
            running = False
            for s in [LEADER] + FOLLOWERS:
                set_mode(s, 6)
            print("RTL sent to all drones")
            return

# ---------- launch ----------
ALL = [LEADER] + FOLLOWERS
for s in ALL: set_mode(s, 4)
time.sleep(1)
print("Arming...")
send_to_all(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, (1, 0, 0, 0, 0, 0, 0), ALL)
print("Taking off...")
send_to_all(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, (0, 0, 0, 0, 0, 0, args.alt), ALL)
print("Climbing, waiting 15 s...")
time.sleep(15)

threading.Thread(target=console, daemon=True).start()

# ---------- follow loop ----------
lead, last_send, t0 = None, 0, time.time()
while running:
    msg = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if msg and msg.get_srcSystem() == LEADER:
        lead = msg
    if lead is None or time.time() - last_send < 0.2:
        continue
    last_send = time.time()

    lat, lon = lead.lat / 1e7, lead.lon / 1e7
    alt = lead.relative_alt / 1000.0
    hdg = math.radians(lead.hdg / 100.0) if lead.hdg != 65535 else 0.0
    t = time.time() - t0

    for i, s in enumerate(FOLLOWERS):
        f, r = offsets(i, K, t)
        north = f * math.cos(hdg) - r * math.sin(hdg)
        east  = f * math.sin(hdg) + r * math.cos(hdg)
        tlat = lat + north / 111320.0
        tlon = lon + east / (111320.0 * math.cos(math.radians(lat)))
        talt = alt + 1.5 * (i % 4)                 # 4 altitude layers, cycling
        m.mav.set_position_target_global_int_send(
            0, s, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0x0DF8, int(tlat * 1e7), int(tlon * 1e7), talt,
            0, 0, 0, 0, 0, 0, 0, 0)
print("Done.")