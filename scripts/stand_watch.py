"""Stand-hold watcher: ramp Go2 to stance then hold it at 500 Hz forever,
printing a pose/vel heartbeat every 5 s. Run while the sim is running.
Stops cleanly on SIGINT/SIGTERM.
"""
from __future__ import annotations

import math
import signal
import sys
import threading
import time

sys.path.insert(0, ".")
from codeep.robot.go2_client import Go2Client, STAND_UP, STAND_DOWN, NUM_MOTORS

DT = 0.002


def _yaw(q):
    w,x,y,z = q
    siny_cosp = 2.0*(w*z+x*y); cosy_cosp = 1.0-2.0*(y*y+z*z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    client = Go2Client()
    t0 = time.time()
    while client.pose() is None and time.time() - t0 < 5.0:
        time.sleep(0.05)
    if client.pose() is None:
        print("[watch] FAIL: no telemetry — is the sim running?", flush=True)
        sys.exit(2)
    print(f"[watch] connected. first pose={client.pose()}", flush=True)
    _q0 = client.quaternion()
    if _q0 is not None:
        print(f"[watch] first yaw (deg) = {math.degrees(_yaw(_q0)):+.1f}", flush=True)

    stop = threading.Event()

    def loop():
        t = 0.0
        while not stop.is_set():
            s = time.perf_counter()
            if t < 1.2:
                phase = math.tanh(t / 1.2)
                q = phase * STAND_UP + (1 - phase) * STAND_DOWN
                kp = phase * 50.0 + (1 - phase) * 20.0
                client.send_motors(q, [kp] * NUM_MOTORS, [3.5] * NUM_MOTORS)
            else:
                client.hold_stance(50.0, 3.5)
            t += DT
            r = DT - (time.perf_counter() - s)
            if r > 0:
                time.sleep(r)

    def _sig(_s, _f):
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    th = threading.Thread(target=loop, daemon=True)
    th.start()

    print("[watch] ramping to stance (1.2s) then holding ...", flush=True)
    last = -1
    while not stop.is_set():
        time.sleep(0.2)
        now = time.time()
        if int(now) - last >= 5:
            last = int(now)
            p = client.pose(); v = client.velocity(); q = client.quaternion()
            if p is None or v is None or q is None:
                continue
            print(f"[watch] pose=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                  f"vel=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f}) yaw={math.degrees(_yaw(q)):+.1f}deg", flush=True)
            print(f"[watch] pose=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                  f"vel=({v[0]:+.3f},{v[1]:+.3f},{v[2]:+.3f})", flush=True)

    th.join(timeout=1.0)
    print("[watch] stopped", flush=True)


if __name__ == "__main__":
    main()