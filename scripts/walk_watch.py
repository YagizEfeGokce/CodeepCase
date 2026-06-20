"""Walk watcher: stand up, then trot forward (vx=0.2) indefinitely so you can
watch the Go2 walk in the MuJoCo viewer. Stops on SIGINT/SIGTERM.
"""
from __future__ import annotations

import math
import signal
import sys
import time

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def main():
    runner = RLRunner(stand_time=3.0)
    runner.start()
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[walk] FAIL: no telemetry — is the sim running?", flush=True)
        sys.exit(2)
    print(f"[walk] connected. pose={runner.pose()}", flush=True)

    stop = {"v": False}

    def _sig(_s, _f):
        stop["v"] = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    print("[walk] standing up 3s ...", flush=True)
    time.sleep(3.0)
    runner.set_command(0.2, 0.0, 0.0)
    print("[walk] trotting forward (vx=0.2) ... watch the MuJoCo window", flush=True)

    start_pose = runner.pose()
    last = -1
    while not stop["v"]:
        time.sleep(0.2)
        p = runner.pose(); v = runner.velocity(); q = runner.quaternion()
        if p and v and q and start_pose is not None:
            now = time.time()
            if int(now) - last >= 5:
                last = int(now)
                disp = math.hypot(p[0] - start_pose[0], p[1] - start_pose[1])
                print(f"[walk] pose=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}) "
                      f"vel=({v[0]:+.3f},{v[1]:+.3f}) yaw={math.degrees(yaw_of(q)):+.1f}deg "
                      f"disp={disp:.2f}m", flush=True)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(0.5)
    runner.stop()
    print("[walk] stopped", flush=True)


if __name__ == "__main__":
    main()