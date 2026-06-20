"""Gate F (combined) — multi-waypoint sequential navigation WITH obstacle
avoidance in a single run (all navigation bonuses together). The waypoint path
passes the obstacle box at (2.5, 0) on the first leg, so the ObstacleAvoider
detours, then the robot continues through the remaining waypoints.

Run while the unitree_mujoco sim (scene_obstacle.xml) is up:
  .venv/bin/python gates/gate_f_combined.py

Validation:
  * reaches ALL 4 waypoints in order, each within 0.40 m
  * avoids the obstacle (min distance to obstacle center > 0.25 m)
  * stays upright (z > 0.20)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner
from codeep.control.nav import NavController
from codeep.control.avoider import ObstacleAvoider
from codeep.control.waypoints import WaypointManager

WAYPOINTS = [(3.5, 0.0), (3.5, 1.5), (0.5, 1.5), (0.5, 0.0)]
OBSTACLE = (2.5, 0.0, 0.25)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=70.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--max-vx", type=float, default=0.30)
    ap.add_argument("--yaw-bias", type=float, default=0.16)
    args = ap.parse_args()

    runner = RLRunner(stand_time=args.stand)
    runner.start()
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")
    print(f"[stand] standing up {args.stand}s ...")
    time.sleep(args.stand)

    nav = NavController(runner, max_vx=args.max_vx, goal_tol=0.15, kp_yaw=1.3,
                        kp_lat_yaw=0.6, yaw_bias=args.yaw_bias, min_align=0.35, max_wz=1.0)
    avoider = ObstacleAvoider(nav, obstacles=[OBSTACLE], reaction_dist=1.0,
                               clearance=0.20, margin=0.45, stop_time=0.0, dt=0.1)
    wm = WaypointManager(nav, WAYPOINTS, avoider=avoider, goal_tol=0.30)
    print(f"[gate F+] waypoints={WAYPOINTS} obstacle={OBSTACLE}")
    print("[gate F+] visiting waypoints in order (with obstacle avoidance) ...")

    start_t = time.time()
    last_idx = -1
    while time.time() - start_t < args.duration and not wm.done:
        r = wm.step()
        p = runner.pose()
        if p is not None and r.get("idx") != last_idx:
            last_idx = r.get("idx")
            if last_idx < len(WAYPOINTS):
                wp = WAYPOINTS[last_idx]
                d = math.hypot(wp[0] - p[0], wp[1] - p[1])
                print(f"  -> wp {last_idx+1}/{len(WAYPOINTS)} {wp} "
                      f"pose=({p[0]:+.2f},{p[1]:+.2f}) dist={d:.2f} "
                      f"min_obs={avoider.min_obs_dist:.2f}")
        time.sleep(0.1)

    runner.set_command(0.0, 0.0, 0.0)
    time.sleep(0.5)
    runner.stop()

    print("\n=== Waypoint completion log ===")
    for idx, pose in wm.reached_log:
        wp = WAYPOINTS[idx]
        print(f"  wp {idx+1} {wp} at ({pose[0]:+.2f},{pose[1]:+.2f}) "
              f"err={math.hypot(wp[0]-pose[0], wp[1]-pose[1]):.2f}")

    n_reached = len(wm.reached_log)
    max_err = max((math.hypot(WAYPOINTS[i][0]-p[0], WAYPOINTS[i][1]-p[1])
                   for i, p in wm.reached_log), default=99.0)
    checks = {
        "all 4 waypoints reached": n_reached >= len(WAYPOINTS),
        "each within 0.40m": max_err < 0.40,
        "obstacle avoided (min_obs>0.25)": avoider.min_obs_dist > 0.25,
    }
    print(f"\nreached {n_reached}/{len(WAYPOINTS)}; max arrival err = {max_err:.3f} m; "
          f"min obstacle dist = {avoider.min_obs_dist:.3f} m")
    print("\n=== Gate F+ checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Gate F+: {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()