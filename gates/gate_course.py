"""Gate Course — multi-waypoint navigation with sensor-based obstacle avoidance.

5 waypoints forming a loop; 3 tall obstacles, each blocking a leg between
consecutive waypoints. The Go2 (ONNX vy policy) visits the waypoints in order;
the RangefinderAvoider DETECTS each obstacle via the onboard rangefinders
(no known map) and detours around it, then resumes to the next waypoint.

Course (scene_course.xml):
  start -> WP1(3,0) -> WP2(3,3) -> WP3(0,3) -> WP4(0,0) -> WP5(1.5,1.5)
  obstacle1 (1.5,0)  on leg start->WP1
  obstacle2 (3,1.5)  on leg WP1->WP2
  obstacle3 (1.5,3)  on leg WP2->WP3
  (legs WP3->WP4 and WP4->WP5 are clear)

Run while the sim (scene_course.xml) is up, or via run.sh course:
  .venv/bin/python gates/gate_course.py --onnx --rf
  bash run.sh course --onnx --rf

Validation:
  * reaches ALL 5 waypoints in order, each within 0.45 m
  * avoids every obstacle (min distance to any obstacle center > 0.30 m)
  * detects >= 1 obstacle (sensor)
  * stays upright (z > 0.20)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

sys.path.insert(0, ".")
from codeep.locomotion.rl_runner import RLRunner
from codeep.locomotion.rl_runner_onnx import RLRunnerOnnx
from codeep.control.nav import NavController
from codeep.control.avoider import ObstacleAvoider
from codeep.control.rangefinder_avoider import RangefinderAvoider
from codeep.control.waypoints import WaypointManager

WAYPOINTS = [(3.0, 0.0), (3.0, 3.0), (0.0, 3.0), (0.0, 0.0), (1.5, 1.5)]
# Obstacle positions are KNOWN ONLY for validation metrics; the robot senses
# them via rangefinders (RangefinderAvoider) and does not read these.
OBSTACLES = [(1.5, 0.0, 0.25), (3.0, 1.5, 0.25), (1.5, 3.0, 0.25)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=110.0)
    ap.add_argument("--stand", type=float, default=3.0)
    ap.add_argument("--max-vx", type=float, default=0.35)
    ap.add_argument("--max-vy", type=float, default=0.3)
    ap.add_argument("--onnx", action="store_true", help="ONNX vy policy (cekirdek, vy closed-loop)")
    ap.add_argument("--rf", action="store_true", help="sensor-based rangefinder detection (bonus)")
    args = ap.parse_args()

    use_vy = args.onnx
    yaw_bias = 0.0 if args.onnx else 0.16
    max_vy = args.max_vy if args.onnx else 0.0
    runner = (RLRunnerOnnx(stand_time=args.stand) if args.onnx
              else RLRunner(stand_time=args.stand))
    runner.start()
    t0 = time.time()
    while runner.pose() is None and time.time() - t0 < 8.0:
        time.sleep(0.05)
    if runner.pose() is None:
        print("[FAIL] no telemetry; is the sim running?"); runner.stop(); sys.exit(2)
    print(f"[init] pose={runner.pose()}")
    print(f"[stand] {args.stand}s ...")
    time.sleep(args.stand)
    start_pose = runner.pose()
    if start_pose is None:
        print("[FAIL] lost telemetry"); runner.stop(); sys.exit(2)

    nav = NavController(runner, max_vx=args.max_vx, goal_tol=0.20, kp_yaw=1.3,
                        kp_lat_yaw=0.6, yaw_bias=yaw_bias, max_vy=max_vy, use_vy=use_vy,
                        min_align=0.35, max_wz=1.0)
    if args.rf:
        avoider = RangefinderAvoider(nav, reaction_dist=1.0, detour_dist=0.8,
                                     goal_tol=0.25, clear_dist=1.5)
        mode = "sensor(rangefinder)"
    else:
        avoider = ObstacleAvoider(nav, obstacles=OBSTACLES, reaction_dist=1.0,
                                  clearance=0.20, margin=0.45, stop_time=0.0, dt=0.1)
        mode = "known-map"
    wm = WaypointManager(nav, WAYPOINTS, avoider=avoider, goal_tol=0.35)
    policy = "ONNX vy" if args.onnx else "all_gait"
    print(f"[course] {policy} / {mode}: {len(WAYPOINTS)} waypoints, 3 obstacles")
    print(f"[course] waypoints={WAYPOINTS}")

    samples = []
    min_obs_dist = float("inf")
    detections = 0
    start_t = time.time()
    last_idx = -1
    last_state = None
    while time.time() - start_t < args.duration and not wm.done:
        r = wm.step()
        p = runner.pose()
        if p is not None:
            for (ox, oy, _r) in OBSTACLES:
                min_obs_dist = min(min_obs_dist, math.hypot(ox - p[0], oy - p[1]))
            samples.append((time.time() - start_t, p[0], p[1], p[2]))
            idx = r.get("idx")
            if idx is not None and idx != last_idx and idx < len(WAYPOINTS):
                last_idx = idx
                wp = WAYPOINTS[idx]
                d = math.hypot(wp[0] - p[0], wp[1] - p[1])
                print(f"  -> wp {idx+1}/{len(WAYPOINTS)} {wp} "
                      f"pose=({p[0]:+.2f},{p[1]:+.2f}) dist={d:.2f} min_obs={min_obs_dist:.2f}")
            # count each obstacle detour via avoider state transition (to_target -> to_detour)
            state = getattr(avoider, "state", None)
            if state is not None and state != last_state:
                if state == "to_detour":
                    detections += 1
                    print(f"  [detect+detour] obstacle sensed (rangefinder) at "
                          f"pose=({p[0]:+.2f},{p[1]:+.2f}) -> detour #{detections}")
                last_state = state
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
    z_arr = np.array([s[3] for s in samples]) if samples else np.array([0.0])
    metrics = {
        "waypoints_reached": n_reached,
        "max_waypoint_err_m": float(max_err),
        "min_obs_dist_m": float(min_obs_dist),
        "obstacle_detections": detections,
        "z_min": float(z_arr.min()),
        "done": bool(wm.done),
    }
    checks = {
        "all 5 waypoints reached": n_reached >= len(WAYPOINTS),
        "each within 0.45m": max_err < 0.45,
        "no collision (min_obs>0.30)": min_obs_dist > 0.30,
        "obstacle detected (sensor)": detections >= 1,
        "z_min>0.20": float(z_arr.min()) > 0.20,
    }
    print("\n=== Course metrics ===")
    for k, v in metrics.items():
        print(f"  {k:24s}: {v}")
    print("\n=== Course checks ===")
    for k, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {k}")
    overall = all(checks.values())
    print(f"\n=== Course: {'PASS' if overall else 'FAIL'} ===")
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()