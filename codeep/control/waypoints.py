"""Waypoint manager (Layer 3c): sequentially visits a list of (x, y) waypoints.
Arrival is measured from the robot POSE (goal_tol), so the manager advances to
the next waypoint BEFORE NavController would command a (0,0,0) "stop" -- which
would freeze the RL policy. Keep nav.goal_tol < waypoint goal_tol so the
controller keeps driving during transitions.
"""
from __future__ import annotations

import math


class WaypointManager:
    def __init__(self, nav, waypoints, avoider=None, goal_tol: float = 0.30):
        self.nav = nav
        self.waypoints = list(waypoints)
        self.avoider = avoider
        self.goal_tol = goal_tol
        self.idx = 0
        self.done = False
        self.reached_log = []  # (idx, pose) per completed waypoint
        if self.waypoints:
            self._goto_current()

    def _goto_current(self):
        wp = self.waypoints[self.idx]
        if self.avoider is not None:
            self.avoider.set_target(*wp)
        else:
            self.nav.set_target(*wp)
            self.nav.reached = False

    def current_target(self):
        return self.waypoints[self.idx] if self.idx < len(self.waypoints) else None

    def step(self):
        runner = self.nav.runner
        if self.done:
            runner.set_command(0.0, 0.0, 0.0)
            return dict(done=True, idx=self.idx)

        pose = runner.pose()
        wp = self.current_target()
        if pose is not None and wp is not None:
            d = math.hypot(wp[0] - pose[0], wp[1] - pose[1])
            if d <= self.goal_tol:
                self.reached_log.append((self.idx, list(pose)))
                self.idx += 1
                if self.idx >= len(self.waypoints):
                    self.done = True
                    runner.set_command(0.0, 0.0, 0.0)
                    return dict(done=True, idx=self.idx, completed=len(self.waypoints))
                self._goto_current()

        if self.avoider is not None:
            self.avoider.step()
        else:
            self.nav.step()
        return dict(done=False, idx=self.idx, target=self.current_target())