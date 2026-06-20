"""Obstacle avoidance (Layer 3b): reactive detect -> stop -> detour on top of
NavController.

Detection is map-based: obstacle positions are known (from the scene/config)
and the robot's pose comes from SportModeState. We project each obstacle into
the body frame; if one lies ahead within `reaction_dist` and within
`clearance` laterally, the robot (1) stops briefly, (2) picks a detour
waypoint offset perpendicular to the obstacle, (3) navigates there, then
(4) resumes to the final target. This realises the PDF's "stop or change
direction on detection" with our own planning code.

A real lidar/depth sensor could replace the geometric detection with no
change to the reaction/planning layer.
"""
from __future__ import annotations

import math

from .nav import yaw_from_quat


class ObstacleAvoider:
    def __init__(self, nav, obstacles, reaction_dist: float = 1.0,
                 clearance: float = 0.20, margin: float = 0.45,
                 stop_time: float = 0.6, dt: float = 0.1):
        self.nav = nav
        self.obstacles = obstacles  # list of (x, y, radius)
        self.reaction_dist = reaction_dist
        self.clearance = clearance
        self.margin = margin
        self.stop_time = stop_time
        self.dt = dt
        self.final_target = None
        self.state = "idle"
        self.stop_timer = 0.0
        self.last_obs = None
        self.detour = None
        self.min_obs_dist = float("inf")
        self.detected = False
        self.reached = False

    def set_target(self, x: float, y: float):
        self.final_target = (x, y)
        self.nav.set_target(x, y)
        self.state = "to_target"
        self.detected = False
        self.detour = None
        self.reached = False

    def _detect(self, pose, yaw):
        if pose is None or yaw is None:
            return None
        cy, sy = math.cos(yaw), math.sin(yaw)
        for (ox, oy, r) in self.obstacles:
            dx, dy = ox - pose[0], oy - pose[1]
            bx = cy * dx + sy * dy      # forward distance in body frame
            by = -sy * dx + cy * dy     # lateral distance in body frame
            if bx > 0.0 and bx < self.reaction_dist and abs(by) < (r + self.clearance):
                return (ox, oy, r)
        return None

    def step(self):
        runner = self.nav.runner
        pose = runner.pose()
        yaw = self.nav._yaw()
        if pose is None or yaw is None:
            return None

        # track closest approach to any obstacle (for validation)
        for (ox, oy, _r) in self.obstacles:
            self.min_obs_dist = min(self.min_obs_dist, math.hypot(ox - pose[0], oy - pose[1]))

        if self.state == "to_target":
            obs = self._detect(pose, yaw)
            if obs is not None:
                self.last_obs = obs
                self.detected = True
                if self.stop_time > 0.0:
                    self.state = "stopping"
                    self.stop_timer = self.stop_time
                    runner.set_command(0.0, 0.0, 0.0)
                    return dict(state="stopping", pose=pose, obs=obs)
                # no stop: reroute immediately (keep walking, arc around)
                ox, oy, r = obs
                side = 1.0
                self.detour = (ox, oy + side * (r + self.margin))
                self.nav.set_target(*self.detour)
                self.nav.reached = False
                self.state = "to_detour"
                return dict(state="to_detour", pose=pose, detour=self.detour, obs=obs)
            r = self.nav.step()
            return dict(state="to_target", pose=pose, nav=r)

        if self.state == "stopping":
            runner.set_command(0.0, 0.0, 0.0)
            self.stop_timer -= self.dt
            if self.stop_timer <= 0.0 and self.last_obs is not None:
                ox, oy, r = self.last_obs
                # detour to the side of the obstacle (perpendicular offset)
                side = 1.0  # +y (left)
                self.detour = (ox, oy + side * (r + self.margin))
                self.nav.set_target(*self.detour)
                self.nav.reached = False
                self.state = "to_detour"
            return dict(state="stopping", pose=pose, stop_timer=self.stop_timer)

        if self.state == "to_detour":
            r = self.nav.step()
            if self.nav.reached:
                self.nav.set_target(*self.final_target)
                self.nav.reached = False
                self.state = "to_final"
            return dict(state="to_detour", pose=pose, detour=self.detour, nav=r)

        if self.state == "to_final":
            r = self.nav.step()
            if self.nav.reached:
                self.state = "done"
                self.reached = True
            return dict(state=self.state, pose=pose, nav=r)

        return dict(state=self.state, pose=pose)