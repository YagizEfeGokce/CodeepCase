"""Rangefinder-based obstacle avoidance (Layer 3b, sensor mode).

Subscribes to DDS topic "rt/rangefinders" (RangefinderData: forward, left,
right). When the forward rangefinder reports a distance < reaction_dist, an
obstacle is *sensed* (not known from a map). The robot detours to the side
with the clearer rangefinder reading, then resumes to the target.

This replaces the map-based ObstacleAvoider with actual sensor input — the
robot "sees" the obstacle via MuJoCo rangefinders, not from a known position.
"""
from __future__ import annotations

import math
from typing import Any

from .nav import yaw_from_quat
from ..robot.rangefinder_idl import RangefinderData


class RangefinderAvoider:
    def __init__(self, nav, reaction_dist: float = 1.0, detour_dist: float = 0.8,
                 goal_tol: float = 0.25, clear_dist: float = 1.5):
        self.nav = nav
        self.reaction_dist = reaction_dist
        self.detour_dist = detour_dist
        self.goal_tol = goal_tol
        self.clear_dist = clear_dist
        self.final_target: tuple[float, float] | None = None
        self.state = "idle"
        self.detour: tuple[float, float] | None = None
        self.detected = False
        self.reached = False
        self._rf = {"forward": 99.0, "left": 99.0, "right": 99.0}
        self._min_forward = 99.0

        # subscribe to rangefinder DDS topic
        from unitree_sdk2py.core.channel import ChannelSubscriber
        self._sub = ChannelSubscriber("rt/rangefinders", RangefinderData)
        self._sub.Init(self._on_rf, 10)

    def _on_rf(self, msg: RangefinderData):
        self._rf["forward"] = float(msg.forward)
        self._rf["left"] = float(msg.left)
        self._rf["right"] = float(msg.right)

    def set_target(self, x: float, y: float):
        self.final_target = (x, y)
        self.nav.set_target(x, y)
        self.state = "to_target"
        self.detected = False
        self.reached = False
        self._min_forward = 99.0

    def step(self) -> dict[str, Any] | None:
        runner = self.nav.runner
        pose = runner.pose()
        yaw = self.nav._yaw()
        if pose is None or yaw is None:
            return None

        fwd = self._rf["forward"]
        self._min_forward = min(self._min_forward, fwd)

        if self.state == "to_target":
            if fwd < self.reaction_dist:
                self.detected = True
                left_d = self._rf["left"]
                right_d = self._rf["right"]
                side = 1.0 if left_d >= right_d else -1.0  # +y (left) or -y (right)
                cy, sy = math.cos(yaw), math.sin(yaw)
                # detour perpendicular to heading, toward the clearer side
                dx = -sy * side * self.detour_dist
                dy = cy * side * self.detour_dist
                self.detour = (pose[0] + dx, pose[1] + dy)
                self.nav.set_target(*self.detour)
                self.nav.reached = False
                self.state = "to_detour"
                return dict(state="to_detour", pose=pose, rf=self._rf, detour=self.detour)
            r = self.nav.step()
            return dict(state="to_target", pose=pose, nav=r, rf=self._rf)

        if self.state == "to_detour":
            if self._wp_reached(pose, self.detour):
                self.nav.set_target(*self.final_target)
                self.nav.reached = False
                self.state = "to_final"
            r = self.nav.step()
            return dict(state=self.state, pose=pose, nav=r, rf=self._rf)

        if self.state == "to_final":
            # re-detect a new obstacle en route -> detour again (toward clearer side)
            if fwd < self.reaction_dist:
                left_d = self._rf["left"]; right_d = self._rf["right"]
                side = 1.0 if left_d >= right_d else -1.0
                cy, sy = math.cos(yaw), math.sin(yaw)
                dx = -sy * side * self.detour_dist
                dy = cy * side * self.detour_dist
                self.detour = (pose[0] + dx, pose[1] + dy)
                self.nav.set_target(*self.detour)
                self.nav.reached = False
                self.state = "to_detour"
                return dict(state="to_detour", pose=pose, rf=self._rf, detour=self.detour)
            r = self.nav.step()
            if self.nav.reached:
                self.state = "done"
                self.reached = True
            return dict(state=self.state, pose=pose, nav=r, rf=self._rf)

        return dict(state=self.state, pose=pose)

    def _wp_reached(self, pose, wp):
        if wp is None:
            return False
        return math.hypot(wp[0] - pose[0], wp[1] - pose[1]) < self.goal_tol