"""DDS IDL type for rangefinder distances (sensor-based obstacle detection).

A simple 3-float DDS message published by the sim's rangefinder thread and
subscribed by the ObstacleAvoider. Uses cyclonedds IdlStruct (no IDL file
compilation needed — same pattern as unitree_sdk2_python/example/helloworld).

NOTE: no `from __future__ import annotations` here — cyclonedds reads
cls.__annotations__ raw, so primitive hints must be real builtin types, not
stringized (else "Type float ... cannot be resolved").
"""
from dataclasses import dataclass
from cyclonedds.idl import IdlStruct


@dataclass
class RangefinderData(IdlStruct, typename="RangefinderData"):
    forward: float   # center rangefinder distance (m), -1 if no hit
    left: float      # left rangefinder (m), -1 if no hit
    right: float     # right rangefinder (m), -1 if no hit