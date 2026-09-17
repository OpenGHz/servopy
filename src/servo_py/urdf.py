"""Initialization-only URDF parser for one fixed-base serial R/P chain."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from collections.abc import Mapping
import xml.etree.ElementTree as ET
import numpy as np
from . import _core
from .api import JointLimits


@dataclass(frozen=True)
class ChainSpec:
    joints: list
    names: tuple[str, ...]
    types: tuple[str, ...]
    limits: JointLimits
    base: str
    tip: str


def _triple(text, default):
    value = np.array(default if text is None else text.split(), dtype=float)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("URDF xyz/rpy/axis must contain three finite values")
    return value


def _origin(element):
    pose = np.eye(4)
    if element is None:
        return pose
    pose[:3, 3] = _triple(element.get("xyz"), (0, 0, 0))
    roll, pitch, yaw = _triple(element.get("rpy"), (0, 0, 0))
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    pose[:3, :3] = rz @ ry @ rx
    return pose


def _limit_values(values, names, label):
    if isinstance(values, Mapping):
        if set(values) != set(names):
            raise ValueError(f"{label} mapping must specify exactly the active joints: {names}")
        result = np.array([values[name] for name in names], dtype=float)
    else:
        result = np.asarray(values, dtype=float)
        if result.ndim == 0:
            result = np.full(len(names), result)
    if result.shape != (len(names),) or not np.isfinite(result).all() or (result <= 0).any():
        raise ValueError(f"{label} must provide one finite positive value per active joint")
    return result


def parse_chain(path, *, base, tip, acceleration_limits, velocity_limits=None, margin=0.01):
    root = ET.parse(Path(path)).getroot()
    if root.tag != "robot":
        raise ValueError("expected a URDF <robot> root")
    link_list = [link.get("name") for link in root.findall("link")]
    links = set(link_list)
    if None in links or "" in links or len(links) != len(link_list):
        raise ValueError("URDF link names must be unique and nonempty")
    if base not in links or tip not in links or base == tip:
        raise ValueError("base and tip must be distinct links in the URDF")
    child_to_joint = {}
    all_names = set()
    for joint in root.findall("joint"):
        name = joint.get("name")
        parent, child = joint.find("parent"), joint.find("child")
        if not name or name in all_names or parent is None or child is None:
            raise ValueError("URDF joints need unique names and parent/child links")
        parent_name, child_name = parent.get("link"), child.get("link")
        if parent_name not in links or child_name not in links or child_name in child_to_joint:
            raise ValueError("invalid or multiply-parented URDF link")
        all_names.add(name)
        child_to_joint[child_name] = joint
    # Check the complete graph so even a cycle above the selected base is rejected.
    for link in links:
        visited = set()
        while link in child_to_joint:
            if link in visited:
                raise ValueError("cyclic URDF graph")
            visited.add(link)
            link = child_to_joint[link].find("parent").get("link")
    chain = []
    cursor = tip
    while cursor != base:
        if cursor not in child_to_joint:
            raise ValueError("tip is not a descendant of base")
        joint = child_to_joint[cursor]
        chain.append(joint)
        cursor = joint.find("parent").get("link")
    chain.reverse()
    joints, names, types, lower, upper, speeds = [], [], [], [], [], []
    supported = {"fixed": _core.JointType.FIXED, "revolute": _core.JointType.REVOLUTE,
                 "continuous": _core.JointType.CONTINUOUS, "prismatic": _core.JointType.PRISMATIC}
    for element in chain:
        kind, name = element.get("type"), element.get("name")
        if kind not in supported or element.find("mimic") is not None:
            raise ValueError(f"joint {name}: only independent fixed/revolute/continuous/prismatic joints are supported")
        joint = _core.Joint()
        joint.name = name
        joint.type = supported[kind]
        joint.origin = _origin(element.find("origin"))
        axis = element.find("axis")
        joint.axis = _triple(None if axis is None else axis.get("xyz"), (1, 0, 0))
        joints.append(joint)
        if kind == "fixed":
            continue
        names.append(name)
        types.append(kind)
        limit = element.find("limit")
        if kind == "continuous":
            lower.append(-np.inf)
            upper.append(np.inf)
        else:
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"joint {name} has no position bounds")
            lo, hi = float(limit.get("lower")), float(limit.get("upper"))
            if not np.isfinite([lo, hi]).all():
                raise ValueError("bounded joints need finite position limits")
            lower.append(lo)
            upper.append(hi)
        speeds.append(np.nan if limit is None else float(limit.get("velocity", "nan")))
    if not names:
        raise ValueError("chain must contain at least one movable joint")
    velocity = _limit_values(speeds if velocity_limits is None else velocity_limits, names, "velocity_limits")
    acceleration = _limit_values(acceleration_limits, names, "acceleration_limits")
    limits = JointLimits(lower, upper, velocity, acceleration, margin)
    limits._native()
    return ChainSpec(joints, tuple(names), tuple(types), limits, base, tip)


def load_urdf(path, *, base: str, tip: str, acceleration_limits,
              velocity_limits=None, margin=0.01):
    """Load the selected URDF chain into a native C++ model.

    Acceleration limits are explicit because standard URDF does not supply them.
    Collision geometry, dynamics, mimic joints and side branches are not loaded.
    Fixed transforms including the TCP are preserved.
    """
    spec = parse_chain(path, base=base, tip=tip, acceleration_limits=acceleration_limits,
                       velocity_limits=velocity_limits, margin=margin)
    return _core.SerialChainModel(spec.joints, spec.limits._native(), base, tip)
