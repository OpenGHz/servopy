"""JSONL command interchange, deterministic replay and reference comparison."""
from __future__ import annotations

from dataclasses import asdict
import json
import operator
from pathlib import Path
import numpy as np

from .api import (CollisionSample, JointJogCommand, JointPositionCommand, JointState,
                  PoseCommand, StopCommand, TwistCommand)
from . import _core

_COMMANDS = {"joint_jog": JointJogCommand, "joint_position": JointPositionCommand,
             "pose": PoseCommand, "twist": TwistCommand, "stop": StopCommand}


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"cannot serialize {type(value).__name__}")


def command_to_dict(command):
    for kind, cls in _COMMANDS.items():
        if isinstance(command, cls):
            return json.loads(json.dumps({"type": kind, **asdict(command)}, default=_json_value, allow_nan=False))
    raise ValueError("unknown command type")


def command_from_dict(data, *, stamp_ns=None):
    """Decode a command; an explicit source timestamp wins over receive time."""
    if not isinstance(data, dict):
        raise ValueError("command must be a JSON object")
    values = dict(data)
    cls = _COMMANDS.get(values.pop("type", None))
    if cls is None:
        raise ValueError("unknown command type")
    if cls is not StopCommand:
        if "stamp_ns" not in values:
            values["stamp_ns"] = stamp_ns
        try:
            stamp = operator.index(values["stamp_ns"])
        except TypeError as exc:
            raise ValueError("command requires an integer timestamp") from exc
        if isinstance(values["stamp_ns"], bool) or not 0 <= stamp < 2**63:
            raise ValueError("timestamp must fit nonnegative int64")
    try:
        command = cls(**values)
        for field, shape in (("linear", (3,)), ("angular", (3,)), ("pose", (4, 4))):
            if hasattr(command, field):
                array = np.asarray(getattr(command, field), dtype=float)
                if array.shape != shape or not np.all(np.isfinite(array)):
                    raise ValueError(f"{field} must have shape {shape} and finite values")
                if field == "pose" and not _core.valid_pose(array):
                    raise ValueError("pose must be a rigid transform")
        if isinstance(command, (JointJogCommand, JointPositionCommand)):
            array = np.asarray(command.velocities if isinstance(command, JointJogCommand) else command.positions, dtype=float)
            if array.ndim != 1 or not array.size or not np.all(np.isfinite(array)):
                raise ValueError("joint command must contain a finite nonempty vector")
            if command.names is not None and (not isinstance(command.names, (list, tuple)) or
                    any(not isinstance(name, str) or not name for name in command.names) or
                    len(command.names) != array.size or len(set(command.names)) != array.size):
                raise ValueError("names must be unique strings matching the joint vector")
        # Reject NaN/Infinity at the interchange boundary as well as in Servo.
        command_to_dict(command)
        return command
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid command fields: {exc}") from exc


class JsonlRecorder:
    def __init__(self, path):
        self._stream = Path(path).open("w", encoding="utf-8")

    def record(self, now_ns, dt, state, command, result, collision=None):
        ref = result.reference
        row = {"schema": 1, "now_ns": now_ns, "dt": dt, "state": asdict(state),
               "command": command_to_dict(command), "collision": asdict(collision) if collision else None,
               "action": result.action.name, "flags": int(result.flags),
               "reference": None if ref is None else {"q": ref.q, "dq": ref.dq, "ddq": ref.ddq, "stamp_ns": ref.stamp_ns}}
        self._stream.write(json.dumps(row, default=_json_value, allow_nan=False) + "\n")

    def close(self):
        self._stream.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


def read_records(path):
    previous = -1
    with Path(path).open(encoding="utf-8") as stream:
        for number, line in enumerate(stream, 1):
            try:
                row = json.loads(line)
                if row["schema"] != 1:
                    raise ValueError("unsupported schema")
                now = operator.index(row["now_ns"])
                if now <= previous or not 0 <= now < 2**63 or not np.isfinite(row["dt"]) or row["dt"] <= 0:
                    raise ValueError("invalid/nonmonotonic timing")
                previous = now
                command_from_dict(row["command"])
                JointState(**row["state"])
            except (ValueError, TypeError, KeyError) as exc:
                raise ValueError(f"invalid record at line {number}: {exc}") from exc
            yield row


def replay(servo, path):
    """Replay recorded feedback/commands into a fresh or explicitly reset Servo."""
    for row in read_records(path):
        collision = CollisionSample(**row["collision"]) if row.get("collision") else None
        yield servo.step(JointState(**row["state"]), command_from_dict(row["command"]),
                         dt=row["dt"], now_ns=row["now_ns"], collision=collision)


def compare_recordings(actual_path, expected_path, *, atol=1e-6):
    """Compare time-aligned reference recordings, including exported MoveIt data.

    Both inputs use schema 1, the same joint order, frames and units. Timestamps
    may have a different epoch, but intervals must match. No resampling silently
    hides scheduling or interpolation differences.
    """
    if not np.isfinite(atol) or atol < 0:
        raise ValueError("atol must be finite and nonnegative")
    actual, expected = list(read_records(actual_path)), list(read_records(expected_path))
    if not actual or len(actual) != len(expected):
        raise ValueError("recordings must have the same nonzero number of records")
    errors = {name: [] for name in ("q", "dq", "ddq")}
    action_mismatches = flag_mismatches = 0
    origin_a, origin_e = actual[0]["now_ns"], expected[0]["now_ns"]
    for a, e in zip(actual, expected):
        if a["now_ns"] - origin_a != e["now_ns"] - origin_e or a["dt"] != e["dt"]:
            raise ValueError("recording intervals are not aligned")
        action_mismatches += a["action"] != e["action"]
        flag_mismatches += a["flags"] != e["flags"]
        if (a["reference"] is None) != (e["reference"] is None):
            raise ValueError("reference availability differs")
        if a["reference"] is None:
            continue
        if a["reference"]["stamp_ns"] - origin_a != e["reference"]["stamp_ns"] - origin_e:
            raise ValueError("reference endpoint timestamps are not aligned")
        for field in errors:
            av, ev = np.asarray(a["reference"][field], dtype=float), np.asarray(e["reference"][field], dtype=float)
            if av.ndim != 1 or not av.size or av.shape != ev.shape or not np.all(np.isfinite(av)) or not np.all(np.isfinite(ev)):
                raise ValueError("invalid reference dimensions or values")
            errors[field].extend((av - ev).tolist())
    metrics = {field: {"max_abs": float(np.max(np.abs(values))) if values else 0.,
                       "rmse": float(np.sqrt(np.mean(np.square(values)))) if values else 0.}
               for field, values in errors.items()}
    return {"samples": len(actual), "action_mismatches": action_mismatches,
            "flag_mismatches": flag_mismatches, "errors": metrics,
            "passed": action_mismatches == flag_mismatches == 0 and all(v["max_abs"] <= atol for v in metrics.values())}


def compare_joint_trajectory(recording_path, trajectory_path, *, joint_names, atol=1e-6):
    """Compare against a JSON export of ROS 2 trajectory_msgs/JointTrajectory.

    This is an offline numerical comparison, not an assertion of MoveIt parity.
    Expected points correspond to reference endpoints, measured from the first
    record's now_ns. Require q/dq/ddq and align by joint name without resampling.
    """
    if not np.isfinite(atol) or atol < 0:
        raise ValueError("atol must be finite and nonnegative")
    rows = list(read_records(recording_path))
    expected = json.loads(Path(trajectory_path).read_text())
    names = tuple(joint_names)
    exported_names = expected["joint_names"]
    if not names or len(set(names)) != len(names) or len(exported_names) != len(names) or set(exported_names) != set(names):
        raise ValueError("joint names must be complete unique permutations")
    order = [exported_names.index(name) for name in names]
    points = expected["points"]
    if not rows or len(rows) != len(points):
        raise ValueError("one trajectory point per recorded reference endpoint is required")
    differences = {field: [] for field in ("q", "dq", "ddq")}
    origin = rows[0]["now_ns"]
    for row, point in zip(rows, points):
        t = point["time_from_start"]
        sec, nsec = operator.index(t["sec"]), operator.index(t["nanosec"])
        if sec < 0 or not 0 <= nsec < 1_000_000_000:
            raise ValueError("invalid trajectory time_from_start")
        ref = row["reference"]
        if ref is None or ref["stamp_ns"] - origin != sec * 1_000_000_000 + nsec:
            raise ValueError("trajectory endpoint times are not aligned")
        for field, source in (("q", "positions"), ("dq", "velocities"), ("ddq", "accelerations")):
            av, ev = np.asarray(ref[field]), np.asarray(point[source])
            if av.shape != (len(names),) or ev.shape != av.shape or not np.all(np.isfinite(av)) or not np.all(np.isfinite(ev)):
                raise ValueError("trajectory requires finite position/velocity/acceleration for every joint")
            differences[field].extend((av - ev[order]).tolist())
    metrics = {field: {"max_abs": float(np.max(np.abs(values))),
                       "rmse": float(np.sqrt(np.mean(np.square(values))))}
               for field, values in differences.items()}
    return {"samples": len(rows), "errors": metrics,
            "passed": all(v["max_abs"] <= atol for v in metrics.values())}
