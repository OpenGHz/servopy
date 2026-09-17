"""Ideal-feedback replay, not a physics or hardware simulation.

Run: python examples/track_pose.py [--csv trajectory.csv]
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from servo_py import Servo, ServoConfig, JointState, PoseCommand, Action, load_urdf


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()
    model = load_urdf(Path(__file__).with_name("planar2.urdf"), base="base", tip="tool",
                      acceleration_limits=[3., 3.])
    servo = Servo(model, ServoConfig(task_axes=(0, 1), max_linear_speed=.5))
    q, dq = np.array([.5, -1.]), np.zeros(2)
    target = model.fk(np.array([.7, -.7]))
    records = []
    for k in range(1200):
        now = k * 10_000_000
        state = JointState(q, dq, now)
        result = servo.step(state, PoseCommand(target, now), dt=.01, now_ns=now)
        if result.action == Action.REJECT:
            raise RuntimeError(result.message)
        reference = result.reference
        q, dq = reference.q, reference.dq
        position = model.fk(q)[:3, 3]
        records.append([reference.stamp_ns * 1e-9, *q, *dq, position[0], position[1],
                        np.linalg.norm(position - target[:3, 3]), int(result.flags)])
    if args.csv:
        with args.csv.open("w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["time_s", "q0", "q1", "dq0", "dq1", "x", "y", "position_error_m", "flags"])
            writer.writerows(records)
    print(json.dumps({"model": "planar2", "feedback": "ideal", "steps": len(records),
                      "final_position_error_m": records[-1][-2],
                      "final_action": result.action.name,
                      "collision": "disabled"}, indent=2))


if __name__ == "__main__":
    main()
