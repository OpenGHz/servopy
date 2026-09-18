"""Compare a servo-py log with another log or a MoveIt JointTrajectory export."""
import argparse
import json
from pathlib import Path
from servo_py import compare_recordings, compare_joint_trajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("actual", type=Path)
    parser.add_argument("expected", type=Path)
    parser.add_argument("--format", choices=("recording", "joint-trajectory"), default="recording")
    parser.add_argument("--joint-names", nargs="+")
    parser.add_argument("--atol", type=float, default=1e-6)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.format == "joint-trajectory":
        if not args.joint_names:
            parser.error("--joint-names must specify the servo recording's model order")
        report = compare_joint_trajectory(args.actual, args.expected, joint_names=args.joint_names, atol=args.atol)
    else:
        report = compare_recordings(args.actual, args.expected, atol=args.atol)
    text = json.dumps(report, indent=2)
    print(text)
    if args.output:
        args.output.write_text(text + "\n")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
