"""Compute latency of Python Servo.step, excluding model load and device I/O.

Uses a synthetic six-axis arm with ideal feedback and simulated timestamps.
These timings do not measure real-time scheduling or physical tracking.
"""
import argparse
import json
from pathlib import Path
import platform
import time
import numpy as np
import servo_py as sp


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=5000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")
    model = sp.load_urdf(Path(__file__).with_name("arm6.urdf"), base="base", tip="tool",
                         acceleration_limits=3.)
    servo = sp.Servo(model)
    q, dq = np.array([.2, -.7, 1., .4, .6, -.3]), np.zeros(6)
    samples = []
    for k in range(args.steps + 200):
        now = k * 2_000_000
        state = sp.JointState(q, dq, now)
        command = sp.TwistCommand([.005 * np.sin(k * .03), .005 * np.cos(k * .03), 0], [0, 0, .005], now)
        start = time.perf_counter_ns()
        result = servo.step(state, command, dt=.002, now_ns=now)
        elapsed = time.perf_counter_ns() - start
        if result.action == sp.Action.REJECT:
            raise RuntimeError(result.message)
        q, dq = result.reference.q, result.reference.dq
        if k >= 200:
            samples.append(elapsed * 1e-3)
    percentiles = np.percentile(samples, [50, 95, 99, 100])
    report = {"servo_py": sp.__version__, "python": platform.python_version(),
              "platform": platform.platform(), "model": "synthetic arm6",
              "backend": "native SerialChainModel", "command": "Twist",
              "steps": args.steps, "warmup_steps": 200, "time_unit": "microseconds",
              "p50": float(percentiles[0]), "p95": float(percentiles[1]),
              "p99": float(percentiles[2]), "max": float(percentiles[3]),
              "scope": "Python Servo.step compute only; simulated time, ideal feedback, collision disabled; not a real-time deadline test"}
    text = json.dumps(report, indent=2)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
