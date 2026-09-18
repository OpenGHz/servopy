"""Best-effort wall-clock runner using an ideal simulated device, never hardware."""
import argparse
import json
from dataclasses import asdict
import time
from servo_py import (JointPositionCommand, JsonlRecorder, Servo, ServoConfig,
                      ServoRunner, SimulatedDevice, load_urdf)
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=Path("periodic-servo.jsonl"))
    args = parser.parse_args()
    model = load_urdf(Path(__file__).with_name("planar2.urdf"), base="base", tip="tool", acceleration_limits=[3., 3.])
    servo = Servo(model, ServoConfig(task_axes=(0, 1), command_timeout=.5))
    device = SimulatedDevice([.4, -.8])
    with JsonlRecorder(args.log) as log:
        runner = ServoRunner(servo, device, period=.01, recorder=log)
        runner.commands.publish(JointPositionCommand([.5, -.9], time.monotonic_ns()))
        stats = runner.run(max_steps=100)
    print(json.dumps(asdict(stats), indent=2))


if __name__ == "__main__":
    main()
