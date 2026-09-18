import json
import numpy as np
import pytest
from servo_py import (JointJogCommand, JointState, JsonlRecorder, PoseCommand, Servo,
                      ServoConfig, StopCommand, command_from_dict, command_to_dict,
                      compare_recordings, read_records, replay)


def record_run(slider, path):
    servo = Servo(slider, ServoConfig(task_axes=(0,)))
    q, dq = [0], [0]
    with JsonlRecorder(path) as recorder:
        for k in range(40):
            now = k * 10_000_000
            state = JointState(q, dq, now)
            command = JointJogCommand([.3], now) if k < 15 else StopCommand()
            result = servo.step(state, command, dt=.01, now_ns=now)
            recorder.record(now, .01, state, command, result)
            q, dq = result.reference.q, result.reference.dq


def test_record_replay_preserves_inputs_timestamps_and_references(slider, tmp_path):
    path = tmp_path / "run.jsonl"
    record_run(slider, path)
    results = list(replay(Servo(slider, ServoConfig(task_axes=(0,))), path))
    records = list(read_records(path))
    for result, row in zip(results, records):
        assert result.action.name == row["action"]
        assert int(result.flags) == row["flags"]
        np.testing.assert_array_equal(result.reference.q, row["reference"]["q"])
    assert compare_recordings(path, path)["passed"]


def test_comparison_reports_perturbation_and_rejects_time_misalignment(slider, tmp_path):
    a, b = tmp_path / "a.jsonl", tmp_path / "b.jsonl"
    record_run(slider, a)
    rows = list(read_records(a))
    rows[3]["reference"]["q"][0] += .01
    b.write_text("".join(json.dumps(row) + "\n" for row in rows))
    comparison = compare_recordings(a, b)
    assert not comparison["passed"]
    assert comparison["errors"]["q"]["max_abs"] == pytest.approx(.01)
    rows[3]["now_ns"] += 1
    b.write_text("".join(json.dumps(row) + "\n" for row in rows))
    with pytest.raises(ValueError, match="aligned"):
        compare_recordings(a, b)


def test_command_codec_preserves_source_stamp():
    command = PoseCommand(np.eye(4), 12)
    data = command_to_dict(command)
    decoded = command_from_dict(data, stamp_ns=999)
    assert decoded.stamp_ns == 12
    np.testing.assert_array_equal(decoded.pose, command.pose)
    assert command_from_dict({"type": "joint_position", "positions": [1]}, stamp_ns=23).stamp_ns == 23


@pytest.mark.parametrize("data", [[], {"type": "unknown"}, {"type": "stop", "extra": 1},
                                  {"type": "pose", "pose": "bad", "stamp_ns": 0},
                                  {"type": "twist", "linear": [1], "angular": [0, 0, 0], "stamp_ns": 0},
                                  {"type": "joint_position", "positions": [1], "names": "j0", "stamp_ns": 0},
                                  {"type": "joint_position", "positions": [1], "stamp_ns": -1},
                                  {"type": "joint_position", "positions": [float('nan')], "stamp_ns": 0}])
def test_invalid_interchange_rejected(data):
    with pytest.raises(ValueError):
        command_from_dict(data)


def test_joint_trajectory_export_comparison(slider, tmp_path):
    from servo_py import compare_joint_trajectory
    log, exported = tmp_path / "log.jsonl", tmp_path / "moveit.json"
    record_run(slider, log)
    points = []
    for row in read_records(log):
        ref = row["reference"]
        sec, ns = divmod(ref["stamp_ns"], 1_000_000_000)
        points.append(dict(positions=ref["q"], velocities=ref["dq"], accelerations=ref["ddq"],
                           time_from_start=dict(sec=sec, nanosec=ns)))
    exported.write_text(json.dumps(dict(joint_names=["j0"], points=points)))
    assert compare_joint_trajectory(log, exported, joint_names=["j0"])["passed"]
    points[0]["positions"][0] += .01
    exported.write_text(json.dumps(dict(joint_names=["j0"], points=points)))
    assert not compare_joint_trajectory(log, exported, joint_names=["j0"])["passed"]
    with pytest.raises(ValueError, match="names"):
        compare_joint_trajectory(log, exported, joint_names=["wrong"])
