import itertools
import numpy as np
import pytest
from servo_py import (Action, BoxQPSolver, DifferentialIK, DifferentialIKRequest,
                      JointState, PoseCommand, Servo, ServoConfig, SafetyFlag, TwistCommand)
from conftest import make_sliders


def request(j, task, lo, hi, preference=None):
    r = DifferentialIKRequest()
    r.jacobian = j
    r.task = task
    r.lower, r.upper = lo, hi
    r.preferred_velocity = np.zeros(len(lo)) if preference is None else preference
    r.damping = .07
    return r


def exhaustive_qp(r):
    """Independent oracle: enumerate active sets for a small strictly convex QP."""
    h = r.jacobian.T @ r.jacobian + r.damping**2 * np.eye(len(r.lower))
    b = r.jacobian.T @ r.task + r.damping**2 * r.preferred_velocity
    candidates = []
    for active in itertools.product((-1, 0, 1), repeat=len(b)):
        fixed = np.flatnonzero(active)
        free = np.flatnonzero(np.asarray(active) == 0)
        x = np.zeros(len(b))
        for i in fixed:
            x[i] = r.lower[i] if active[i] == -1 else r.upper[i]
        if len(free):
            x[free] = np.linalg.solve(h[np.ix_(free, free)], b[free] - h[np.ix_(free, fixed)] @ x[fixed])
        if np.all(x >= r.lower - 1e-9) and np.all(x <= r.upper + 1e-9):
            candidates.append((.5 * x @ h @ x - b @ x, x))
    return min(candidates, key=lambda item: item[0])[1]


def test_qp_matches_exhaustive_active_set_oracle():
    rng = np.random.default_rng(28)
    for _ in range(50):
        r = request(rng.normal(size=(2, 4)), rng.normal(size=2), -rng.random(4), rng.random(4), rng.normal(size=4))
        np.testing.assert_allclose(BoxQPSolver().solve(r), exhaustive_qp(r), atol=2e-7)


def test_qp_redistributes_task_when_a_joint_is_saturated():
    r = request(np.array([[1., 1.]]), [1.], [-1., -1.], [.1, 1.])
    q = BoxQPSolver().solve(r)
    assert q[0] == pytest.approx(.1)
    assert q.sum() > .99  # Clipping a DLS solution would yield only ~0.6.


@pytest.mark.parametrize("bad", [None, [float('nan')], [0., 1.], "exception"])
def test_custom_solver_failure_latches(slider, bad):
    class Solver(DifferentialIK):
        def solve(self, r):
            if isinstance(bad, str):
                raise RuntimeError("solver failed")
            return bad
    servo = Servo(slider, ServoConfig(task_axes=(0,)), differential_ik=Solver())
    result = servo.step(JointState([0], [0], 0), TwistCommand([.1, 0, 0], [0, 0, 0], 0), dt=.01, now_ns=0)
    assert result.action == Action.REJECT
    assert result.flags & SafetyFlag.SOLVER_ERROR
    assert servo.step(JointState([0], [0], 1), TwistCommand([0, 0, 0], [0, 0, 0], 1), dt=.01, now_ns=1).flags & SafetyFlag.FAULT_LATCHED


@pytest.mark.parametrize("qp", [False, True])
@pytest.mark.parametrize("pose", [False, True])
def test_seven_joint_nullspace_posture_preserves_task(qp, pose):
    model = make_sliders(7, lower=-2, upper=2)
    config = ServoConfig(task_axes=(0, 1, 2), nullspace_gain=.5,
                         nullspace_reference=[.2, .1, -.1, -.2, -.1, .1, 0])
    servo = Servo(model, config, differential_ik=BoxQPSolver() if qp else None)
    q, dq = np.zeros(7), np.zeros(7)
    for k in range(200):
        stamp = k * 10_000_000
        command = PoseCommand(np.eye(4), stamp) if pose else TwistCommand([0, 0, 0], [0, 0, 0], stamp)
        result = servo.step(JointState(q, dq, stamp), command, dt=.01, now_ns=stamp)
        assert result.action != Action.REJECT
        q, dq = result.reference.q, result.reference.dq
        np.testing.assert_allclose(model.fk(q)[:3, 3], 0, atol=1e-7)
    assert q[0] > .1
    assert result.diagnostics.nullspace_speed > 0
    if pose:
        assert result.flags & SafetyFlag.GOAL_REACHED
        assert result.action == Action.TRACK  # TCP reached; posture still moves.


def test_centering_moves_only_uncontrolled_directions():
    model = make_sliders(2)
    servo = Servo(model, ServoConfig(task_axes=(0,), joint_centering_gain=.5))
    result = servo.step(JointState([0, .8], [0, 0], 0), TwistCommand([0, 0, 0], [0, 0, 0], 0), dt=.01, now_ns=0)
    assert result.reference.dq[0] == 0
    assert result.reference.dq[1] < 0


@pytest.mark.parametrize("kwargs", [{"nullspace_gain": -1}, {"joint_centering_gain": float('nan')},
                                    {"nullspace_gain": 1}, {"nullspace_reference": [2]}])
def test_invalid_posture_config(slider, kwargs):
    with pytest.raises(ValueError):
        Servo(slider, ServoConfig(task_axes=(0,), **kwargs))
