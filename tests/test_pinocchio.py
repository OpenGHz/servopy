import numpy as np
import pytest
from servo_py import load_urdf, Servo, ServoConfig, JointState, TwistCommand, Action
from conftest import ROOT

pin = pytest.importorskip("pinocchio")
from servo_py.backends.pinocchio import PinocchioModel
pytestmark = pytest.mark.pinocchio


def test_native_and_pinocchio_planar_parity(planar):
    alternate = PinocchioModel.from_urdf(ROOT / "examples/planar2.urdf", base="base", tip="tool", acceleration_limits=3.)
    for q in np.random.default_rng(73).uniform(-2, 2, (8, 2)):
        np.testing.assert_allclose(planar.fk(q), alternate.fk(q), atol=1e-12)
        np.testing.assert_allclose(planar.jacobian(q), alternate.jacobian(q), atol=1e-12)
    state = JointState([.5, -.8], [0, 0], 0)
    cmd = TwistCommand([.01, .02, 0], [0, 0, 0], 0)
    config = ServoConfig(task_axes=(0, 1))
    a = Servo(planar, config).step(state, cmd, dt=.01, now_ns=0)
    b = Servo(alternate, config).step(state, cmd, dt=.01, now_ns=0)
    assert b.action != Action.REJECT, b.message
    np.testing.assert_allclose(a.reference.q, b.reference.q, atol=1e-12)


def test_mixed_chain_and_nonroot_base_parity(mixed_urdf):
    kwargs = dict(base="base", tip="tool", acceleration_limits=[2., 1.])
    native = load_urdf(mixed_urdf, **kwargs)
    alternate = PinocchioModel.from_urdf(mixed_urdf, **kwargs)
    for q in ([.4, .2], [3.5, -.3], [-7., .1]):
        np.testing.assert_allclose(native.fk(q), alternate.fk(q), atol=1e-12)
        np.testing.assert_allclose(native.jacobian(q), alternate.jacobian(q), atol=1e-12)
    np.testing.assert_allclose(native.difference([3.2, .1], [-3.1, .2]),
                               alternate.difference([3.2, .1], [-3.1, .2]), atol=1e-12)


def test_six_axis_pinocchio_parity():
    path = ROOT / "examples/arm6.urdf"
    kwargs = dict(base="base", tip="tool", acceleration_limits=3.)
    native = load_urdf(path, **kwargs)
    alternate = PinocchioModel.from_urdf(path, **kwargs)
    for q in np.random.default_rng(27).uniform(-1, 1, (8, 6)):
        np.testing.assert_allclose(native.fk(q), alternate.fk(q), atol=1e-12)
        np.testing.assert_allclose(native.jacobian(q), alternate.jacobian(q), atol=1e-12)
