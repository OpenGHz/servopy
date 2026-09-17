import numpy as np
import pytest
from servo_py import load_urdf
from servo_py import _core
from conftest import ROOT


def finite_difference_jacobian(model, q):
    h = 1e-6
    current = model.fk(q)
    numerical = np.zeros((6, len(q)))
    for i in range(len(q)):
        shift = np.zeros(len(q)); shift[i] = h
        plus, minus = model.fk(q + shift), model.fk(q - shift)
        numerical[:3, i] = (plus[:3, 3] - minus[:3, 3]) / (2 * h)
        skew = ((plus[:3, :3] - minus[:3, :3]) / (2 * h)) @ current[:3, :3].T
        numerical[3:, i] = [skew[2, 1], skew[0, 2], skew[1, 0]]
    return numerical


def test_planar_fk_closed_form(planar):
    q = np.array([.3, -.8])
    pose = planar.fk(q)
    np.testing.assert_allclose(pose[:3, 3], [np.cos(q[0]) + np.cos(q.sum()),
                                           np.sin(q[0]) + np.sin(q.sum()), 0])
    assert planar.joint_names() == ["shoulder", "elbow"]


@pytest.mark.parametrize("seed", [11, 29, 47])
def test_jacobian_against_finite_difference(planar, seed):
    q = np.random.default_rng(seed).uniform(-1.5, 1.5, 2)
    np.testing.assert_allclose(planar.jacobian(q), finite_difference_jacobian(planar, q), atol=1e-8)


def test_mixed_joints_rotated_origins_and_tcp(mixed_urdf):
    model = load_urdf(mixed_urdf, base="base", tip="tool", acceleration_limits=[2., 1.])
    q = np.array([.8, .2])
    np.testing.assert_allclose(model.jacobian(q), finite_difference_jacobian(model, q), atol=1e-8)
    assert model.joint_names() == ["spin", "slide"]
    np.testing.assert_allclose(model.difference([np.pi + .01, .1], [-np.pi + .01, 0]), [0, .1], atol=1e-12)
    # Configuration integration is unwrapped even for continuous joints.
    np.testing.assert_allclose(model.integrate([np.pi, 0], [.1, .2]), [np.pi + .1, .2])


@pytest.mark.parametrize("q", [[0], [0, 0, 0], [float("nan"), 0]])
def test_model_rejects_invalid_q(planar, q):
    with pytest.raises(ValueError):
        planar.fk(q)
    with pytest.raises(ValueError):
        planar.jacobian(q)


@pytest.mark.parametrize("change", [
    lambda t: t.replace('type="continuous"', 'type="floating"'),
    lambda t: t.replace('<axis xyz="0 1 0"/>', '<axis xyz="0 0 0"/>'),
    lambda t: t.replace('<axis xyz="0 1 0"/>', '<mimic joint="slide"/>'),
    lambda t: t.replace('velocity="2"', 'velocity="nan"'),
    lambda t: t.replace('lower="-.5"', 'lower=".6"'),
    lambda t: t.replace('name="slide"', 'name="spin"'),
    lambda t: t.replace('<parent link="world"/><child link="base"/>', '<parent link="tool"/><child link="base"/>'),
])
def test_unsupported_or_malformed_urdf(mixed_urdf, change):
    mixed_urdf.write_text(change(mixed_urdf.read_text()))
    with pytest.raises(ValueError):
        load_urdf(mixed_urdf, base="base", tip="tool", acceleration_limits=1.)


def test_explicit_limit_mapping(mixed_urdf):
    model = load_urdf(mixed_urdf, base="base", tip="tool",
                     acceleration_limits={"slide": .3, "spin": .7},
                     velocity_limits={"slide": .2, "spin": .5})
    np.testing.assert_allclose(model.limits.acceleration, [.7, .3])
    with pytest.raises(ValueError):
        load_urdf(mixed_urdf, base="base", tip="tool", acceleration_limits={"slide": 1})


def test_six_axis_jacobian():
    model = load_urdf(ROOT / "examples/arm6.urdf", base="base", tip="tool", acceleration_limits=3.)
    q = np.array([.2, -.7, 1., .4, .6, -.3])
    np.testing.assert_allclose(model.jacobian(q), finite_difference_jacobian(model, q), atol=1e-8)
