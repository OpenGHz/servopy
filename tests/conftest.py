from pathlib import Path
import numpy as np
import pytest
from servo_py import load_urdf, ServoConfig
from servo_py import _core

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def planar():
    return load_urdf(ROOT / "examples/planar2.urdf", base="base", tip="tool",
                     acceleration_limits=[3., 3.])


@pytest.fixture
def xy_config():
    return ServoConfig(task_axes=(0, 1), max_linear_speed=0.5)


def make_sliders(n=1, *, lower=-1., upper=1., velocity=1., acceleration=2., margin=.01):
    limits = _core.Limits()
    for name, value in [("lower", lower), ("upper", upper), ("velocity", velocity),
                        ("acceleration", acceleration), ("margin", margin)]:
        setattr(limits, name, np.broadcast_to(value, (n,)).copy())
    joints = []
    for i in range(n):
        joint = _core.Joint()
        joint.name = f"j{i}"
        joint.type = _core.JointType.PRISMATIC
        joint.axis = np.eye(3)[i % 3]
        joints.append(joint)
    return _core.SerialChainModel(joints, limits, "base", "tool")


@pytest.fixture
def slider():
    return make_sliders()


@pytest.fixture
def mixed_urdf(tmp_path):
    path = tmp_path / "mixed.urdf"
    path.write_text('''<robot name="mixed">
      <link name="world"/><link name="base"/><link name="a"/>
      <link name="b"/><link name="tool"/><link name="sibling"/>
      <joint name="mount" type="fixed"><parent link="world"/><child link="base"/>
        <origin xyz=".2 -.1 .4" rpy=".2 -.3 .4"/></joint>
      <joint name="spin" type="continuous"><parent link="base"/><child link="a"/>
        <origin xyz=".1 .2 .3" rpy=".4 .1 -.3"/><axis xyz="0 1 0"/>
        <limit velocity="2" effort="10"/></joint>
      <joint name="slide" type="prismatic"><parent link="a"/><child link="b"/>
        <origin xyz=".3 .1 .2" rpy="-.2 .3 .1"/><axis xyz="0 0 1"/>
        <limit lower="-.5" upper=".5" velocity="1" effort="10"/></joint>
      <joint name="tcp" type="fixed"><parent link="b"/><child link="tool"/>
        <origin xyz=".1 .2 .3" rpy=".3 .2 .1"/></joint>
      <joint name="other" type="revolute"><parent link="world"/><child link="sibling"/>
        <axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="10"/></joint>
    </robot>''')
    return path
