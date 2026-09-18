"""Check version agreement and the complete Linux publication artifact set."""
from __future__ import annotations

import argparse
from email.parser import BytesParser
from pathlib import Path
import re
import tarfile
import tomllib
import zipfile

from packaging.utils import parse_wheel_filename


ROOT = Path(__file__).resolve().parents[1]


def check_source(tag):
    project = tomllib.loads((ROOT / "pyproject.toml").read_text())["project"]
    version = project["version"]
    cmake = re.search(r"project\(servo_py VERSION ([\d.]+)", (ROOT / "CMakeLists.txt").read_text())[1]
    binding = re.search(r'm\.attr\("__version__"\) = "([^"]+)"', (ROOT / "cpp/bindings.cpp").read_text())[1]
    assert version == cmake == binding, (version, cmake, binding)
    if tag:
        assert tag == f"v{version}", f"Release tag {tag!r} must match v{version}"
    return version


def check_artifacts(directory, version):
    expected = {(f"cp3{minor}", arch) for minor in range(10, 15) for arch in ("x86_64", "aarch64")}
    actual = set()
    wheels = sorted(directory.glob("*.whl"))
    assert len(wheels) == len(expected), f"Expected 10 wheels, found {len(wheels)}"
    for path in wheels:
        name, wheel_version, _, tags = parse_wheel_filename(path.name)
        assert name == "servo-py" and str(wheel_version) == version, path.name
        matches = {(tag.interpreter, arch) for tag in tags for arch in ("x86_64", "aarch64")
                   if tag.abi == tag.interpreter and tag.platform == f"manylinux_2_28_{arch}"}
        assert len(matches) == 1 and not actual.intersection(matches), path.name
        actual.update(matches)
        with zipfile.ZipFile(path) as archive:
            names = archive.namelist()
            for suffix in ("servo_py/examples/mujoco_panda.py", "servo_py/examples/planar2.urdf",
                           "servo_py/examples/assets/panda-source.json", "servo_py/examples/_panda_assets.py",
                           "servo_py/py.typed",
                           "/licenses/LICENSE", "/licenses/NOTICE", "/licenses/LICENSES/Eigen-MPL2.txt",
                           "/licenses/LICENSES/pybind11-BSD.txt",
                           "/licenses/examples/assets/PANDA-LICENSE.txt"):
                assert any(name.endswith(suffix) for name in names), (path.name, suffix)
            assert not any(name.endswith("/panda.zip") for name in names), path.name
            metadata = BytesParser().parsebytes(archive.read(next(name for name in names if name.endswith(".dist-info/METADATA"))))
            assert metadata["Version"] == version and metadata["Requires-Python"] == ">=3.10", path.name
            assert any(name.startswith("servo_py/_core.") and name.endswith(".so") for name in names), path.name
    assert actual == expected, (expected - actual, actual - expected)
    sdists = list(directory.glob("*.tar.gz"))
    assert len(sdists) == 1, sdists
    with tarfile.open(sdists[0]) as archive:
        names = archive.getnames()
        for suffix in ("/pyproject.toml", "/CMakeLists.txt", "/cpp/bindings.cpp", "/README.pypi.md",
                       "/examples/assets/panda-source.json", "/src/servo_py/examples/__init__.py", "/scripts/smoke_wheel.py"):
            assert any(name.endswith(suffix) for name in names), suffix
        assert not any(name.endswith("/panda.zip") for name in names), "Panda archive must not be in the sdist"
    print("Verified 10 manylinux_2_28 wheels and one source distribution")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", default="")
    parser.add_argument("--dist", type=Path)
    args = parser.parse_args()
    version = check_source(args.tag)
    if args.dist:
        check_artifacts(args.dist, version)
    print(f"Release metadata checks passed: servo-py {version}")


if __name__ == "__main__":
    main()
