"""Exercise an installed wheel from a fresh directory, without checkout assets."""
from __future__ import annotations

import argparse
import hashlib
from importlib import metadata, resources
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

import servo_py


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mujoco", action="store_true", help="Also test the installed Panda entry point.")
    args = parser.parse_args()

    assert servo_py.__version__ == metadata.version("servo-py")
    files = resources.files("servo_py.examples")
    for name in ("planar2.urdf", "arm6.urdf", "assets/PANDA-LICENSE.txt"):
        assert files.joinpath(name).is_file(), name
    manifest = json.loads(files.joinpath("assets/panda-source.json").read_text())
    assert not files.joinpath("assets/panda.zip").is_file(), "Panda assets must be downloaded on demand"
    assert any(ep.name == "servo-py-panda" for ep in metadata.distribution("servo-py").entry_points)

    with tempfile.TemporaryDirectory(prefix="servo-py-installed-") as directory:
        environment = os.environ.copy()
        environment.pop("SERVO_PY_PANDA_ARCHIVE", None)
        environment["XDG_CACHE_HOME"] = str(Path(directory) / "cache")
        cache = Path(environment["XDG_CACHE_HOME"]) / "servo-py/panda" / manifest["archive_sha256"] / "panda.zip"

        def run(*command):
            return subprocess.check_output(command, cwd=directory, env=environment, text=True)

        result = json.loads(run(sys.executable, "-I", "-m", "servo_py.examples.track_pose"))
        assert result["steps"] == 1200
        assert result["final_action"] == "HOLD"
        assert result["final_position_error_m"] < 0.0001
        assert not cache.exists(), "The basic example must not fetch the Panda model"
        if args.mujoco:
            entrypoint = Path(sys.executable).parent / "servo-py-panda"
            assert "--control-mode" in run(str(entrypoint), "--help")
            assert not cache.exists(), "--help must not fetch the Panda model"
            # Use the default 18-second trajectory: shorter runs alter its speed.
            result = json.loads(run(str(entrypoint), "--headless"))
            assert result["completed"]
            assert result["final_position_error_m"] < 0.001
            assert hashlib.sha256(cache.read_bytes()).hexdigest() == manifest["archive_sha256"]
            # Reuse the populated cache with network access explicitly disabled.
            run(sys.executable, "-I", "-c", """
from unittest.mock import patch
from servo_py.examples.mujoco_panda import load_panda
with patch('servo_py.examples._panda_assets.urlopen', side_effect=AssertionError('Unexpected download')):
    assert load_panda().nq > 0
""")
    print(f"Installed wheel smoke checks passed: servo-py {servo_py.__version__}")


if __name__ == "__main__":
    main()
