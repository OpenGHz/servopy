"""Exercise the installed Panda CLI with actual X11 mouse/keyboard events.

Run on Linux with a display, or via xvfb-run. Test-only dependencies:
python-xlib and Pillow. The installed package needs the mujoco extra.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from PIL import Image
from Xlib import X, XK, display
from Xlib.ext import xtest
from Xlib.protocol import event


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("outputs/panda-viewer"))
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    log_path, metrics_path = output / "control.jsonl", output / "metrics.json"
    stdout_path = output / "viewer.log"
    connection = display.Display()
    process = None

    def rows():
        if not log_path.exists():
            return []
        # The recorder may be in the middle of writing the final line.
        lines = log_path.read_text().splitlines(keepends=True)
        return [json.loads(line) for line in lines if line.endswith("\n")]

    def wait_for(predicate, label, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f"Viewer exited during {label}: {stdout_path.read_text()}")
            value = predicate()
            if value:
                return value
            time.sleep(.05)
        raise TimeoutError(f"Timed out waiting for {label}")

    def after_time(seconds):
        records = rows()
        return records[-1] if records and records[-1]["now_ns"] >= seconds * 1e9 else None

    def key(name, pressed):
        code = connection.keysym_to_keycode(XK.string_to_keysym(name))
        if not code:
            raise RuntimeError(f"Missing X11 key: {name}")
        xtest.fake_input(connection, X.KeyPress if pressed else X.KeyRelease, code)
        connection.sync()

    def tap(name):
        key(name, True)
        time.sleep(.05)
        key(name, False)

    def find_window():
        for window in connection.screen().root.query_tree().children:
            name = window.get_wm_name() or ""
            geometry = window.get_geometry()
            if ("mujoco" in str(name).lower()
                    and window.get_attributes().map_state == X.IsViewable
                    and geometry.width >= 320 and geometry.height >= 240):
                return window
        return None

    def screenshot(window, name):
        geometry = window.get_geometry()
        pixels = window.get_image(0, 0, geometry.width, geometry.height, X.ZPixmap, 0xffffffff)
        Image.frombytes("RGB", (geometry.width, geometry.height), pixels.data, "raw", "BGRX").save(output / name)

    def drag(window, button, dx, dy):
        geometry = window.get_geometry()
        x, y = geometry.width // 2, geometry.height // 2
        window.warp_pointer(x, y)
        connection.sync()
        key("Control_L", True)
        xtest.fake_input(connection, X.ButtonPress, button)
        connection.sync()
        time.sleep(.15)  # Native viewer initializes its perturbation on sync.
        for step in range(1, 13):
            window.warp_pointer(x + round(dx * step / 12), y + round(dy * step / 12))
            connection.sync()
            time.sleep(.05)
        xtest.fake_input(connection, X.ButtonRelease, button)
        connection.sync()
        key("Control_L", False)

    try:
        with stdout_path.open("w") as stdout:
            # No --duration: verify that interactive mode remains active past
            # the automatic demo's original 18-second time limit.
            process = subprocess.Popen([
                sys.executable, "-m", "servo_py.examples.mujoco_panda",
                "--interactive-target", "--log", str(log_path), "--metrics", str(metrics_path),
            ], cwd=output, stdout=stdout, stderr=subprocess.STDOUT,
                env={**os.environ, "MUJOCO_GL": "glfw", "PYTHONUNBUFFERED": "1"})
            window = wait_for(find_window, "MuJoCo window")
            window.set_input_focus(X.RevertToParent, X.CurrentTime)
            connection.sync()
            wait_for(lambda: connection.get_input_focus().focus == window, "viewer input focus")
            initial = wait_for(lambda: after_time(.5), "first control records")
            initial_pose = np.asarray(initial["command"]["pose"])
            initial_q = np.asarray(initial["state"]["q"])
            screenshot(window, "initial.png")

            drag(window, 3, 18, -12)  # Ctrl + right button translates the selected target.
            translated = wait_for(lambda: after_time(initial["now_ns"] / 1e9 + 4), "translation tracking")
            moved_pose = np.asarray(translated["command"]["pose"])
            distance = float(np.linalg.norm(moved_pose[:3, 3] - initial_pose[:3, 3]))
            screenshot(window, "translated.png")
            assert distance > .005, f"Mouse translation did not move target: {distance}"
            assert np.max(np.abs(np.asarray(translated["state"]["q"]) - initial_q)) > .001

            drag(window, 1, 7, -4)  # Ctrl + left button rotates it.
            rotated = wait_for(lambda: after_time(translated["now_ns"] / 1e9 + 4), "rotation tracking")
            rotated_pose = np.asarray(rotated["command"]["pose"])
            rotation = float(np.linalg.norm(rotated_pose[:3, :3] - moved_pose[:3, :3]))
            screenshot(window, "rotated.png")
            assert rotation > .005, f"Mouse rotation did not rotate target: {rotation}"

            # Pause, drag without advancing physics, then reset the handle to
            # the measured TCP. Resume should use this pose, not the paused drag.
            tap("space")
            time.sleep(.3)
            before_pause = rows()[-1]
            paused_records = len(rows())
            drag(window, 3, -32, 16)
            assert len(rows()) == paused_records, "Space did not pause the control loop"
            tap("F6")
            time.sleep(.3)
            tap("space")
            resumed = wait_for(lambda: after_time(before_pause["now_ns"] / 1e9 + .8), "resume after target reset")

            from servo_py.examples.mujoco_panda import PandaKinematics, load_panda, pose_error
            backend = PandaKinematics(load_panda())
            measured_before_pause = backend.fk(np.asarray(before_pause["state"]["q"]))
            reset_pose = np.asarray(resumed["command"]["pose"])
            np.testing.assert_allclose(reset_pose, measured_before_pause, atol=.002)

            final = wait_for(lambda: after_time(19.5), "continued interaction after 18 seconds", timeout=40)
            assert final["command"]["type"] == "pose"
            actual = backend.fk(np.asarray(final["state"]["q"]))
            final_error = pose_error(np.asarray(final["command"]["pose"]), actual)
            assert np.linalg.norm(final_error[:3]) < .002
            assert np.linalg.norm(final_error[3:]) < .01
            screenshot(window, "final.png")

            # Send the same close-window event a desktop window manager uses.
            window.send_event(event.ClientMessage(
                window=window,
                client_type=connection.intern_atom("WM_PROTOCOLS"),
                data=(32, [connection.intern_atom("WM_DELETE_WINDOW"), X.CurrentTime, 0, 0, 0]),
            ), event_mask=X.NoEventMask)
            connection.sync()
            assert process.wait(timeout=15) == 0, stdout_path.read_text()
            metrics = json.loads(metrics_path.read_text())
            assert metrics["interactive_target"] and not metrics["completed"]
            assert metrics["steps"] >= 1950
            report = {
                "mouse_translation_m": distance,
                "mouse_rotation_matrix_change": rotation,
                "final_position_error_m": float(np.linalg.norm(final_error[:3])),
                "pause_reset_resume": "passed",
                "continued_past_18_seconds": True,
                "window_close_exit_code": process.returncode,
            }
            (output / "checks.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2))
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        connection.close()


if __name__ == "__main__":
    main()
