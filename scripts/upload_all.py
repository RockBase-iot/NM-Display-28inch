"""
upload_all.py — PlatformIO extra_script (post:upload hook)

After the firmware is flashed, automatically build and upload the SPIFFS
filesystem image.  This means clicking the single "Upload" button (PlatformIO
bottom toolbar, `pio run --target upload`, or VS Code task) flashes both the
firmware and the filesystem in one step.

Usage — referenced in platformio.ini:
    extra_scripts = post:scripts/upload_all.py
"""

import subprocess
import sys
import time

Import("env")   # noqa: F821  — SCons environment injected by PlatformIO


def _upload_spiffs(source, target, env):   # noqa: ANN001
    """Post-upload action: build + upload SPIFFS image."""

    # Small delay so the ESP has time to fully reset before the next esptool run.
    time.sleep(1.5)

    print()
    print("=" * 60)
    print("[upload_all] Uploading SPIFFS filesystem image …")
    print("=" * 60)

    result = subprocess.run(
        [
            sys.executable, "-m", "platformio",
            "run",
            "--target", "uploadfs",
            "--environment", env.subst("$PIOENV"),
        ],
        cwd=env.subst("$PROJECT_DIR"),
    )

    if result.returncode == 0:
        print("[upload_all] SPIFFS upload complete.")
    else:
        print(
            "[upload_all] WARNING: SPIFFS upload exited with code %d"
            % result.returncode
        )


env.AddPostAction("upload", _upload_spiffs)
