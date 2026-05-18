"""
upload_all.py — PlatformIO extra_script (post-build + post-upload hooks)

Post-build:
  After firmware.bin is compiled, build the SPIFFS image (if not already up
  to date) then merge bootloader + partitions + firmware + spiffs into a
  single all-in-one binary at the project root:
      firmware-allinone.bin  (flash from address 0x0)

Post-upload:
  After the firmware is flashed, automatically build and upload the SPIFFS
  filesystem image, then re-run the merge so the allinone binary is always
  in sync with what is on the device.

Usage — referenced in platformio.ini:
    extra_scripts = post:scripts/upload_all.py
"""

import os
import subprocess
import sys
import time

Import("env")   # noqa: F821  — SCons environment injected by PlatformIO

# ─── Merge helper ─────────────────────────────────────────────────────────────

def _merge_allinone(env, spiffs_ready=True):
    """Merge bootloader + partitions + boot_app0 + firmware [+ spiffs] → firmware-allinone.bin."""

    build_dir   = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    output      = os.path.join(project_dir, "firmware.bin")

    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    firmware   = os.path.join(build_dir, "firmware.bin")
    spiffs     = os.path.join(build_dir, "spiffs.bin")

    # boot_app0.bin initialises the OTA data partition (0xe000) so the
    # bootloader knows which app slot to use. Without it the device
    # enters a watchdog reset loop when an OTA partition table is present.
    framework_dir = env.subst("$PROJECT_PACKAGES_DIR")
    boot_app0 = os.path.join(
        framework_dir,
        "framework-arduinoespressif32", "tools", "partitions", "boot_app0.bin"
    )
    # Fallback: search platformio home for the file
    if not os.path.isfile(boot_app0):
        pio_home = os.path.expanduser("~/.platformio/packages")
        for entry in os.listdir(pio_home):
            candidate = os.path.join(
                pio_home, entry, "tools", "partitions", "boot_app0.bin"
            )
            if "arduinoespressif32" in entry and os.path.isfile(candidate):
                boot_app0 = candidate
                break

    # Verify required files exist
    for f in (bootloader, partitions, firmware):
        if not os.path.isfile(f):
            print("[merge] WARNING: %s not found, skipping merge." % f)
            return
    if not os.path.isfile(boot_app0):
        print("[merge] WARNING: boot_app0.bin not found, skipping merge.")
        print("[merge]          Expected: %s" % boot_app0)
        return

    esptool = os.path.join(
        env.subst("$PROJECT_PACKAGES_DIR"),
        "tool-esptoolpy", "esptool.py"
    )
    if not os.path.isfile(esptool):
        esptool_cmd = [sys.executable, "-m", "esptool"]
    else:
        esptool_cmd = [sys.executable, esptool]

    # Use --flash_mode dio to match what PlatformIO's write_flash does
    # (even though the board runs QIO at runtime, the ROM reads the header
    # in DIO-compatible mode first; the bootloader later switches to QIO).
    args = esptool_cmd + [
        "--chip", "esp32s3",
        "merge_bin",
        "-o", output,
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "16MB",
        "0x0000", bootloader,
        "0x8000", partitions,
        "0xe000", boot_app0,
        "0x10000", firmware,
    ]

    if spiffs_ready and os.path.isfile(spiffs):
        args += ["0x810000", spiffs]
        has_spiffs = True
    else:
        has_spiffs = False

    print()
    print("=" * 60)
    print("[merge] Creating firmware.bin%s" %
          (" (firmware + SPIFFS)" if has_spiffs else " (firmware only, no SPIFFS yet)"))
    print("=" * 60)

    result = subprocess.run(args, cwd=project_dir)

    if result.returncode == 0:
        size_mb = os.path.getsize(output) / (1024 * 1024)
        print("[merge] OK → firmware.bin  (%.1f MB)" % size_mb)
        print("[merge] Flash from address 0x0 to programme the whole device.")
        if not has_spiffs:
            print("[merge] NOTE: SPIFFS not included. Run 'pio run --target buildfs'")
            print("[merge]       then rebuild to get a complete allinone binary.")
    else:
        print("[merge] WARNING: merge_bin exited with code %d" % result.returncode)


# ─── Post-build hook ──────────────────────────────────────────────────────────

def _post_build_merge(source, target, env):   # noqa: ANN001
    """Called after firmware.bin is built. Builds SPIFFS then merges."""

    pio_env     = env.subst("$PIOENV")
    project_dir = env.subst("$PROJECT_DIR")
    build_dir   = env.subst("$BUILD_DIR")
    spiffs      = os.path.join(build_dir, "spiffs.bin")

    # Build SPIFFS image if it doesn't exist yet (first time) or data/ is newer
    data_dir = env.subst("$PROJECT_DATA_DIR")
    need_buildfs = False
    if not os.path.isfile(spiffs):
        need_buildfs = True
    elif os.path.isdir(data_dir):
        spiffs_mtime = os.path.getmtime(spiffs)
        for root, _dirs, files in os.walk(data_dir):
            for fname in files:
                if os.path.getmtime(os.path.join(root, fname)) > spiffs_mtime:
                    need_buildfs = True
                    break
            if need_buildfs:
                break

    if need_buildfs:
        print()
        print("=" * 60)
        print("[merge] Building SPIFFS filesystem image …")
        print("=" * 60)
        subprocess.run(
            [sys.executable, "-m", "platformio", "run",
             "--target", "buildfs", "--environment", pio_env],
            cwd=project_dir,
        )

    _merge_allinone(env, spiffs_ready=True)


# ─── Post-upload hook ─────────────────────────────────────────────────────────

def _upload_spiffs(source, target, env):   # noqa: ANN001
    """Post-upload action: build + upload SPIFFS image, then refresh the merge."""

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

    # Re-merge so the allinone binary reflects the freshly uploaded SPIFFS
    _merge_allinone(env, spiffs_ready=True)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", _post_build_merge)
env.AddPostAction("upload", _upload_spiffs)
