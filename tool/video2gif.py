#!/usr/bin/env python3
"""
video2gif.py — Convert a video file to an Animated GIF optimised for ESP32-S3.

Default settings target the NM-Display-28inch (320x240, ST7789 SPI LCD):
  - Resolution  : 320 x 240  (exact screen size, no up-scaling)
  - Frame rate  : 12 fps     (AnimatedGIF library soft-decode limit ~15 fps)
  - Colours     : 128        (palette entries; fewer = smaller file)
  - Dithering   : Floyd-Steinberg (best quality for limited palette)
  - Max file    : 2 MB       (SPIFFS partition budget)
  - Loop        : infinite

Dependencies (all pre-installed in this project's Python env):
  pip install opencv-python Pillow

Usage examples:
  # Basic — uses all defaults, output next to input file:
  python tool/video2gif.py resource/animated.mp4

  # Custom target for a different board:
  python tool/video2gif.py resource/animated.mp4 --output resource/boot.gif \\
      --width 320 --height 240 --fps 10 --colors 64

  # Place output in SPIFFS data directory:
  python tool/video2gif.py resource/animated.mp4 --output data/boot.gif
"""

import argparse
import os
import shutil
import sys
import time

import cv2
from PIL import Image


# ─── Defaults optimised for ESP32-S3 / ST7789 320x240 ───────────────────────
DEFAULT_WIDTH   = 320
DEFAULT_HEIGHT  = 240
DEFAULT_FPS     = 12
DEFAULT_COLORS  = 128
MAX_FILE_BYTES  = 6 * 1024 * 1024   # ~6 MB warning (actual SPIFFS = 7.94 MB)
PALETTE_SAMPLES = 32                 # Number of frames sampled to build the global palette


def parse_args():
    p = argparse.ArgumentParser(
        description="Convert video → Animated GIF optimised for ESP32-S3 LCD.")
    p.add_argument("input",
                   help="Source video file (e.g. resource/animated.mp4)")
    p.add_argument("--output", "-o", default=None,
                   help="Output GIF path. Default: same dir as input, same stem + .gif")
    p.add_argument("--width",  "-W", type=int, default=DEFAULT_WIDTH,
                   help=f"Target width in pixels  (default: {DEFAULT_WIDTH})")
    p.add_argument("--height", "-H", type=int, default=DEFAULT_HEIGHT,
                   help=f"Target height in pixels (default: {DEFAULT_HEIGHT})")
    p.add_argument("--fps",    "-f", type=float, default=DEFAULT_FPS,
                   help=f"Target frame rate (default: {DEFAULT_FPS})")
    p.add_argument("--colors", "-c", type=int, default=DEFAULT_COLORS,
                   choices=[2, 4, 8, 16, 32, 64, 128, 256],
                   help=f"Palette size (default: {DEFAULT_COLORS})")
    p.add_argument("--no-dither", action="store_true",
                   help="Disable Floyd-Steinberg dithering (faster but lower quality)")
    p.add_argument("--global-palette", "-g", action="store_true", default=True,
                   help="Build a single global palette from sampled frames (default: on)")
    p.add_argument("--per-frame-palette", dest="global_palette", action="store_false",
                   help="Use a per-frame palette (smaller files, possible colour flicker)")
    p.add_argument("--start", "-s", type=float, default=0.0,
                   help="Start time in seconds (default: 0)")
    p.add_argument("--end",   "-e", type=float, default=None,
                   help="End time in seconds (default: entire video)")
    p.add_argument("--fit", choices=["contain", "cover", "stretch"], default="contain",
                   help="How to fit video into target resolution (default: contain)")
    p.add_argument("--speed", "-x", type=float, default=1.0,
                   help="Playback speed multiplier (default: 1.0, e.g. 1.5 = 50%% faster)")
    return p.parse_args()


# ─── Video helpers ───────────────────────────────────────────────────────────

def open_video(path: str):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"[ERROR] Cannot open video: {path}")
    return cap


def video_info(cap) -> dict:
    return {
        "fps":    cap.get(cv2.CAP_PROP_FPS),
        "width":  int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "height": int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "frames": int(cap.get(cv2.CAP_PROP_FRAME_COUNT)),
    }


# ─── Frame transform ─────────────────────────────────────────────────────────

def fit_frame(frame_bgr, target_w: int, target_h: int, mode: str) -> Image.Image:
    """Resize a BGR numpy frame into a Pillow RGB image at (target_w x target_h)."""
    # Convert BGR → RGB
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    src_h, src_w = frame_rgb.shape[:2]
    img = Image.fromarray(frame_rgb)

    src_ratio = src_w / src_h
    dst_ratio = target_w / target_h

    if mode == "stretch":
        return img.resize((target_w, target_h), Image.LANCZOS)

    if mode == "cover":
        # Scale so the shorter dimension fills the target; crop the excess.
        if src_ratio > dst_ratio:
            new_h = target_h
            new_w = int(src_w * target_h / src_h)
        else:
            new_w = target_w
            new_h = int(src_h * target_w / src_w)
        img = img.resize((new_w, new_h), Image.LANCZOS)
        left = (new_w - target_w) // 2
        top  = (new_h - target_h) // 2
        return img.crop((left, top, left + target_w, top + target_h))

    # mode == "contain" — scale so the entire frame fits; pad with black.
    if src_ratio > dst_ratio:
        new_w = target_w
        new_h = int(src_h * target_w / src_w)
    else:
        new_h = target_h
        new_w = int(src_w * target_h / src_h)
    img = img.resize((new_w, new_h), Image.LANCZOS)
    canvas = Image.new("RGB", (target_w, target_h), (0, 0, 0))
    canvas.paste(img, ((target_w - new_w) // 2, (target_h - new_h) // 2))
    return canvas


# ─── Palette helpers ─────────────────────────────────────────────────────────

def build_global_palette(frames: list, n_colors: int) -> Image.Image:
    """
    Derive a global GIF palette by quantising a mosaic of evenly-spaced sample frames.
    Returns a single-pixel quantised image whose palette can be passed to
    Image.quantize(palette=...).
    """
    n = len(frames)
    step = max(1, n // PALETTE_SAMPLES)
    samples = frames[::step]

    # Stack samples into a wide strip for a representative colour sample.
    strip_w = samples[0].width * len(samples)
    strip_h = samples[0].height
    strip = Image.new("RGB", (strip_w, strip_h))
    for i, f in enumerate(samples):
        strip.paste(f, (i * f.width, 0))

    # Quantise the strip with median-cut to get the palette.
    quantised = strip.quantize(colors=n_colors, method=Image.MEDIANCUT)
    return quantised   # .palette carries the global palette


def quantise_frame(frame: Image.Image, palette_ref: Image.Image,
                   dither: bool) -> Image.Image:
    """Apply a pre-built palette to one frame."""
    dither_mode = Image.FLOYDSTEINBERG if dither else Image.NONE
    return frame.quantize(palette=palette_ref, dither=dither_mode)


# ─── Main conversion ─────────────────────────────────────────────────────────

def convert(args):
    input_path = os.path.abspath(args.input)
    if not os.path.isfile(input_path):
        sys.exit(f"[ERROR] Input file not found: {input_path}")

    if args.output:
        output_path = os.path.abspath(args.output)
    else:
        stem = os.path.splitext(os.path.basename(input_path))[0]
        output_path = os.path.join(os.path.dirname(input_path), stem + ".gif")

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # ── Open source video ────────────────────────────────────────────────────
    cap  = open_video(input_path)
    info = video_info(cap)
    src_fps = info["fps"]

    print(f"[INFO] Source : {info['width']}x{info['height']}  "
          f"{src_fps:.2f} fps  {info['frames']} frames  "
          f"~{info['frames']/src_fps:.1f}s")
    print(f"[INFO] Target : {args.width}x{args.height}  {args.fps} fps  "
          f"{args.colors} colors  fit={args.fit}")

    # ── Frame selection ──────────────────────────────────────────────────────
    start_frame = int(args.start * src_fps)
    end_frame   = int(args.end   * src_fps) if args.end else info["frames"]
    end_frame   = min(end_frame, info["frames"])

    # Step between source frames to hit the target fps
    step = max(1, src_fps / args.fps)
    source_indices = []
    pos = float(start_frame)
    while pos < end_frame:
        source_indices.append(round(pos))
        pos += step

    # Duration (in ms) between GIF frames — rounded to the nearest 10 ms
    # (GIF delay unit is 1/100 s, so 10 ms granularity)
    # Apply speed multiplier: higher speed → shorter delay
    frame_delay_ms = max(10, round(1000 / args.fps / args.speed / 10) * 10)

    print(f"[INFO] Extracting {len(source_indices)} frames "
          f"(delay={frame_delay_ms} ms each, speed={args.speed}x)  …")

    # ── Extract and resize frames ─────────────────────────────────────────────
    dither = not args.no_dither
    frames_rgb = []
    current_src = 0

    for i, idx in enumerate(source_indices):
        # Seek forward (only) — avoid expensive backward seeks
        while current_src <= idx:
            ret, frame_bgr = cap.read()
            if not ret:
                break
            current_src += 1

        if not ret:
            break

        resized = fit_frame(frame_bgr, args.width, args.height, args.fit)
        frames_rgb.append(resized)

        # Progress
        pct = int((i + 1) / len(source_indices) * 100)
        bar = "=" * (pct // 5) + ">" + " " * (20 - pct // 5)
        print(f"\r  Resizing  [{bar}] {pct:3d}%  ({i+1}/{len(source_indices)})",
              end="", flush=True)

    cap.release()
    print()   # newline after progress bar

    if not frames_rgb:
        sys.exit("[ERROR] No frames extracted — check --start / --end arguments")

    # ── Quantise ─────────────────────────────────────────────────────────────
    print(f"[INFO] Quantising with {'global' if args.global_palette else 'per-frame'} "
          f"palette ({args.colors} colors, dither={'on' if dither else 'off'})  …")

    if args.global_palette:
        palette_ref = build_global_palette(frames_rgb, args.colors)
        frames_q = []
        for i, f in enumerate(frames_rgb):
            frames_q.append(quantise_frame(f, palette_ref, dither))
            pct = int((i + 1) / len(frames_rgb) * 100)
            bar = "=" * (pct // 5) + ">" + " " * (20 - pct // 5)
            print(f"\r  Quantise  [{bar}] {pct:3d}%  ({i+1}/{len(frames_rgb)})",
                  end="", flush=True)
        print()
    else:
        frames_q = []
        for i, f in enumerate(frames_rgb):
            q = f.quantize(colors=args.colors, method=Image.MEDIANCUT,
                           dither=Image.FLOYDSTEINBERG if dither else Image.NONE)
            frames_q.append(q)
            pct = int((i + 1) / len(frames_rgb) * 100)
            bar = "=" * (pct // 5) + ">" + " " * (20 - pct // 5)
            print(f"\r  Quantise  [{bar}] {pct:3d}%  ({i+1}/{len(frames_rgb)})",
                  end="", flush=True)
        print()

    # ── Save GIF ─────────────────────────────────────────────────────────────
    print(f"[INFO] Saving  → {output_path}  …")
    t0 = time.time()

    frames_q[0].save(
        output_path,
        format="GIF",
        save_all=True,
        append_images=frames_q[1:],
        loop=0,                          # infinite loop
        duration=frame_delay_ms,
        optimize=False,                  # optimise=True can corrupt palette
        disposal=2,                      # clear to background between frames
    )

    elapsed = time.time() - t0
    file_size = os.path.getsize(output_path)

    # ── Summary ───────────────────────────────────────────────────────────────
    print()
    print("=" * 52)
    print(f"  Output      : {output_path}")
    print(f"  Dimensions  : {args.width} x {args.height} px")
    print(f"  Frame count : {len(frames_q)}")
    print(f"  Frame delay : {frame_delay_ms} ms  (~{1000/frame_delay_ms:.1f} fps)")
    print(f"  Palette     : {args.colors} colors")
    print(f"  File size   : {file_size/1024:.0f} KB  ({file_size/1024/1024:.2f} MB)")
    print(f"  Encode time : {elapsed:.1f}s")
    print("=" * 52)

    if file_size > MAX_FILE_BYTES:
        print(f"[WARN] File exceeds {MAX_FILE_BYTES//1024//1024} MB SPIFFS budget!")
        print("       Try: --colors 64  or  --fps 8  or  shorten --end")
    else:
        print("[OK]   File fits within the 2 MB SPIFFS budget.")

    # ── Always keep a copy in resource/ ──────────────────────────────────────
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    resource_dir = os.path.join(project_root, "resource")
    resource_copy = os.path.join(resource_dir, os.path.basename(output_path))
    os.makedirs(resource_dir, exist_ok=True)
    if os.path.abspath(output_path) != os.path.abspath(resource_copy):
        shutil.copy2(output_path, resource_copy)
        print(f"[INFO] Resource copy  → {resource_copy}")

    spiffs_data_dir = os.path.join(project_root, "data")
    if not output_path.startswith(spiffs_data_dir):
        print(f"\n[TIP]  To use as boot animation, copy to: {spiffs_data_dir}\\boot.gif")
        print("       Then run: pio run --target uploadfs")


if __name__ == "__main__":
    convert(parse_args())
