"""
overlay_pip.py  -  Picture-in-picture: embed one video in a corner of another.

Embeds OVERLAY_VIDEO (e.g. IMG_0488.MOV) into a corner of MAIN_VIDEO
(e.g. boomliftlimitstestbyagents.mp4) and re-encodes to a single MP4.

The two source clips can be completely different formats / codecs / colour
spaces (the overlay here is 10-bit HDR HEVC .MOV, the main is 8-bit H.264 .mp4)
- everything is decoded, converted and re-encoded, so mismatches are handled.

------------------------------------------------------------------------------
TWEAKING THE SYNC
------------------------------------------------------------------------------
The two knobs you care about live in the "SYNC / TIMING" block:

  OVERLAY_APPEAR_AT     WHEN (seconds on the MAIN video's clock) the little
                        video pops into the corner. Set this to the timestamp
                        of the event in the main video you want to line up with.

  OVERLAY_SOURCE_START  WHERE inside the overlay clip playback begins (skip its
                        first N seconds). Use this to line the overlay's own
                        action up with the main-video event.

Iterate quickly with the PREVIEW block (renders only a few seconds) until the
timing looks right, then set PREVIEW_SECONDS = None for the full render.
------------------------------------------------------------------------------
"""

from pathlib import Path
import re
import shutil
import subprocess
import sys

# =============================================================================
# CONFIGURATION  -  edit this section
# =============================================================================

# ---- FILES (relative to this script) ----------------------------------------
MAIN_VIDEO    = "BrakemodulecorrectFltBodydetect.mp4"   # the big / background video
OVERLAY_VIDEO = "BrakeCorrectFaultBodyfault.mp4"                      # the small corner video
OUTPUT_VIDEO  = "AgentDiagnoseTrueFailure.mp4"

# ---- SYNC / TIMING (all in seconds) -----------------------------------------
# WHEN the overlay appears on the MAIN video's timeline.
OVERLAY_APPEAR_AT    = 5
# WHERE to start playing inside the overlay clip (trim off its head).
OVERLAY_SOURCE_START = 0.0
# How long to keep the overlay on screen. None = until the overlay (or main) ends.
OVERLAY_DURATION     = None

# ---- LAYOUT -----------------------------------------------------------------
CORNER             = "top-right"   # top-right | top-left | bottom-right | bottom-left
OVERLAY_WIDTH_FRAC = 0.15        # overlay width as a fraction of the main width
MARGIN             = 24            # px gap between the overlay and the frame edges
BORDER_PX          = 4             # px border drawn around the overlay (0 = none)
BORDER_COLOR       = "white"

# ---- AUDIO ------------------------------------------------------------------
#   main    -> keep only the main video's audio
#   overlay -> keep only the overlay clip's audio (trimmed + delayed to match)
#   mix     -> mix both together
#   none    -> silent output
AUDIO_MODE = "main"

# ---- ENCODING ---------------------------------------------------------------
CRF    = 20            # x264 quality (lower = better/larger, 18-24 is sensible)
PRESET = "veryfast"    # x264 speed/quality trade-off (ultrafast .. veryslow)

# ---- PREVIEW (fast iteration) ----------------------------------------------
# Render only a short window so you can check the sync quickly.
# PREVIEW_START   : where on the MAIN timeline to start the preview (seconds)
# PREVIEW_SECONDS : how many seconds to render; None = render the whole thing.
PREVIEW_START   = 0
PREVIEW_SECONDS = None    # <-- set to None for the full-length render

# =============================================================================
# IMPLEMENTATION  -  you shouldn't need to edit below here
# =============================================================================

BASE_DIR = Path(__file__).parent


def find_ffmpeg() -> str:
    """Locate an ffmpeg binary: bundled (imageio-ffmpeg) or on PATH."""
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        exe = shutil.which("ffmpeg")
        if exe:
            return exe
    sys.exit("[ERROR] ffmpeg not found. Install it with:  pip install imageio-ffmpeg")


def probe_size(ffmpeg: str, path: Path) -> tuple[int, int]:
    """Return (width, height) of the first video stream by parsing ffmpeg -i."""
    out = subprocess.run(
        [ffmpeg, "-hide_banner", "-i", str(path)],
        stderr=subprocess.PIPE, stdout=subprocess.DEVNULL, text=True,
    ).stderr
    for line in out.splitlines():
        if "Video:" in line:
            m = re.search(r"(\d{2,5})x(\d{2,5})", line)
            if m:
                return int(m.group(1)), int(m.group(2))
    sys.exit(f"[ERROR] Could not determine resolution of {path}")


def even(n: int) -> int:
    """x264 needs even dimensions."""
    return n if n % 2 == 0 else n - 1


def corner_position(corner: str, margin: int) -> str:
    """overlay x:y expression for the chosen corner (W/w = main/overlay width)."""
    m = margin
    return {
        "top-right":    f"x=main_w-overlay_w-{m}:y={m}",
        "top-left":     f"x={m}:y={m}",
        "bottom-right": f"x=main_w-overlay_w-{m}:y=main_h-overlay_h-{m}",
        "bottom-left":  f"x={m}:y=main_h-overlay_h-{m}",
    }.get(corner, f"x=main_w-overlay_w-{m}:y={m}")


def build_command(ffmpeg: str) -> list[str]:
    main_path    = BASE_DIR / MAIN_VIDEO
    overlay_path = BASE_DIR / OVERLAY_VIDEO
    out_path     = BASE_DIR / OUTPUT_VIDEO

    for p in (main_path, overlay_path):
        if not p.exists():
            sys.exit(f"[ERROR] File not found: {p}")

    main_w, _ = probe_size(ffmpeg, main_path)
    ow = even(max(2, int(main_w * OVERLAY_WIDTH_FRAC)))

    # Preview support: fast-seek the MAIN input and shrink the appear time to
    # match, so the overlay still lands on the same main-video event.
    preview_start = PREVIEW_START if PREVIEW_SECONDS is not None else 0.0
    appear = max(0.0, OVERLAY_APPEAR_AT - preview_start)

    # ---- overlay (video) filter chain -------------------------------------
    src_end = ""
    if OVERLAY_DURATION is not None:
        src_end = f":end={OVERLAY_SOURCE_START + OVERLAY_DURATION}"

    pip = (
        f"[1:v]trim=start={OVERLAY_SOURCE_START}{src_end},"
        f"setpts=PTS-STARTPTS+{appear}/TB,"
        f"scale={ow}:-2,format=yuv420p"
    )
    b = BORDER_PX
    if b > 0:
        pip += f",pad=iw+{2*b}:ih+{2*b}:{b}:{b}:color={BORDER_COLOR}"
    pip += "[pip]"

    if OVERLAY_DURATION is not None:
        enable = f"between(t\\,{appear}\\,{appear + OVERLAY_DURATION})"
    else:
        enable = f"gte(t\\,{appear})"

    pos = corner_position(CORNER, MARGIN)
    video = (
        f"{pip};"
        f"[0:v][pip]overlay={pos}:eof_action=pass:enable='{enable}'[v]"
    )

    # ---- audio filter chain / mapping -------------------------------------
    filtergraph = video
    audio_maps: list[str] = []

    if AUDIO_MODE == "main":
        audio_maps = ["-map", "0:a?"]
    elif AUDIO_MODE == "none":
        audio_maps = ["-an"]
    elif AUDIO_MODE in ("overlay", "mix"):
        delay_ms = int(appear * 1000)
        oa = (
            f"[1:a]atrim=start={OVERLAY_SOURCE_START},asetpts=PTS-STARTPTS,"
            f"adelay={delay_ms}|{delay_ms}[oa]"
        )
        if AUDIO_MODE == "overlay":
            filtergraph += ";" + oa
            audio_maps = ["-map", "[oa]"]
        else:  # mix
            filtergraph += (
                ";" + oa +
                ";[0:a][oa]amix=inputs=2:duration=first:dropout_transition=0[a]"
            )
            audio_maps = ["-map", "[a]"]
    else:
        sys.exit(f"[ERROR] Unknown AUDIO_MODE: {AUDIO_MODE!r}")

    # ---- assemble command --------------------------------------------------
    cmd = [ffmpeg, "-hide_banner", "-y"]

    if PREVIEW_SECONDS is not None and preview_start > 0:
        cmd += ["-ss", str(preview_start)]     # fast input seek on the MAIN video
    cmd += ["-i", str(main_path)]
    cmd += ["-i", str(overlay_path)]

    cmd += ["-filter_complex", filtergraph, "-map", "[v]"]
    cmd += audio_maps

    cmd += [
        "-c:v", "libx264", "-crf", str(CRF), "-preset", PRESET,
        "-pix_fmt", "yuv420p",
        "-c:a", "aac", "-b:a", "192k",
        "-movflags", "+faststart",
    ]
    if PREVIEW_SECONDS is not None:
        cmd += ["-t", str(PREVIEW_SECONDS)]

    cmd += [str(out_path)]
    return cmd


def main() -> None:
    ffmpeg = find_ffmpeg()
    cmd = build_command(ffmpeg)

    print("Running ffmpeg:\n  " + " ".join(f'"{c}"' if " " in c else c for c in cmd) + "\n")
    if PREVIEW_SECONDS is not None:
        print(f"[PREVIEW] Rendering {PREVIEW_SECONDS}s from {PREVIEW_START}s "
              f"- set PREVIEW_SECONDS = None for the full render.\n")

    result = subprocess.run(cmd)
    if result.returncode != 0:
        sys.exit(f"[ERROR] ffmpeg exited with code {result.returncode}")

    print(f"\nDone -> {BASE_DIR / OUTPUT_VIDEO}")


if __name__ == "__main__":
    main()
