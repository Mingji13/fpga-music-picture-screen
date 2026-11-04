#!/usr/bin/env python3
"""Offline beat review and sample-indexed image cues for the music screen."""

import argparse
import base64
import csv
import hashlib
import importlib.util
import json
import math
import re
import shutil
import sys
import wave
from datetime import datetime
from pathlib import Path

import numpy as np

import cue_format

RATE = 48000
ANALYSIS_RATE = 24000
HOP = 128
ROOT = Path(__file__).resolve().parent
SCHEMA = "music-screen-cues-v1"


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_pcm(path):
    size = Path(path).stat().st_size
    if size == 0 or size % 2:
        raise ValueError("PCM must contain a nonempty, even number of bytes (s16le mono).")
    with Path(path).open("rb") as source:
        if source.read(4) in (b"RIFF", b"RF64"):
            raise ValueError("This is a WAV container, not raw PCM. Use its .wav extension.")
    samples = np.fromfile(path, dtype="<i2")
    if len(samples) >= 2**32:
        raise ValueError("This version uses 32-bit sample positions (maximum about 24 hours).")
    return samples


def prepare_pcm(input_path, destination):
    if input_path.suffix.lower() == ".pcm":
        read_pcm(input_path)
        shutil.copyfile(input_path, destination)
    elif input_path.suffix.lower() == ".wav":
        # Reuse the exact board converter, including its downmix/resampling.
        spec = importlib.util.spec_from_file_location("board_wav_converter", ROOT.parent / "make_audio_corpus.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.convert(input_path, destination, False)
    else:
        raise ValueError("Select a .wav or raw 48 kHz mono s16le .pcm file.")
    return read_pcm(destination)


def grid_samples(bpm, first_downbeat, sample_count):
    if not math.isfinite(bpm) or not 20 <= bpm <= 400:
        raise ValueError("BPM must be between 20 and 400.")
    if not math.isfinite(first_downbeat) or not 0 <= first_downbeat < sample_count / RATE:
        raise ValueError("First downbeat must be inside the song.")
    first = first_downbeat * RATE
    step = RATE * 60.0 / bpm
    count = int(math.ceil((sample_count - first) / step))
    # Compute every position from the original reference; never accumulate a rounded interval.
    return [int(math.floor(first + i * step + 0.5)) for i in range(count)
            if int(math.floor(first + i * step + 0.5)) < sample_count]


def make_cues(beats, anchor, meter, bars, start_after, sample_count):
    if type(meter) is not int or not 2 <= meter <= 12:
        raise ValueError("Beats per bar must be an integer between 2 and 12.")
    if type(bars) is not int or bars not in (1, 2):
        raise ValueError("Choose one or two bars per picture.")
    if not math.isfinite(start_after) or not 0 <= start_after < sample_count / RATE:
        raise ValueError("Start-after time must be inside the song.")
    if not beats:
        if anchor != 0:
            raise ValueError("An empty beat list requires anchor 0.")
        return []
    if type(anchor) is not int or not 0 <= anchor < len(beats):
        raise ValueError("Downbeat reference is outside the beat list.")
    if any(type(x) is not int or x < 0 or x >= sample_count for x in beats):
        raise ValueError("Beat positions must be integer sample indices within the song.")
    if any(b <= a for a, b in zip(beats, beats[1:])):
        raise ValueError("Beat positions must be strictly increasing.")
    threshold = int(math.floor(start_after * RATE + 0.5))
    return [{"sample": beats[i], "seconds": beats[i] / RATE,
             "beat_index": i, "bar_index": 1 + (i - anchor) // meter}
            for i in range(anchor, len(beats), meter * bars) if beats[i] >= threshold]


def analyze_beats(pcm, meter, bpm_hint=None):
    import librosa
    from scipy.signal import resample_poly

    notes = ["Meter is a user assumption, not automatically identified.",
             "Downbeat phase is an accent heuristic, not a trained downbeat detector; audition required.",
             "The tracker uses a global tempo prior; rubato, tempo/meter changes and half/double tempo need review."]
    y = resample_poly(pcm.astype(np.float32) / 32768.0, 1, 2)
    if len(y) < ANALYSIS_RATE * 2 or np.max(np.abs(y)) < 1e-5:
        return [], 0, {"estimated_bpm": None, "phase_scores": [],
                       "warnings": notes + ["Too short or silent: no automatic cues generated."]}
    # Keep spectral work on the PC; the board only receives reviewed sample positions.
    mel = librosa.feature.melspectrogram(y=y, sr=ANALYSIS_RATE, n_fft=1024,
                                        hop_length=HOP, n_mels=64, fmax=10000)
    log_mel = librosa.power_to_db(mel, ref=np.max)
    onset = librosa.onset.onset_strength(S=log_mel, sr=ANALYSIS_RATE, hop_length=HOP)
    if np.max(onset) <= 1e-8:
        return [], 0, {"estimated_bpm": None, "phase_scores": [],
                       "warnings": notes + ["No usable onset pattern: no automatic cues generated."]}
    tempo, frames = librosa.beat.beat_track(onset_envelope=onset, sr=ANALYSIS_RATE,
                                          hop_length=HOP, start_bpm=bpm_hint or 120,
                                          bpm=bpm_hint, trim=True)
    beats = sorted(set(int(x) * HOP * (RATE // ANALYSIS_RATE) for x in frames
                       if int(x) * HOP * (RATE // ANALYSIS_RATE) < len(pcm)))
    if len(beats) < meter * 2:
        return [], 0, {"estimated_bpm": None, "phase_scores": [],
                       "warnings": notes + ["Too few tracked beats: use a manually checked grid."]}
    # Low-band and broadband onsets provide only an audition starting point.
    frequencies = librosa.mel_frequencies(n_mels=64, fmin=0, fmax=10000)
    low = librosa.onset.onset_strength(S=log_mel[frequencies < 250],
                                      sr=ANALYSIS_RATE, hop_length=HOP)
    strength = onset / max(float(np.max(onset)), 1e-9) + low / max(float(np.max(low)), 1e-9)
    accents = []
    for sample in beats:
        frame = sample // (HOP * 2)
        accents.append(float(np.max(strength[max(0, frame - 3):min(len(strength), frame + 4)])))
    # Median across repeated bars discourages one isolated loud hit from choosing the phase.
    phase_scores = [float(np.median(accents[p::meter])) if accents[p::meter] else 0
                    for p in range(meter)]
    anchor = int(np.argmax(phase_scores))
    intervals = np.diff(beats) / RATE
    estimated = float(np.asarray(tempo).reshape(-1)[0])
    return beats, anchor, {"estimated_bpm": estimated, "phase_scores": phase_scores,
                          "beat_interval_seconds_p10_p50_p90": np.quantile(intervals, [.1, .5, .9]).tolist(),
                          "analysis_hop_seconds": HOP / ANALYSIS_RATE,
                          "warnings": notes}


def write_wav(path, samples):
    with wave.open(str(path), "wb") as target:
        target.setnchannels(1)
        target.setsampwidth(2)
        target.setframerate(RATE)
        target.writeframes(np.asarray(samples, dtype="<i2").tobytes())


def preview_samples(pcm, beats, cues, anchor, meter, mode):
    mixed = pcm.astype(np.float64) / 32768.0 * 0.65
    events = [(x, i >= anchor and (i - anchor) % meter == 0) for i, x in enumerate(beats)]
    if mode == "cuts":
        events = [(cue["sample"], True) for cue in cues]
    for position, strong in events:
        length = min(int(RATE * 0.035), len(pcm) - position)
        t = np.arange(length) / RATE
        frequency = (1760 if mode == "cuts" else 1320) if strong else 880
        amplitude = 0.28 if strong else 0.14
        mixed[position:position + length] += amplitude * np.sin(2 * np.pi * frequency * t) * np.exp(-t * 100)
    return np.rint(np.clip(mixed, -1, 32767 / 32768) * 32768).astype("<i2")


def validate_schedule(schedule, pcm_path):
    pcm = read_pcm(pcm_path)
    if schedule.get("schema") != SCHEMA:
        raise ValueError("Unknown schedule schema.")
    fmt = schedule["pcm"]
    if (fmt["sample_rate"], fmt["channels"], fmt["encoding"], fmt["sample_count"]) != (RATE, 1, "s16le", len(pcm)):
        raise ValueError("Schedule PCM format or length mismatch.")
    if fmt["sha256"].lower() != sha256(pcm_path):
        raise ValueError("Schedule belongs to different PCM content (SHA256 mismatch).")
    if type(schedule.get("reviewed")) is not bool:
        raise ValueError("reviewed must be true or false.")
    settings = schedule["settings"]
    if settings["mode"] not in ("tracked", "grid"):
        raise ValueError("Unknown beat mode.")
    beats = schedule["beat_samples"]
    if settings["mode"] == "grid":
        expected = grid_samples(settings["bpm"], settings["first_downbeat_seconds"], len(pcm))
        if beats != expected:
            raise ValueError("Grid positions disagree with BPM/reference.")
    cues = make_cues(beats, settings["anchor_beat"], settings["beats_per_bar"],
                     settings["bars_per_change"], settings["start_after_seconds"], len(pcm))
    if schedule["cues"] != cues:
        raise ValueError("Cue positions disagree with beat list/settings; regenerate the schedule.")
    return pcm


def write_bundle(directory, pcm, schedule):
    beats, settings, cues = schedule["beat_samples"], schedule["settings"], schedule["cues"]
    (directory / "schedule.json").write_text(json.dumps(schedule, ensure_ascii=False, indent=2), encoding="utf-8")
    with (directory / "cues.csv").open("w", newline="", encoding="utf-8-sig") as target:
        writer = csv.writer(target)
        writer.writerow(["sample_index_zero_based", "seconds", "bar_index", "beat_index_zero_based"])
        for cue in cues:
            writer.writerow([cue["sample"], f'{cue["seconds"]:.9f}', cue["bar_index"], cue["beat_index"]])
    write_wav(directory / "music.wav", pcm)
    for mode in ("beats", "cuts"):
        write_wav(directory / (mode + "_preview.wav"), preview_samples(
            pcm, beats, cues, settings["anchor_beat"], settings["beats_per_bar"], mode))
    peaks = [float(np.max(np.abs(part.astype(np.float32))) / 32768)
             for part in np.array_split(pcm, min(1600, len(pcm)))]
    data = {"schedule": schedule, "peaks": peaks}
    safe_data = json.dumps(data, ensure_ascii=True).replace("<", "\\u003c").replace(">", "\\u003e").replace("&", "\\u0026")
    template = (ROOT / "review.html").read_text(encoding="utf-8")
    html = template.replace("/*__DATA__*/", safe_data).replace("__PCM_BASE64__", base64.b64encode(pcm.astype("<i2").tobytes()).decode("ascii"))
    (directory / "review.html").write_text(html, encoding="utf-8")


def new_directory(requested, title):
    path = requested or ROOT / "runs" / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") + "_" + title[:40])
    path = Path(path).resolve()
    path.mkdir(parents=True, exist_ok=False)  # Never replace an existing run or song.
    return path


def export_cue(schedule_path, pcm_path, output_path=None):
    schedule_path, pcm_path = Path(schedule_path), Path(pcm_path)
    schedule = json.loads(schedule_path.read_text(encoding="utf-8-sig"))
    validate_schedule(schedule, pcm_path)
    if schedule["reviewed"] is not True:
        raise ValueError("Audition and confirm the schedule before FPGA CUE export (reviewed must be true).")
    if not re.fullmatch(r"SONG(?:[1-9]|1[0-6])\.PCM", pcm_path.name, flags=re.IGNORECASE):
        raise ValueError("Select the final SONG1.PCM..SONG16.PCM so the CUE is paired by song number.")
    expected_name = pcm_path.stem.upper() + ".CUE"
    output_path = Path(output_path) if output_path else pcm_path.with_name(expected_name)
    if output_path.name.upper() != expected_name:
        raise ValueError(f"CUE must have the matching filename {expected_name}.")
    encoded = cue_format.encode([cue["sample"] for cue in schedule["cues"]],
                                schedule["pcm"]["sample_count"], schedule["pcm"]["sha256"])
    cue_format.decode(encoded)
    # Exclusive creation: never silently overwrite a previous export or an input file.
    with output_path.open("xb") as target:
        target.write(encoded)
    return output_path, cue_format.decode(output_path.read_bytes())


def inspect_cue(cue_path, pcm_path=None):
    info = cue_format.decode(Path(cue_path).read_bytes())
    if pcm_path is not None:
        pcm = read_pcm(pcm_path)
        if len(pcm) != info["sample_count"] or sha256(pcm_path) != info["pcm_sha256"]:
            raise ValueError("CUE belongs to different PCM content (length/SHA256 mismatch).")
    return info


def choose_file(kind="audio", initialdir=None):
    import tkinter as tk
    from tkinter import filedialog
    root = tk.Tk()
    root.withdraw()
    try:
        titles = {"audio": "选择 WAV 或 48kHz 单声道 PCM", "schedule": "选择已试听确认的时间表 JSON",
                  "pcm": "选择配套的 SONG1.PCM～SONG16.PCM"}
        patterns = {"audio": "*.wav *.pcm", "schedule": "*.json", "pcm": "*.pcm"}
        return filedialog.askopenfilename(title=titles[kind], initialdir=str(initialdir) if initialdir else None,
                                          filetypes=[("Music data", patterns[kind]), ("All", "*.*")])
    finally:
        root.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description="PC-only beat audition and image cue preparation. Board RTL is not changed.")
    commands = parser.add_subparsers(dest="command", required=True)
    analyze = commands.add_parser("analyze", help="Convert/copy audio, find candidate beats and create an interactive review.")
    analyze.add_argument("input", type=Path, nargs="?", help="WAV or raw 48000 Hz mono signed 16-bit LE PCM; omitted: file picker")
    analyze.add_argument("--output", type=Path, help="New output directory (must not exist)")
    analyze.add_argument("--beats-per-bar", type=int, default=4, help="Assumed meter, default 4; not auto-detected")
    analyze.add_argument("--bars", type=int, choices=[1, 2], default=2)
    analyze.add_argument("--bpm", type=float, help="Tempo prior for tracking, or exact BPM for --grid")
    analyze.add_argument("--first-downbeat", type=float, help="Seconds: nearest tracked beat or exact grid reference")
    analyze.add_argument("--grid", action="store_true", help="Use fixed BPM; requires --bpm and --first-downbeat")
    analyze.add_argument("--start-after", type=float, default=0.0, help="Skip earlier cues, preserving bar phase")
    analyze.add_argument("--open", action="store_true", help="Open the review page after completion")
    render = commands.add_parser("render", help="Validate a reviewed JSON against PCM and render exact preview WAVs")
    render.add_argument("schedule", type=Path)
    render.add_argument("--pcm", type=Path, required=True)
    render.add_argument("--output", type=Path)
    export = commands.add_parser("export-cue", help="Export a confirmed JSON and matching SONGn.PCM to SONGn.CUE")
    export.add_argument("schedule", type=Path, nargs="?", help="Confirmed JSON; omitted: file picker")
    export.add_argument("--pcm", type=Path, help="Matching final SONG1.PCM..SONG16.PCM; omitted: file picker")
    export.add_argument("--output", type=Path, help="New matching SONGn.CUE; default: beside the PCM; never overwritten")
    inspect = commands.add_parser("inspect-cue", help="Validate a binary CUE; optionally check its PCM binding")
    inspect.add_argument("cue", type=Path)
    inspect.add_argument("--pcm", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command in ("export-cue", "inspect-cue"):
            if args.command == "export-cue":
                selected = args.schedule or choose_file("schedule")
                if not selected:
                    print("Cancelled.")
                    return 0
                schedule_path = Path(selected).resolve(strict=True)
                selected_pcm = args.pcm or choose_file("pcm", schedule_path.parent)
                if not selected_pcm:
                    print("Cancelled.")
                    return 0
                path, info = export_cue(schedule_path, selected_pcm, args.output)
                print(f"Created: {path}\nBytes: {path.stat().st_size}; PCM binding verified.")
            else:
                info = inspect_cue(args.cue, args.pcm)
                print("CUE integrity verified." + (" PCM binding verified." if args.pcm else " PCM binding not checked."))
            print(f"MCUE v{info['version']}: {info['cue_count']} cues; {info['sample_count']} mono s16le samples at {info['sample_rate']} Hz")
            print(f"First cue: sample {info['positions'][0]} ({info['positions'][0] / RATE:.6f} s)")
            print(f"Last cue: sample {info['positions'][-1]} ({info['positions'][-1] / RATE:.6f} s)")
            print("Use CUE-enabled FPGA firmware; copy matching SONGn.PCM and SONGn.CUE to the SD root.")
            return 0
        if args.command == "analyze":
            selected = args.input or choose_file()
            if not selected:
                print("Cancelled.")
                return 0
            input_path = Path(selected).resolve(strict=True)
            if not 2 <= args.beats_per_bar <= 12:
                raise ValueError("--beats-per-bar must be between 2 and 12.")
            if args.bpm is not None and (not math.isfinite(args.bpm) or not 20 <= args.bpm <= 400):
                raise ValueError("--bpm must be between 20 and 400.")
            if args.grid and (args.bpm is None or args.first_downbeat is None):
                raise ValueError("--grid requires --bpm and --first-downbeat.")
            directory = new_directory(args.output, input_path.stem)
            print("Preparing final board-format PCM...", flush=True)
            pcm = prepare_pcm(input_path, directory / "audio.pcm")
            if args.grid:
                beats = grid_samples(args.bpm, args.first_downbeat, len(pcm))
                anchor = 0
                analysis = {"estimated_bpm": args.bpm, "warnings": ["Fixed grid: user must verify tempo and downbeat reference throughout the song."]}
            else:
                print("Analyzing candidate beats on the PC (first run may compile numerical kernels)...", flush=True)
                beats, anchor, analysis = analyze_beats(pcm, args.beats_per_bar, args.bpm)
                if args.first_downbeat is not None:
                    if not math.isfinite(args.first_downbeat) or not 0 <= args.first_downbeat < len(pcm) / RATE or not beats:
                        raise ValueError("Cannot anchor this time to a tracked beat. Try an explicit --grid.")
                    anchor = min(range(len(beats)), key=lambda i: abs(beats[i] / RATE - args.first_downbeat))
                    analysis["anchor_snap_error_seconds"] = beats[anchor] / RATE - args.first_downbeat
            settings = {"mode": "grid" if args.grid else "tracked", "beats_per_bar": args.beats_per_bar,
                        "bars_per_change": args.bars, "anchor_beat": anchor, "start_after_seconds": args.start_after,
                        "bpm": args.bpm if args.grid else analysis.get("estimated_bpm"),
                        "first_downbeat_seconds": beats[anchor] / RATE if beats else 0.0}
            if args.grid:
                settings["first_downbeat_seconds"] = args.first_downbeat
            cues = make_cues(beats, anchor, args.beats_per_bar, args.bars, args.start_after, len(pcm))
            schedule = {"schema": SCHEMA, "reviewed": False, "title": input_path.stem,
                        "created": datetime.now().astimezone().isoformat(),
                        "source": {"path": str(input_path), "sha256": sha256(input_path)},
                        "pcm": {"file": "audio.pcm", "sha256": sha256(directory / "audio.pcm"),
                                "sample_rate": RATE, "channels": 1, "encoding": "s16le", "sample_count": len(pcm)},
                        "settings": settings, "analysis": analysis, "beat_samples": beats,
                        "tracked_beat_samples": beats if not args.grid else [], "cues": cues,
                        "cue_semantics": "Start image transition at this zero-based played PCM sample index; WAV+auto only.",
                        "firmware_support": False}
        else:
            schedule = json.loads(args.schedule.read_text(encoding="utf-8-sig"))
            pcm = validate_schedule(schedule, args.pcm)
            directory = new_directory(args.output, "reviewed")
            shutil.copyfile(args.pcm, directory / "audio.pcm")
        print("Writing sample-aligned previews and review page...", flush=True)
        write_bundle(directory, pcm, schedule)
        validate_schedule(schedule, directory / "audio.pcm")
        print(f"Output: {directory}\nBeats: {len(schedule['beat_samples'])}; image cues: {len(schedule['cues'])}")
        print(f"Review status: {'user-confirmed' if schedule['reviewed'] else 'UNREVIEWED CANDIDATE'}")
        print("Open review.html, audition and confirm, then export-cue for CUE-enabled FPGA firmware.")
        if getattr(args, "open", False):
            import webbrowser
            webbrowser.open((directory / "review.html").as_uri())
        return 0
    except (OSError, ValueError, KeyError, TypeError, wave.Error, ImportError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
