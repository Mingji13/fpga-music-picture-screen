#!/usr/bin/env python3
"""Convert an uncompressed PCM WAV to 48 kHz, mono, signed 16-bit PCM."""

import argparse
import audioop
import sys
import wave
from pathlib import Path


TARGET_RATE = 48000


def convert(input_path: Path, output_path: Path, overwrite: bool) -> tuple[int, int, int]:
    if input_path.resolve() == output_path.resolve():
        raise ValueError("Input and output paths must be different")
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Output already exists: {output_path} (use --overwrite to replace it)")

    with wave.open(str(input_path), "rb") as source:
        channels = source.getnchannels()
        sample_rate = source.getframerate()
        sample_width = source.getsampwidth()
        frame_count = source.getnframes()

        if source.getcomptype() != "NONE":
            raise ValueError("Only uncompressed PCM WAV files are supported")
        if channels not in (1, 2):
            raise ValueError(f"Expected mono or stereo WAV, found {channels} channels")
        if sample_width not in (1, 2, 3, 4):
            raise ValueError(f"Unsupported PCM sample width: {sample_width} bytes")
        if sample_rate <= 0:
            raise ValueError(f"Invalid sample rate: {sample_rate}")

        pcm = source.readframes(frame_count)

    # WAV 8-bit PCM is unsigned; audioop expects signed samples.
    if sample_width == 1:
        pcm = audioop.bias(pcm, 1, -128)

    if channels == 2:
        pcm = audioop.tomono(pcm, sample_width, 0.5, 0.5)

    if sample_width != 2:
        pcm = audioop.lin2lin(pcm, sample_width, 2)

    if sample_rate != TARGET_RATE:
        pcm, _ = audioop.ratecv(pcm, 2, 1, sample_rate, TARGET_RATE, None)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("wb") as output:
        output.write(pcm)

    sample_count = len(pcm) // 2
    return sample_count, len(pcm), frame_count


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert PCM WAV to raw 48 kHz, mono, signed 16-bit little-endian PCM."
    )
    parser.add_argument("input_wav", type=Path, help="Input uncompressed PCM WAV")
    parser.add_argument("output_pcm", type=Path, help="Output raw PCM file")
    parser.add_argument("--overwrite", action="store_true", help="Replace an existing output file")
    args = parser.parse_args()

    try:
        samples, output_bytes, input_frames = convert(
            args.input_wav, args.output_pcm, args.overwrite
        )
    except (OSError, ValueError, wave.Error, audioop.error) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1

    duration = samples / TARGET_RATE
    print(f"Input frames: {input_frames}")
    print(f"Output: {args.output_pcm}")
    print(f"Format: {TARGET_RATE} Hz, mono, signed 16-bit little-endian PCM")
    print(f"Samples: {samples} ({duration:.2f} seconds)")
    print(f"Size: {output_bytes} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
