"""MCUE v1 binary interchange. Pure standard library; no audio analysis here."""

import struct

MAGIC = b"MCUE"
VERSION = 1
HEADER_BYTES = 64
RATE = 48000
MAX_CUES = 1024
FLAGS = 1  # Request start of image transition at a zero-based PCM sample index.
HEADER = struct.Struct("<4sHHIIII32sII")


def word_sum(data):
    if len(data) % 4:
        raise ValueError("CUE size must be a multiple of four bytes.")
    return sum(word[0] for word in struct.iter_unpack("<I", data)) & 0xFFFFFFFF


def validate_positions(positions, sample_count):
    if type(sample_count) is not int or not 1 <= sample_count <= 0xFFFFFFFF:
        raise ValueError("PCM sample count must fit in a nonzero unsigned 32-bit integer.")
    if not 1 <= len(positions) <= MAX_CUES:
        raise ValueError(f"CUE requires 1..{MAX_CUES} positions; split/reduce denser schedules.")
    if any(type(position) is not int or not 0 <= position < sample_count for position in positions):
        raise ValueError("CUE positions must be integer sample indices inside the PCM.")
    if any(second <= first for first, second in zip(positions, positions[1:])):
        raise ValueError("CUE positions must be strictly increasing.")


def encode(positions, sample_count, pcm_sha256):
    validate_positions(positions, sample_count)
    if not isinstance(pcm_sha256, str) or len(pcm_sha256) != 64:
        raise ValueError("PCM SHA256 must contain exactly 64 hex digits.")
    digest = bytes.fromhex(pcm_sha256)
    if len(digest) != 32:
        raise ValueError("Invalid PCM SHA256.")
    payload = struct.pack(f"<{len(positions)}I", *positions)
    header = HEADER.pack(MAGIC, VERSION, HEADER_BYTES, RATE, sample_count,
                         len(positions), FLAGS, digest, 0, 0)
    checksum = (-word_sum(header + payload)) & 0xFFFFFFFF
    header = HEADER.pack(MAGIC, VERSION, HEADER_BYTES, RATE, sample_count,
                         len(positions), FLAGS, digest, checksum, 0)
    return header + payload


def decode(data):
    if len(data) < HEADER_BYTES:
        raise ValueError("CUE header is truncated.")
    magic, version, header_bytes, rate, sample_count, count, flags, digest, checksum, reserved = HEADER.unpack_from(data)
    if (magic, version, header_bytes) != (MAGIC, VERSION, HEADER_BYTES):
        raise ValueError("Unsupported CUE magic, version or header length.")
    if rate != RATE or flags != FLAGS or reserved != 0:
        raise ValueError("Unsupported CUE sample rate, flags or reserved field.")
    if not 1 <= count <= MAX_CUES:
        raise ValueError("Unsupported CUE position count.")
    if len(data) != HEADER_BYTES + count * 4:
        raise ValueError("CUE size does not match its position count.")
    if word_sum(data) != 0:
        raise ValueError("CUE integrity checksum mismatch.")
    positions = list(struct.unpack_from(f"<{count}I", data, HEADER_BYTES))
    validate_positions(positions, sample_count)
    return {"version": version, "sample_rate": rate, "sample_count": sample_count,
            "cue_count": count, "flags": flags, "pcm_sha256": digest.hex(),
            "checksum": checksum, "positions": positions}
