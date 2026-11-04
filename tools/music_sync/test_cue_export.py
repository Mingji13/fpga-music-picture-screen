"""Binary layout, corruption rejection, and confirmed JSON/PCM export checks."""

import contextlib
import copy
import hashlib
import io
import json
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

import cue_format
import music_sync as sync


def repair_checksum(data):
    """Independent fixture helper: let semantic errors reach decoder checks."""
    result = bytearray(data)
    struct.pack_into("<I", result, 56, 0)
    total = sum(int.from_bytes(result[i:i + 4], "little") for i in range(0, len(result), 4))
    struct.pack_into("<I", result, 56, (-total) & 0xFFFFFFFF)
    return bytes(result)


class BinaryCueTests(unittest.TestCase):
    def setUp(self):
        self.digest = hashlib.sha256(b"fixture PCM").hexdigest()
        self.positions = [0, 48000, 191999]
        self.encoded = cue_format.encode(self.positions, 192000, self.digest)

    def test_exact_layout_endianness_and_checksum(self):
        data = self.encoded
        self.assertEqual(len(data), 64 + 3 * 4)
        self.assertEqual(data[:8], b"MCUE\x01\x00\x40\x00")
        self.assertEqual(struct.unpack_from("<IIII", data, 8), (48000, 192000, 3, 1))
        self.assertEqual(data[24:56], bytes.fromhex(self.digest))
        self.assertEqual(data[60:64], b"\x00" * 4)
        self.assertEqual(data[64:], b"\x00\x00\x00\x00\x80\xbb\x00\x00\xff\xed\x02\x00")
        total = sum(int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4))
        self.assertEqual(total & 0xFFFFFFFF, 0)
        info = cue_format.decode(data)
        self.assertEqual(info["positions"], self.positions)
        self.assertEqual(info["sample_count"], 192000)
        self.assertEqual(info["pcm_sha256"], self.digest)

    def test_position_and_count_boundaries(self):
        for positions, count in (([0], 1), ([0xFFFFFFFE], 0xFFFFFFFF),
                                 (list(range(1024)), 1024)):
            with self.subTest(count=count, cues=len(positions)):
                data = cue_format.encode(positions, count, self.digest)
                self.assertEqual(cue_format.decode(data)["positions"], positions)
        invalid = (([], 10), (list(range(1025)), 1025), ([-1], 10), ([10], 10),
                   ([1, 1], 10), ([2, 1], 10), ([0.0], 10), ([True], 10),
                   ([0], 0), ([0], 2**32), ([0], True))
        for positions, count in invalid:
            with self.subTest(positions=positions[:3], count=count):
                with self.assertRaises(ValueError):
                    cue_format.encode(positions, count, self.digest)

    def test_truncation_extra_bytes_and_corruption(self):
        for data in (b"", self.encoded[:63], self.encoded[:-1], self.encoded[:-4],
                     self.encoded + b"\x00", self.encoded + b"\x00" * 4):
            with self.subTest(length=len(data)):
                with self.assertRaises(ValueError):
                    cue_format.decode(data)
        corrupted = bytearray(self.encoded)
        corrupted[68] ^= 1
        with self.assertRaisesRegex(ValueError, "checksum"):
            cue_format.decode(corrupted)

    def test_rejects_unsupported_header_even_with_valid_checksum(self):
        fields = ((0, "4s", b"NOPE"), (4, "H", 2), (6, "H", 68),
                  (8, "I", 44100), (12, "I", 0), (16, "I", 0),
                  (16, "I", 1025), (20, "I", 0), (20, "I", 3), (60, "I", 1))
        for offset, fmt, value in fields:
            with self.subTest(offset=offset, value=value):
                data = bytearray(self.encoded)
                struct.pack_into("<" + fmt, data, offset, value)
                with self.assertRaises(ValueError):
                    cue_format.decode(repair_checksum(data))

    def test_rejects_invalid_payload_even_with_valid_checksum(self):
        for positions in ([0, 0, 191999], [48000, 0, 191999], [0, 48000, 192000]):
            with self.subTest(positions=positions):
                data = bytearray(self.encoded)
                struct.pack_into("<3I", data, 64, *positions)
                with self.assertRaises(ValueError):
                    cue_format.decode(repair_checksum(data))


class ExportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.pcm_path = self.root / "SONG2.PCM"
        pcm = np.arange(48000 * 10, dtype=np.int32).astype("<i2")
        self.pcm_path.write_bytes(pcm.tobytes())
        beats = sync.grid_samples(120, .25, len(pcm))
        self.schedule = {
            "schema": sync.SCHEMA, "reviewed": True,
            "pcm": {"sample_rate": 48000, "channels": 1, "encoding": "s16le",
                    "sample_count": len(pcm), "sha256": sync.sha256(self.pcm_path)},
            "settings": {"mode": "grid", "bpm": 120, "first_downbeat_seconds": .25,
                         "anchor_beat": 0, "beats_per_bar": 4, "bars_per_change": 2,
                         "start_after_seconds": 0},
            "beat_samples": beats,
            "cues": sync.make_cues(beats, 0, 4, 2, 0, len(pcm)),
        }
        self.schedule_path = self.root / "reviewed_schedule.json"
        self.save_schedule(self.schedule)

    def save_schedule(self, schedule):
        self.schedule_path.write_text(json.dumps(schedule), encoding="utf-8-sig")

    def test_export_and_inspect_preserve_inputs_and_existing_exports(self):
        before = (self.schedule_path.read_bytes(), self.pcm_path.read_bytes())
        path, info = sync.export_cue(self.schedule_path, self.pcm_path)
        self.assertEqual(path, self.root / "SONG2.CUE")
        self.assertEqual(path.stat().st_size, 76)
        self.assertEqual(info["positions"], [12000, 204000, 396000])
        self.assertEqual(sync.inspect_cue(path, self.pcm_path), info)
        original_export = path.read_bytes()
        with self.assertRaises(FileExistsError):
            sync.export_cue(self.schedule_path, self.pcm_path)
        self.assertEqual(path.read_bytes(), original_export)
        self.assertEqual((self.schedule_path.read_bytes(), self.pcm_path.read_bytes()), before)

    def test_rejects_unreviewed_or_inconsistent_schedule(self):
        for field in ("reviewed", "cues", "schema", "empty"):
            with self.subTest(field=field):
                schedule = copy.deepcopy(self.schedule)
                if field == "reviewed":
                    schedule["reviewed"] = False
                elif field == "cues":
                    schedule["cues"][0]["sample"] += 1
                elif field == "schema":
                    schedule["schema"] = "unknown"
                else:
                    schedule["settings"].update(mode="tracked", anchor_beat=0)
                    schedule["beat_samples"] = []
                    schedule["cues"] = []
                self.save_schedule(schedule)
                with self.assertRaises(ValueError):
                    sync.export_cue(self.schedule_path, self.pcm_path)
                self.assertFalse((self.root / "SONG2.CUE").exists())

    def test_same_length_different_audio_rejected_at_export_and_inspect(self):
        path, _ = sync.export_cue(self.schedule_path, self.pcm_path)
        data = bytearray(self.pcm_path.read_bytes())
        data[100] ^= 1
        changed = self.root / "SONG3.PCM"
        changed.write_bytes(data)
        with self.assertRaisesRegex(ValueError, "SHA256"):
            sync.export_cue(self.schedule_path, changed)
        with self.assertRaisesRegex(ValueError, "SHA256"):
            sync.inspect_cue(path, changed)
        self.assertFalse((self.root / "SONG3.CUE").exists())

    def test_song_pairing_and_lowercase_names(self):
        for name in ("audio.pcm", "SONG0.PCM", "SONG17.PCM", "SONG02.PCM"):
            with self.subTest(name=name):
                other = self.root / name
                other.write_bytes(self.pcm_path.read_bytes())
                with self.assertRaisesRegex(ValueError, "SONG1.PCM"):
                    sync.export_cue(self.schedule_path, other)
        with self.assertRaisesRegex(ValueError, "matching filename"):
            sync.export_cue(self.schedule_path, self.pcm_path, self.root / "SONG3.CUE")
        lowercase = self.root / "song16.pcm"
        lowercase.write_bytes(self.pcm_path.read_bytes())
        path, _ = sync.export_cue(self.schedule_path, lowercase)
        self.assertEqual(path.name, "SONG16.CUE")

    def test_cli_export_inspect_and_clean_error(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(sync.main(["export-cue", str(self.schedule_path),
                                        "--pcm", str(self.pcm_path)]), 0)
            self.assertEqual(sync.main(["inspect-cue", str(self.root / "SONG2.CUE"),
                                        "--pcm", str(self.pcm_path)]), 0)
        self.assertIn("PCM binding verified", output.getvalue())
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(sync.main(["export-cue", str(self.schedule_path),
                                        "--pcm", str(self.pcm_path)]), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
