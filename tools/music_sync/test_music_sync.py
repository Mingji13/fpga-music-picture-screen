"""Checks for timing integrity, malformed inputs, and audible click alignment."""
import contextlib
import io
import json
import tempfile
import unittest
import wave
from decimal import Decimal, ROUND_HALF_UP
from pathlib import Path

import numpy as np
import music_sync as sync


class CueTests(unittest.TestCase):
    def test_known_musical_example_and_no_accumulated_rounding(self):
        beats = sync.grid_samples(120, 1.3, 48000 * 20)
        cues = sync.make_cues(beats, 0, 4, 2, 0, 48000 * 20)
        self.assertEqual([x['sample'] for x in cues], [62400, 254400, 446400, 638400, 830400])
        bpm = 123.456
        beats = sync.grid_samples(bpm, 0.123, 48000 * 600)
        for i in (0, 1, 100, len(beats) - 1):
            expected = (Decimal('0.123') * 48000 + i * Decimal(48000 * 60) / Decimal('123.456')).quantize(Decimal(1), rounding=ROUND_HALF_UP)
            self.assertEqual(beats[i], int(expected))

    def test_cues_preserve_phase_when_intro_is_skipped(self):
        beats = sync.grid_samples(120, 1.3, 48000 * 20)
        cues = sync.make_cues(beats, 0, 4, 2, 6, 48000 * 20)
        self.assertEqual([x['sample'] for x in cues], [446400, 638400, 830400])
        self.assertEqual([x['bar_index'] for x in cues], [5, 7, 9])
        with self.assertRaises(ValueError):
            sync.make_cues([0, 100, 100], 0, 4, 2, 0, 48000)
        with self.assertRaises(ValueError):
            sync.grid_samples(float('nan'), 0, 48000)

    def test_preview_does_not_move_music_or_click(self):
        pcm = np.zeros(48000 * 3, dtype='<i2')
        pcm[12000] = 10000
        beats = [48000, 72000, 96000, 120000]
        cues = sync.make_cues(beats, 0, 4, 2, 0, len(pcm))
        mixed = sync.preview_samples(pcm, beats, cues, 0, 4, 'cuts')
        self.assertEqual(len(mixed), len(pcm))
        self.assertEqual(mixed[12000], 6500)
        self.assertEqual(np.count_nonzero(mixed[12001:48000]), 0)
        self.assertNotEqual(mixed[48001], 0)
        self.assertEqual(np.count_nonzero(mixed[50000:]), 0)

    def test_synthetic_onsets_and_silence(self):
        pcm = np.zeros(48000 * 32, dtype=np.float32)
        positions = np.arange(1, 31, .5)
        for i, second in enumerate(positions):
            t = np.arange(4800) / 48000
            hit = np.sin(2 * np.pi * 110 * t) * np.exp(-t * 35)
            at = int(second * 48000)
            pcm[at:at+len(hit)] += hit * (.9 if i % 4 == 0 else .4)
        samples = (pcm * 30000).astype('<i2')
        beats, anchor, report = sync.analyze_beats(samples, 4)
        self.assertGreater(len(beats), 45)
        errors = [min(abs(sample / 48000 - positions)) for sample in beats]
        self.assertLess(float(np.quantile(errors, .95)), .06)
        self.assertLess(abs(report['estimated_bpm'] - 120), 3)
        self.assertLess(anchor, 4)
        quiet, _, _ = sync.analyze_beats(np.zeros(48000 * 4, dtype='<i2'), 4)
        self.assertEqual(quiet, [])

    def test_raw_pcm_errors_and_schedule_audio_binding(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            bad = root / 'bad.pcm'
            bad.write_bytes(b'123')
            with self.assertRaises(ValueError):
                sync.read_pcm(bad)
            original = root / 'source.wav'
            pcm = np.arange(48000 * 8, dtype=np.int32).astype('<i2')
            sync.write_wav(original, pcm)
            original_hash = sync.sha256(original)
            output = root / 'result'
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(sync.main(['analyze', str(original), '--grid', '--bpm', '120',
                                            '--first-downbeat', '1.3', '--output', str(output)]), 0)
            self.assertEqual(original_hash, sync.sha256(original))
            self.assertEqual((output / 'audio.pcm').read_bytes(), pcm.tobytes())
            schedule = json.loads((output / 'schedule.json').read_text(encoding='utf-8'))
            np.testing.assert_array_equal(sync.validate_schedule(schedule, output / 'audio.pcm'), pcm)
            with wave.open(str(output / 'cuts_preview.wav'), 'rb') as sound:
                self.assertEqual((sound.getframerate(), sound.getnchannels(), sound.getsampwidth(), sound.getnframes()), (48000, 1, 2, len(pcm)))
            changed = root / 'changed.pcm'
            damaged = pcm.copy(); damaged[123] += 1; damaged.tofile(changed)
            with self.assertRaisesRegex(ValueError, 'SHA256'):
                sync.validate_schedule(schedule, changed)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(sync.main(['analyze', str(original), '--output', str(output)]), 1)
            schedule['cues'][0]['sample'] += 1
            with self.assertRaisesRegex(ValueError, 'Cue positions'):
                sync.validate_schedule(schedule, output / 'audio.pcm')


if __name__ == '__main__':
    unittest.main(verbosity=2)
