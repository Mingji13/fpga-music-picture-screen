"""Optional browser integration test; requires playwright and installed Chrome/Edge."""
import argparse
import json
import tempfile
import wave
from pathlib import Path

import numpy as np
from playwright.sync_api import sync_playwright
import music_sync as sync


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--browser', type=Path, default=Path(r'C:\Program Files\Google\Chrome\Application\chrome.exe'))
    parser.add_argument('--screenshot', type=Path)
    args = parser.parse_args()
    bundle = args.bundle.resolve()
    with tempfile.TemporaryDirectory() as temp, sync_playwright() as pw:
        root = Path(temp)
        browser = pw.chromium.launch(executable_path=str(args.browser), headless=True)
        page = browser.new_page(viewport={'width': 1280, 'height': 1100}, accept_downloads=True)
        errors = []
        page.on('pageerror', lambda error: errors.append(str(error)))
        page.goto((bundle / 'review.html').as_uri())
        page.wait_for_function("document.getElementById('summary').textContent.includes('个候选拍点')")
        if args.screenshot:
            if args.screenshot.exists():
                raise FileExistsError(args.screenshot)
            page.screenshot(path=str(args.screenshot), full_page=True)
        page.locator('#play').click()
        page.wait_for_function('running && now() > 0.15')
        page.locator('#play').click()
        assert page.evaluate('running') is False
        assert page.evaluate('now()') > 0.15
        original_anchor = page.evaluate('schedule.settings.anchor_beat')
        page.locator('#nextBeat').click()
        assert page.evaluate('schedule.settings.anchor_beat') == original_anchor + 1
        page.locator('#reviewed').check()
        page.locator('#bars').select_option('1')
        assert not page.locator('#reviewed').is_checked()
        page.locator('#reviewed').check()
        with page.expect_download() as result:
            page.locator('#saveJson').click()
        result.value.save_as(root / 'tracked.json')
        tracked = json.loads((root / 'tracked.json').read_text(encoding='utf-8'))
        pcm = sync.validate_schedule(tracked, bundle / 'audio.pcm')
        assert tracked['reviewed'] is True
        page.locator('#mode').select_option('grid')
        page.locator('#bpm').fill('120')
        page.locator('#anchorTime').fill('1.3')
        page.locator('#bars').select_option('2')
        page.locator('#startAfter').fill('6')
        page.locator('#startAfter').press('Tab')
        assert page.locator('#error').text_content() == ''
        with page.expect_download() as result:
            page.locator('#saveJson').click()
        result.value.save_as(root / 'grid.json')
        grid = json.loads((root / 'grid.json').read_text(encoding='utf-8'))
        sync.validate_schedule(grid, bundle / 'audio.pcm')
        assert grid['cues'][0]['sample'] == 446400
        with page.expect_download() as result:
            page.locator('#saveWav').click()
        result.value.save_as(root / 'preview.wav')
        with wave.open(str(root / 'preview.wav'), 'rb') as sound:
            assert sound.getnframes() == len(pcm)
            assert sound.getframerate() == 48000
            downloaded = np.frombuffer(sound.readframes(len(pcm)), dtype='<i2')
        s = grid['settings']
        reference = sync.preview_samples(pcm, grid['beat_samples'], grid['cues'], s['anchor_beat'], s['beats_per_bar'], 'cuts')
        assert np.max(np.abs(downloaded.astype(np.int32) - reference.astype(np.int32))) <= 1
        assert sync.main(['render', str(root / 'grid.json'), '--pcm', str(bundle / 'audio.pcm'), '--output', str(root / 'rendered')]) == 0
        page.locator('#bpm').fill('0')
        page.locator('#bpm').press('Tab')
        assert page.locator('#saveJson').is_disabled()
        assert page.locator('#error').text_content()
        page.locator('#reset').click()
        assert not page.locator('#saveJson').is_disabled()
        assert not errors, errors
        browser.close()
    print('PASS: local HTML playback, phase/bar edits, grid calibration, reviewed reset, JSON/PCM binding, exact WAV export, render roundtrip, invalid-input handling; no browser errors.')


if __name__ == '__main__':
    main()
