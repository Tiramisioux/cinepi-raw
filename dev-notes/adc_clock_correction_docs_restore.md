# Restoring the `--audio-clock-ppm` / ADC clock-correction docs

The README documentation for the `--audio-clock-ppm` ADC clock-correction feature
was removed in commit **`019d2de`** because the feature is **not implemented in
committed code** on any branch (the post-take `ffmpeg` pass is `-c:a copy`, with no
resampler), and the RØDE ADC was verified at ~0 ppm so the correction is
unnecessary in practice.

## Bring the docs back

**Option A — revert the commit:**

```sh
git revert 019d2de
```

**Option B — apply the restore patch:**

```sh
git apply dev-notes/adc_clock_correction_docs_restore.patch   # from the repo root
```

Either restores the `--audio-clock-ppm` table row, the `### ADC clock correction`
README section, and the residual "clock correction" mentions in the WAV
timecode-offset section.

## Note

If/when the `--audio-clock-ppm` resampler is actually implemented (option parse +
the `asetrate`/`aresample` ffmpeg branch + the `audio_clock_correction.json`
lookup in Cinemate), restore these docs and update them to match the real code.
