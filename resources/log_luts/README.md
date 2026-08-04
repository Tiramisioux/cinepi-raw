# CineMate Log curve specs

Curve specs for `--log-encode`, which log-compands a linear sensor signal to 10 or
12 bits in the recording DNG writer and stores a DNG `LinearizationTable` (tag
0xC618) so the file decodes back to linear in any DNG app.

## Files

| file | what |
|---|---|
| `gen_cinemate_log.py` | generator — the single source of truth for the curve |
| `cinemate_log_16to12.json` | imx585 16-bit ClearHDR → 12-bit log (−25%, 197 codes/stop) |
| `cinemate_log_16to10.json` | imx585 16-bit ClearHDR → 10-bit log (−37.5%, 48 codes/stop) |
| `cinemate_log_12to10.json` | 12-bit linear → 10-bit log (−16.6%, 76 codes/stop) |

Each spec holds `params{}` (µ, black/white level, source/target bits, footroom) plus
the full `linearization_table` — the exact code→linear values written to tag 0xC618.
cinepi-raw rebuilds the dense forward/inverse tables from `params` at load; the
embedded table is for direct DNG-tag writing and for validating the C++ against the
generator entry-for-entry.

## The curve — CineMate Log v1.1

µ-law above black, plus a linear footroom segment below it:

```
F = footroom codes (32), TOP = CMAX - F, CMAX = 2^target - 1
encode L>=BL: C = F + nearbyint(TOP*log1p(mu*x)/log1p(mu)), x = clamp((L-BL)/(WL-BL),0,1)
encode L< BL: C = clamp(floor((L-(BL-foot))/foot*F), 0, F-1)
decode C>=F : L = BL + (pow(1+mu,(C-F)/TOP)-1)/mu*(WL-BL)
decode C< F : L = (BL-foot) + (C+0.5)/F*foot
```

Two details are load-bearing and easy to get wrong:

- **`floor` on encode, bin *centre* on decode.** Using `round` + the bin edge biases
  the reconstructed black point (measured: +1.5 LSB lift, noise std −26%).
- **`nearbyint`, not `round`.** The generator uses numpy's banker's rounding
  (ties-to-even). C++ `std::round` is ties-away-from-zero and will not reproduce the
  table. `std::nearbyint` under the default FE_TONEAREST matches.

`BlackLevel`/`WhiteLevel` in the DNG stay in the **linear (table-output) domain** —
they are *not* rescaled to the stored code range. Verified: LibRaw auto-applies the
table and returns the linear signal with black/white intact.

## Footroom is per-sensor data

Like black/white level, the footroom extent has no formula that fits every sensor.
Rule of thumb: `foot ≈ F × read-noise σ` (in source LSB), capped at the black level.

| sensor | BL | σ | foot | result |
|---|---|---|---|---|
| imx585 16-bit ClearHDR | 3200 | ~12.8 | 487 | black lift −0.34 LSB, noise std ×1.035 |
| 12-bit linear | 200 | ~10.8 | 200 | black lift +0.02 LSB, noise std ×1.003 |

Measure a new sensor's sub-black tail before picking its value.

## Regenerating

```
python3 gen_cinemate_log.py                       # regenerate all three specs
python3 gen_cinemate_log.py --src 16 --tgt 12 --bl 3200 --wl 65535 [--foot-lsb N]
```

Needs numpy. Prints codes/stop, round-trip error in milli-stops, and a monotonicity
check for the emitted table.
