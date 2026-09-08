#!/usr/bin/env python3
"""Generate + validate a CineMate Log curve (linear -> N-bit log) + DNG table.

CineMate Log v1.1 = mu-law companding of the signal above black, plus a small
Cineon-style FOOTROOM segment below black. Reversible, monotonic, generic across
source and target bit depth:

  codes [0, F)     linear ramp over [BL-foot, BL)   -- sub-black noise floor
  codes [F, CMAX]  mu-law over [BL, WL]             -- the picture

  encode (above black):  C = F + round(TOP * ln(1+mu*x)/ln(1+mu)), x=(L-BL)/(WL-BL)
  decode (above black):  L = BL + ((1+mu)^((C-F)/TOP)-1)/mu*(WL-BL)
  decode (footroom):     L = (BL-foot) + (C+0.5)/F*foot
  where CMAX = 2^target-1 and TOP = CMAX-F.

Why footroom (v1.1): v1 clamped everything below BlackLevel to code 0. Measured on
a real imx585 ClearHDR frame, 4.1% of pixels sit below BL (sensor noise), and
crushing them half-wave-rectifies the noise floor: black point lifts +1.50 LSB and
noise std drops 26% (a "plasticky" deepest shadow, and sub-black black-level
calibration becomes impossible). F=32 footroom codes -- 0.8% of a 12-bit range --
takes the lift to -0.09 LSB and restores the noise std, costing 1 code/stop.

Both directions stay pure tables, so the C++ engine is identical either way; only
this generator knows the curve shape.

The runtime flag chooses the TARGET bit depth (10 or 12). SOURCE bit depth / BL /
WL come from the sensor+mode. mu defaults to SPAN/toe_lsb (a linear toe of ~toe_lsb
source-LSB), which auto-scales with the source DR; override if needed.

Usage: gen_cinemate_log.py --src 16 --tgt 10 --bl 3200 --wl 65535 [--mu M] [--toe 6]
                           [--foot-codes 32] [--foot-lsb N]
       (no args -> regenerates the three reference specs + prints the matrix)
"""
import sys, json, math
import numpy as np

FOOT_CODES = 32          # validated sweet spot for both 10- and 12-bit targets


def default_foot_lsb(BL, WL):
    """Footroom extent below black, in linear LSB.

    PER-SENSOR DATA, like BL/WL — there is no formula that fits every sensor.
    Rule of thumb: foot_lsb ~= F * sigma (sensor read-noise std in source LSB),
    capped at BL (below 0 is not physical). That makes the footroom step ~= sigma,
    which adds ~4% to the noise std -- negligible -- while covering the whole tail.

    Validated on real frames (F=32):
      imx585 16-bit ClearHDR  BL=3200  sigma~12.8  -> 487  (band lift -0.34, std x1.035)
      12-bit linear           BL= 200  sigma~10.8  -> 200  (band lift +0.02, std x1.003)

    The SPAN/128 default below happens to land on 487 for the 16-bit ClearHDR case;
    for low-black sensors it under-covers, so it is capped at BL and should be set
    explicitly (--foot-lsb) for any new sensor after measuring the sub-black tail.
    """
    return min(BL, max(8, int(round((WL - BL) / 128.0)))) if BL else 0


def build(BL, WL, target_bits, mu, foot_codes=FOOT_CODES, foot_lsb=None):
    """Return (encode_fn, decode_fn, CMAX, SPAN, F, foot_lsb)."""
    CMAX = (1 << target_bits) - 1
    SPAN = WL - BL
    F = int(foot_codes)
    TOP = CMAX - F
    foot = default_foot_lsb(BL, WL) if foot_lsb is None else int(foot_lsb)

    def enc(L):
        L = np.asarray(L, float)
        x = np.clip((L - BL) / SPAN, 0.0, 1.0)
        C = np.round(np.log1p(mu * x) / math.log1p(mu) * TOP) + F
        if F:
            sub = L < BL
            if np.any(sub):
                # floor (not round) so the F sub-black bins are uniform: code c
                # covers [lo+c*step, lo+(c+1)*step), step = foot/F. With round()
                # the top bin would be 1.5 steps wide and bias the black point.
                sub_c = np.clip(np.floor((L - (BL - foot)) / foot * F), 0, F - 1)
                C = np.where(sub, sub_c, C)
        return np.clip(C, 0, CMAX).astype(int)

    def dec(C):
        C = np.asarray(C, float)
        up = (np.power(1.0 + mu, np.clip(C - F, 0, None) / TOP) - 1.0) / mu
        L = BL + up * SPAN
        if F:
            # bin CENTRE of the floor() bins above: lo + (c+0.5)*step. Using the
            # bin edge instead biases the reconstructed black point by step/2.
            L = np.where(C < F, (BL - foot) + (C + 0.5) / F * foot, L)
        return np.clip(np.round(L), 0, WL).astype(int)

    return enc, dec, CMAX, SPAN, F, foot


def validate(BL, WL, target_bits, mu, foot_codes=FOOT_CODES, foot_lsb=None):
    enc, dec, CMAX, SPAN, F, foot = build(BL, WL, target_bits, mu, foot_codes, foot_lsb)
    Ss = 2.0 ** np.arange(3, math.log2(SPAN) + 1)          # 8 LSB (near noise) -> white
    cps = [int(enc(BL + s1) - enc(BL + s0)) for s0, s1 in zip(Ss[:-1], Ss[1:])]
    err = max(abs(math.log2(max(dec(enc(BL + S)) - BL, 1e-6) / S)) * 1000
              for S in (8, 32, 256, 2048, 16384) if S < SPAN)
    inv = dec(np.arange(CMAX + 1))
    monotonic = bool(np.all(np.diff(inv) >= 0))
    verdict = "OK" if min(cps) >= 64 else ("MARGINAL" if min(cps) >= 40 else "BANDS")
    return dict(min_codes_per_stop=min(cps), codes_per_stop=cps,
                worst_err_milli_stops=round(err, 1), verdict=verdict,
                footroom_codes=F, footroom_lsb=foot, table_monotonic=monotonic)


def spec(BL, WL, source_bits, target_bits, mu, foot_codes=FOOT_CODES, foot_lsb=None):
    enc, dec, CMAX, SPAN, F, foot = build(BL, WL, target_bits, mu, foot_codes, foot_lsb)
    inv = dec(np.arange(CMAX + 1)).astype(int).tolist()   # code -> linear = DNG tag 0xC618
    return dict(name="CineMate Log v1.1 (mu-law + footroom)", version=2,
                params=dict(mu=mu, black_level=BL, white_level=WL,
                            source_bits=source_bits, target_bits=target_bits,
                            code_max=CMAX, footroom_codes=F, footroom_lsb=foot),
                encoding="C = F + round(TOP*ln(1+mu*x)/ln(1+mu)), x=(L-BL)/(WL-BL), TOP=CMAX-F; "
                         "L<BL: C = round((L-(BL-foot))/foot*F)",
                decoding="C>=F: L = BL + ((1+mu)^((C-F)/TOP)-1)/mu*(WL-BL); "
                         "C<F: L = (BL-foot) + (C+0.5)/F*foot   [= LinearizationTable]",
                note="DNG: strip = target_bits log codes; tag 0xC618 LinearizationTable = this "
                     "table (code->linear); BlackLevel=BL, WhiteLevel=WL in the LINEAR "
                     "(table-output) domain -- NOT scaled to the stored code range. Runtime flag "
                     "picks target_bits; BL/WL/source from mode.",
                linearization_table_len=len(inv), linearization_table=inv,
                validation=validate(BL, WL, target_bits, mu, foot_codes, foot_lsb))


def main(argv):
    if not argv:
        print("case                    mu     min c/stop  err(mstop)  foot  verdict")
        # foot_lsb is measured per sensor (see default_foot_lsb); 200 for the
        # 12-bit case because its sub-black tail reaches all the way to 0.
        for name, BL, WL, src, tgt, mu, foot in (
                ("16->12 imx585 HDR", 3200, 65535, 16, 12, 10000, None),
                ("16->10 imx585 HDR", 3200, 65535, 16, 10, 10000, None),
                ("12->10 12-bit",      200,  4095, 12, 10, 1500, 200)):
            v = validate(BL, WL, tgt, mu, FOOT_CODES, foot)
            print(f"{name:22s} {mu:6d}  {v['min_codes_per_stop']:5d}      "
                  f"{v['worst_err_milli_stops']:6.1f}   {v['footroom_codes']:3d}/"
                  f"{v['footroom_lsb']:<4d} {v['verdict']}")
            fn = f"cinemate_log_{src}to{tgt}.json"
            json.dump(spec(BL, WL, src, tgt, mu, FOOT_CODES, foot), open(fn, "w"))
            print(f"    -> {fn}  codes/stop {v['codes_per_stop']}  monotonic={v['table_monotonic']}")
        return
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=int, default=16)
    ap.add_argument("--tgt", type=int, default=12)
    ap.add_argument("--bl", type=int, default=3200)
    ap.add_argument("--wl", type=int, default=65535)
    ap.add_argument("--toe", type=float, default=6.0, help="linear toe size in source LSB")
    ap.add_argument("--mu", type=float)
    ap.add_argument("--foot-codes", type=int, default=FOOT_CODES,
                    help="codes reserved below black (0 disables footroom = v1 behaviour)")
    ap.add_argument("--foot-lsb", type=int, help="footroom extent below black in linear LSB")
    a = ap.parse_args(argv)
    mu = a.mu if a.mu else max(64.0, (a.wl - a.bl) / a.toe)   # auto: mu = SPAN / toe_lsb
    s = spec(a.bl, a.wl, a.src, a.tgt, mu, a.foot_codes, a.foot_lsb)
    out = f"cinemate_log_{a.src}to{a.tgt}.json"
    json.dump(s, open(out, "w"))
    print(f"mu={mu:.0f}  {json.dumps(s['validation'])}\n  wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1:])
