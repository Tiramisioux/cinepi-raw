/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * dng_output_depth.hpp - which code depth a DNG's raw strip is packed at,
 * for the LINEAR path.
 *
 * Pure boolean/arithmetic logic, pulled out of DngEncoder::setup_encoder()
 * so the rule can be tested without a live Camera/StreamConfiguration --
 * same reasoning and same shape as ccmp_gate.hpp, which this deliberately
 * mirrors. See tests/dng_output_depth_test.cpp.
 *
 * "Linear path" is the scope, and it is the whole subtlety: CineMate Log and
 * the CCMP decompand both override the depth AFTER this decision runs, and
 * the log path clears both flags outright because log_lut_ owns the row
 * conversion. So this answers "what would the linear packer do", not "what
 * depth does the file end up at". setup_encoder() composes the two.
 *
 * WHY A DECISION IS NEEDED AT ALL
 * On a Pi 5, libcamera's PiSP pipeline handler cannot emit a raw stream in
 * anything but an unpacked 16-bit container -- "We cannot output CSI2 packed
 * or non 16-bit output from the frontend" (pipeline/rpi/pisp/pisp.cpp). So
 * the Bayer format's own bit count stops describing the data: every SDR mode,
 * 10-bit and 12-bit alike, arrives as SRGGB16. The sensor mode's depth is the
 * only thing that still says how many of those bits are real, and the
 * significant ones are MSB-aligned, so the container has to be shifted down
 * by (container - sensor) before packing.
 *
 * On a Pi 4 / VC4 none of this applies: the rows arrive at their native depth
 * (SBGGR10/12 and their _CSI2P forms), already right-justified or CSI2-packed,
 * and must be left alone. That is what the `container == 16` requirement
 * below is for -- it is not a tautology.
 *
 * THE ASYMMETRY THAT SHAPES THE RULE
 * Getting this wrong is not equally bad in both directions. Packing DOWN
 * further than the data justifies destroys real bits irrecoverably; packing
 * down less than it could just stores known-zero padding, which costs card
 * space and nothing else. Every guard here therefore fails toward the
 * larger file. An unrecognised depth packs at 12 exactly as it always has,
 * rather than being guessed at.
 */

#ifndef CINEPI_DNG_OUTPUT_DEPTH_HPP
#define CINEPI_DNG_OUTPUT_DEPTH_HPP

struct DngOutputDepth
{
    unsigned bits;      /* code depth the raw strip is packed at */
    unsigned shift;     /* right-shift applied to each container sample first */
    bool     pack12;    /* take dng_save()'s 16->12 branch */
    bool     pack10;    /* take dng_save()'s 16->10 branch */

    unsigned white() const { return (1u << bits) - 1u; }
};

/* Resolve the linear output depth.
 *
 *   container        - the Bayer format's own bit count (bf.bits): 16 on every
 *                      Pi 5 raw stream, the native depth on VC4.
 *   sensor_bit_depth - the snapshot of the mode the camera actually configured.
 *   trusted          - whether that snapshot describes this stream at all (a
 *                      dimensions match; see cinepi_raw.cpp and ccmp_gate.hpp).
 *   packed / compressed
 *                    - the row layout: CSI2-packed, or PiSP COMP1. Both are
 *                      dispatched on earlier in dng_save(). `packed` is not
 *                      eligible for a narrowing repack; `compressed` is, and
 *                      takes the 10-bit one -- see the note at that branch.
 *
 * Returns the container's own depth unchanged whenever no narrowing applies,
 * so the caller can assign unconditionally and never leave a stale flag set
 * across takes.
 */
inline DngOutputDepth resolve_dng_output_depth(unsigned container,
                                               unsigned sensor_bit_depth,
                                               bool     trusted,
                                               bool     packed,
                                               bool     compressed)
{
    /* A VC4 stream (container != 16) carries its rows at their native depth
     * already, and an untrusted snapshot says nothing about this stream at
     * all. Either way, keep the container's own depth. */
    if (!trusted || container != 16 || sensor_bit_depth == 16)
        return { container, 0u, false, false };

    /* NOTE the asymmetry between the two narrowings, which is load-bearing
     * rather than an oversight: `packed`/`compressed` gate ONLY the 10-bit
     * case. The 12-bit narrowing must stay true for a COMP1 row, because
     * dng_save()'s compressed branch reads that same flag to choose between
     * unpack_pisp_comp1_row_to_packed12() and a 2 B/px verbatim write --
     * clearing it there would silently turn every COMP1 take into a 16-bit
     * file.
     *
     * `compressed` USED TO gate the 10-bit case too, on the grounds that
     * COMP1's dequantisation "does not reconstruct multiples of 64 in three of
     * its four quantisation modes". That count was measured over LUT indices
     * and, re-measured on 2026-09-15 over the decoder's actual output, does not
     * survive: in qmodes 1, 2 and 3 EVERY off-grid value is the 65535
     * saturation clamp -- white, which a 10-bit and a 12-bit file each record
     * correctly as their own white level -- and not reconstruction detail at
     * all. qmode 0 is the only mode with real sub-64 levels.
     *
     * And even qmode 0's cannot be signal. What enters the compressor on a
     * 10-bit mode is the sensor code MSB-aligned, i.e. an exact multiple of 64
     * (the frontend's BLA block is a no-op for every shipped tuning -- they all
     * give a single scalar black_level, so it computes in - BL + BL), so a
     * decoded value off the 64-grid is always codec error, never a level the
     * sensor could have sent. Rounding to 10 bits recovers the same original
     * code the 12-bit sample implies, for every value the decoder can emit,
     * with zero disagreements. The full measurement is written out at
     * unpack_pisp_comp1_row_to_packed10() in dng_pack.hpp and pinned by
     * tests/dng_pack_test.cpp.
     *
     * So COMP1 is eligible, and a 10-bit imx519 mode stops paying 1.5 B/px for
     * 1.25 B/px of information. `packed` still gates: a CSI2-packed 16-bit row
     * is a shape dng_save() has no unpacker for. */
    if (sensor_bit_depth == 10 && !packed)
        return { 10u, container - 10u, false, true };

    /* 12 is the fallback for every remaining depth -- 0 (unset), 8, 14, or
     * anything a stray redis write leaves behind -- not just for 12 itself.
     * That is the long-standing behaviour and the non-destructive direction;
     * deriving a packer from an unrecognised depth would emit an untested
     * file. */
    return { 12u, container - 12u, true, false };
}

#endif /* CINEPI_DNG_OUTPUT_DEPTH_HPP */
