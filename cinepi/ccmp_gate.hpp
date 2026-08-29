/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_gate.hpp - whether dng_encoder.cpp's CCMP12 table should be
 * considered at all for the current sensor mode.
 *
 * Pure boolean logic, pulled out of DngEncoder::setup_encoder() so the
 * mismatch-refusal rule can be tested without a live Camera/
 * StreamConfiguration -- see tests/ccmp_gate_test.cpp.
 *
 * Scope is exactly ClearHDR ON *and* a 12-bit sensor mode -- dng_encoder.cpp's
 * own long-standing rule, restated here rather than duplicated: 16-bit
 * ClearHDR is delivered linear with no compander in the path, and a 12-bit
 * SDR mode never companded either, so gating on bit depth alone would
 * decompand data that was never companded.
 *
 * `mode_trusted` is the round-2 addition: a 12-bit result is only believed
 * when the requested mode's dimensions actually matched what the camera
 * configured (see cinepi_raw.cpp's WARN and CinePIRecorder::SensorBinning()'s
 * comment). When they didn't, sensor_mode_bit_depth_ is a snapshot of the
 * REQUEST, not of what the camera actually streamed -- and on the observed
 * hardware failure (a 12-bit request landing on the real 16-bit sensor mode,
 * journal 2026-08-27 19:52: raw stream R16 with the hdr flag set) it still
 * reads 12 despite the stream being genuinely linear 16-bit. The R16
 * container is ambiguous between true 16-bit and mono 12-in-16, and dims
 * alone cannot disambiguate it either (both land on 3856x2180 on this
 * driver) -- reading the sensor subdev's actual media-bus code would settle
 * it and is a later, separate fix, out of scope here.
 *
 * Until then this fails toward NO table: a companded take missing its table
 * renders magenta but is recoverable in post (the LinearizationTable is
 * metadata, the pixels are untouched); a linear take mislabelled with a
 * decompand table is silently wrong in every raw app, which is the worse
 * failure. See setup_encoder() for the loud warn that covers the refused
 * case.
 */

#ifndef CINEPI_CCMP_GATE_HPP
#define CINEPI_CCMP_GATE_HPP

#include <string>

inline bool ccmp_gate_should_consider(const std::string &hdr, unsigned sensor_mode_bit_depth,
                                       bool sensor_mode_trusted)
{
    return (hdr == "sensor" || hdr == "auto") && sensor_mode_bit_depth == 12 && sensor_mode_trusted;
}

#endif /* CINEPI_CCMP_GATE_HPP */
