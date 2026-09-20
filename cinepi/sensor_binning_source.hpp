/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * sensor_binning_source.hpp - which binning value the encoder should trust:
 * the sensor driver's own read-only geometry controls, or the existing
 * active-area/delivered-size ratio (CinePIRecorder::SensorBinning()).
 *
 * WP-CPR-2 (finding C2). Pulled out of cinepi_raw.cpp so the decision can be
 * tested without libcamera or a live camera: see
 * tests/sensor_binning_source_test.cpp. The ioctl probe that produces the
 * driver's candidate value cannot be tested here at all (Linux-only,
 * core/driver_mode_metadata.hpp) — this is the part of that change that CAN
 * be pinned.
 *
 * WHY THIS EXISTS
 * CinePIRecorder::SensorBinning() infers binning by dividing the sensor's
 * active area by the delivered stream size and rounding per axis. That is
 * exactly right for a full-field mode and wrong for a window crop: a
 * 1440x1080 2x2 window against a 3840x2160 active area rounds to
 * round(2.67) * round(2) = 6, and a 1x1 1920x1120 crop rounds to 4 when the
 * truth is 1. cinepi/ccmp_lut.hpp's kT1Effective anchors exist for exactly
 * two values, 1 and 4, so a wrong ratio answer silently selects no
 * ClearHDR-12 decompand table at all (ccmp_params_for_binning refuses it)
 * and the preview renders magenta.
 *
 * The sensor driver knows its own window and does not need to guess: WP-585-1
 * (imx585) and WP-283-5 (imx283) expose it as five read-only controls,
 * "Mode Binning" among them, found by name — see driver_mode_metadata.hpp.
 * This header is the last step: given that candidate (if any) and the
 * existing ratio answer, decide which one the encoder is told.
 *
 * THE DOMAIN MISMATCH TO NOT LOSE
 * The driver's "Mode Binning" is a LINEAR factor (1 or 2 — see
 * driver_mode_metadata.hpp). ccmp_lut.hpp and
 * CinePIRecorder::SensorBinning() both work in the SQUARED "samples per
 * output pixel" domain (1 or 4). Squaring the driver's linear value happens
 * HERE, once, so every caller of choose_sensor_binning() gets an answer
 * that is already in the domain setSensorBinning() expects.
 */

#ifndef CINEPI_SENSOR_BINNING_SOURCE_HPP
#define CINEPI_SENSOR_BINNING_SOURCE_HPP

#include <optional>

enum class SensorBinningSource
{
	kRatio,   /* CinePIRecorder::SensorBinning(): active area / delivered size */
	kDriver,  /* the sensor driver's own "Mode Binning" control */
};

struct SensorBinningDecision
{
	double binning;
	SensorBinningSource source;
};

/*
 * driver_binning - the driver's linear "Mode Binning" value (1 or 2), or
 *                  std::nullopt when the sensor exposes no such control
 *                  (every stock sensor today) or the probe failed.
 * ratio_binning  - CinePIRecorder::SensorBinning()'s own answer, in the
 *                  squared domain (1, 4, ...). Always available; this is
 *                  the fallback and the value used unconditionally today.
 *
 * A driver value is trusted only when it is 1 or 2, matching the only two
 * binnings any programmed window on these sensors actually produces.
 * Anything else — 0 (unset), negative, or larger than 2 — is a bogus
 * reading and is rejected in favour of the ratio: picking the wrong
 * binning here silently selects the wrong CCMP curve (or none at all),
 * which is a worse failure than falling back to the ratio's own
 * long-standing behaviour.
 */
inline SensorBinningDecision choose_sensor_binning(std::optional<int> driver_binning,
													double ratio_binning)
{
	if (driver_binning && *driver_binning >= 1 && *driver_binning <= 2)
	{
		const double linear = static_cast<double>(*driver_binning);
		return { linear * linear, SensorBinningSource::kDriver };
	}

	return { ratio_binning, SensorBinningSource::kRatio };
}

#endif // CINEPI_SENSOR_BINNING_SOURCE_HPP
