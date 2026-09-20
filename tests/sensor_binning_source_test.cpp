// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the binning-source decision (cinepi/sensor_binning_source.hpp).
//
// Pure / self-contained: no libcamera, no Redis, no ioctl. Build & run:
//   c++ -std=c++17 -Wall -Wextra -I. -O1 tests/sensor_binning_source_test.cpp -o /tmp/sensor_binning_source_test && /tmp/sensor_binning_source_test
// (or via meson: `meson test sensor_binning_source`).
//
// WP-CPR-2 / finding C2: CinePIRecorder::SensorBinning() infers binning from
// active-area-over-delivered-size and gets it wrong on a window crop (a
// 1440x1080 2x2 window rounds to 6, a 1x1 1920x1120 crop rounds to 4 when the
// truth is 1). The sensor driver's own "Mode Binning" control is the fix when
// it is present and sane; these cases pin exactly when it wins.

#include "cinepi/sensor_binning_source.hpp"

#include <cstdio>
#include <optional>

// ── tiny test harness (same shape as dng_output_depth_test.cpp) ────────────
static int g_failures = 0;
static int g_checks   = 0;
#define CHECK(cond, msg)                                                        \
	do {                                                                        \
		++g_checks;                                                             \
		if (!(cond)) {                                                          \
			++g_failures;                                                       \
			std::printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);    \
		}                                                                       \
	} while (0)

static bool same(const SensorBinningDecision &d, double binning, SensorBinningSource source)
{
	if (d.binning == binning && d.source == source)
		return true;
	std::printf("  got binning=%.1f source=%s; expected binning=%.1f source=%s\n",
				d.binning, d.source == SensorBinningSource::kDriver ? "driver" : "ratio",
				binning, source == SensorBinningSource::kDriver ? "driver" : "ratio");
	return false;
}

// A sane driver value must win over the ratio, and it is SQUARED into the
// samples-per-pixel domain ccmp_lut.hpp and SensorBinning() both use.
static void test_valid_driver_value_wins()
{
	std::printf("test_valid_driver_value_wins\n");
	// The exact defect this fixes: a 1440x1080 2x2 window, where the ratio
	// path would say 6 (round(2.67) * round(2)) instead of 4.
	CHECK(same(choose_sensor_binning(std::optional<int>(2), 6.0), 4.0, SensorBinningSource::kDriver),
		  "driver 2 (linear, 2x2) beats a wrong ratio of 6 -> squared to 4");
	// And a 1x1 crop, where the ratio path would say 4 instead of 1.
	CHECK(same(choose_sensor_binning(std::optional<int>(1), 4.0), 1.0, SensorBinningSource::kDriver),
		  "driver 1 (linear, 1x1) beats a wrong ratio of 4 -> squared to 1");
}

// No controls at all (every stock sensor today) falls back to the ratio,
// unchanged.
static void test_absent_falls_back()
{
	std::printf("test_absent_falls_back\n");
	CHECK(same(choose_sensor_binning(std::nullopt, 1.0), 1.0, SensorBinningSource::kRatio),
		  "no driver metadata -> ratio 1, unchanged");
	CHECK(same(choose_sensor_binning(std::nullopt, 4.0), 4.0, SensorBinningSource::kRatio),
		  "no driver metadata -> ratio 4, unchanged");
}

// A nonsensical driver reading (0, negative, or larger than 2 — the only two
// binnings any programmed window produces) must not be trusted.
static void test_nonsensical_rejected()
{
	std::printf("test_nonsensical_rejected\n");
	CHECK(same(choose_sensor_binning(std::optional<int>(0), 4.0), 4.0, SensorBinningSource::kRatio),
		  "driver 0 (unset) rejected -> falls back to ratio");
	CHECK(same(choose_sensor_binning(std::optional<int>(-1), 1.0), 1.0, SensorBinningSource::kRatio),
		  "driver -1 rejected -> falls back to ratio");
	CHECK(same(choose_sensor_binning(std::optional<int>(3), 4.0), 4.0, SensorBinningSource::kRatio),
		  "driver 3 (no such window) rejected -> falls back to ratio");
	CHECK(same(choose_sensor_binning(std::optional<int>(100), 1.0), 1.0, SensorBinningSource::kRatio),
		  "driver 100 rejected -> falls back to ratio");
}

// Every mode shipping today has no driver metadata at all (WP-585-1/WP-283-5
// are not deployed yet), so the result must be byte-identical to what
// SensorBinning() alone already produces.
static void test_shipped_modes_unchanged()
{
	std::printf("test_shipped_modes_unchanged\n");
	struct Mode { const char *what; double ratio_binning; };
	const Mode modes[] = {
		{ "imx585 3840x2160 full-field 1x1",  1.0 },
		{ "imx585 1920x1080 full-field 2x2",  4.0 },
		{ "imx477 1332x990 full-field 1x1",   1.0 },
		{ "imx296 1456x1088 full-field 1x1",  1.0 },
		{ "imx283 5568x3664 full-field 1x1",  1.0 },
	};
	for (const Mode &m : modes)
	{
		const SensorBinningDecision d = choose_sensor_binning(std::nullopt, m.ratio_binning);
		CHECK(d.binning == m.ratio_binning && d.source == SensorBinningSource::kRatio, m.what);
	}
}

// Invariant swept over the whole reachable domain: the result is always
// exactly the ratio, or exactly the square of a driver value in {1, 2}.
static void test_invariant()
{
	std::printf("test_invariant\n");
	bool bad = false;
	for (int driver = -3; driver <= 6; ++driver)
	{
		for (double ratio : { 1.0, 4.0, 6.0, 9.0 })
		{
			const SensorBinningDecision d = choose_sensor_binning(std::optional<int>(driver), ratio);
			const bool driver_sane = (driver == 1 || driver == 2);
			if (driver_sane)
			{
				if (d.source != SensorBinningSource::kDriver ||
					d.binning != static_cast<double>(driver) * static_cast<double>(driver))
					bad = true;
			}
			else
			{
				if (d.source != SensorBinningSource::kRatio || d.binning != ratio)
					bad = true;
			}
		}
	}
	CHECK(!bad, "result is always ratio, or the square of a driver value in {1, 2}");
}

int main()
{
	std::printf("=== sensor_binning_source unit tests ===\n");
	test_valid_driver_value_wins();
	test_absent_falls_back();
	test_nonsensical_rejected();
	test_shipped_modes_unchanged();
	test_invariant();

	std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
	if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
	return g_failures == 0 ? 0 : 1;
}
