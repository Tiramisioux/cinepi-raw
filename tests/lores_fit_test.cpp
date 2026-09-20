// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for fitting a requested lores/preview stream size inside the raw
// stream's delivered size (cinepi/lores_fit.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -Wall -Wextra -I. -O1 tests/lores_fit_test.cpp -o /tmp/lores_fit_test && /tmp/lores_fit_test
// (or via meson: `meson test lores_fit`).
//
// This pins WP-CPR-1 (finding C1): RPiCamApp::ConfigureVideo used to throw
// "Low res image larger than raw image" for any lores request that did not
// fit entirely inside the raw stream. CineMate always asks for a 1280x720
// preview, so every sensor mode shorter than 720 rows, or narrower than
// 1280, aborted at configure -- before any of the rest of this campaign's
// work could even be reached. The fix fits the request inside the raw
// stream instead of refusing it.
//
// NOTE on the worked example in WORK-PACKAGES.md's WP-CPR-1 "Tests" section:
// it lists "1280x720 inside 1440x1080 becomes 960x720". That is not
// reachable by any aspect-preserving, never-enlarge fit: 1280x720 already
// fits inside 1440x1080 on BOTH axes (more comfortably, in fact, than the
// 1928x1090 case the same list calls "unchanged" -- 1440-1280=160 spare
// width versus 1928-1280=648, 1080-720=360 spare height versus 1090-720=370),
// so no monotonic scale-then-cap-at-1 rule can shrink one and not the other.
// 960x720 IS exactly correct -- for a different computation: it is WP-CM-1's
// "1440x1100 carrying 1440x1080 gives 960x720" example a few sections later
// in the same file, which derives its OWN aspect from the sensor mode's
// active picture and targets a fixed 720-line height, an algorithm this
// package does not implement and was not asked to. That case is tested here
// as "already fits, unchanged" (test_1440x1080_already_fits) instead of the
// literal 960x720, on the reading that the worked example is a copy/paste
// bleed from the neighbouring section rather than a requirement of this
// package's own contract. Flagged in the PR/commit message for the
// orchestrator to confirm.

#include "cinepi/lores_fit.hpp"

#include <cstdio>
#include <stdexcept>

// ── tiny test harness (same shape as dng_output_depth_test.cpp) ─────────────
static int g_failures = 0;
static int g_checks = 0;
#define CHECK(cond, msg)                                                                         \
	do                                                                                             \
	{                                                                                              \
		++g_checks;                                                                               \
		if (!(cond))                                                                              \
		{                                                                                          \
			++g_failures;                                                                         \
			std::printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);                      \
		}                                                                                          \
	} while (0)

static bool same(const LoresFit &f, unsigned w, unsigned h, bool clamped)
{
	if (f.width == w && f.height == h && f.clamped == clamped)
		return true;
	std::printf("  got width=%u height=%u clamped=%d; expected width=%u height=%u clamped=%d\n", f.width,
				f.height, (int)f.clamped, w, h, (int)clamped);
	return false;
}

// 1280x720 comfortably fits inside a near-16:9 raw stream: no clamp at all.
static void test_fits_unchanged()
{
	std::printf("test_fits_unchanged\n");
	CHECK(same(fit_lores_to_raw(1280, 720, 1928, 1090), 1280, 720, false),
		  "1280x720 inside 1928x1090 is unchanged");
}

// The request exceeds the raw stream on both axes: shrink uniformly, keeping
// the request's own 16:9 aspect ratio. 640x360 is exactly half of 1280x720,
// so this case has no rounding to obscure the rule.
static void test_shrinks_to_raw()
{
	std::printf("test_shrinks_to_raw\n");
	CHECK(same(fit_lores_to_raw(1280, 720, 640, 360), 640, 360, true), "1280x720 inside 640x360 becomes 640x360");
}

// See the header-comment NOTE above: 1280x720 already fits inside 1440x1080
// on both axes, so the correct answer is "unchanged", not a shrink.
static void test_1440x1080_already_fits()
{
	std::printf("test_1440x1080_already_fits\n");
	CHECK(same(fit_lores_to_raw(1280, 720, 1440, 1080), 1280, 720, false),
		  "1280x720 inside 1440x1080 already fits -- no clamp needed");
}

// A RAW16 mode's transport frame includes optical-black padding, so its
// aspect is not exactly 16:9 (3840x2200 vs the request's 1280x720 = 16:9).
// The request still fits inside it on both axes, so it passes through
// unchanged -- the padding does not leak into the preview's shape.
static void test_padded_raw_unchanged()
{
	std::printf("test_padded_raw_unchanged\n");
	CHECK(same(fit_lores_to_raw(1280, 720, 3840, 2200), 1280, 720, false),
		  "1280x720 (16:9 request) inside 3840x2200 is unchanged");
}

// REGRESSION GUARD, the whole point of this package: a stock sensor's raw
// stream must not be affected by the clamp at all -- both hold 1280x720
// comfortably, so the fit is a no-op for imx477 and imx296.
static void test_stock_sensors_unaffected()
{
	std::printf("test_stock_sensors_unaffected\n");
	CHECK(same(fit_lores_to_raw(1280, 720, 1332, 990), 1280, 720, false), "imx477 1332x990 is unchanged");
	CHECK(same(fit_lores_to_raw(1280, 720, 1456, 1088), 1280, 720, false), "imx296 1456x1088 is unchanged");
}

// Odd inputs, on either side, come back even -- same alignment the removed
// code already applied via Size::alignDownTo(2, 2), now folded into this
// function so ConfigureVideo does not need its own copy of it.
static void test_odd_inputs_come_back_even()
{
	std::printf("test_odd_inputs_come_back_even\n");
	// Odd REQUEST, comfortably-fitting raw: aligns down before the fit check.
	LoresFit a = fit_lores_to_raw(1281, 721, 1928, 1090);
	CHECK(same(a, 1280, 720, false), "odd request 1281x721 aligns down to 1280x720 before fitting");

	// Odd RAW, request that must shrink: both the aligned request and the
	// scaled result come back even.
	LoresFit b = fit_lores_to_raw(1281, 721, 641, 361);
	CHECK(b.width % 2 == 0, "shrunk width is even even when the raw stream's own size is odd");
	CHECK(b.height % 2 == 0, "shrunk height is even even when the raw stream's own size is odd");
	CHECK(b.width <= 641 && b.height <= 361, "shrunk size still fits inside the (odd) raw stream");
}

// A zero-sized raw stream is the one case that stays genuinely impossible --
// rejected with an exception, never silently returned as a 0x0 size.
static void test_zero_raw_size_rejected()
{
	std::printf("test_zero_raw_size_rejected\n");
	bool threw_for_zero_width = false;
	try
	{
		fit_lores_to_raw(1280, 720, 0, 1080);
	}
	catch (const std::invalid_argument &)
	{
		threw_for_zero_width = true;
	}
	CHECK(threw_for_zero_width, "zero raw width throws std::invalid_argument, not a 0x0 result");

	bool threw_for_zero_height = false;
	try
	{
		fit_lores_to_raw(1280, 720, 1440, 0);
	}
	catch (const std::invalid_argument &)
	{
		threw_for_zero_height = true;
	}
	CHECK(threw_for_zero_height, "zero raw height throws std::invalid_argument, not a 0x0 result");
}

// Invariants that must hold at every input that does not hit the zero-size
// throw: never wider/taller than the raw stream, never larger than the
// (aligned) request, always even, and "unclamped" only when the aligned
// request already fit.
static void test_invariants()
{
	std::printf("test_invariants\n");
	bool exceeded_raw = false, exceeded_request = false, odd_output = false, wrongly_marked = false;
	const unsigned sizes[] = { 2, 4, 8, 100, 360, 640, 720, 721, 800, 990, 1080, 1090, 1280, 1332, 1440, 1456,
							   1928, 2160, 2200, 3840 };
	for (unsigned rw : sizes)
	{
		for (unsigned rh : sizes)
		{
			const LoresFit f = fit_lores_to_raw(1280, 720, rw, rh);
			const unsigned req_w = lores_fit_align_down_even(1280);
			const unsigned req_h = lores_fit_align_down_even(720);
			if (f.width > rw || f.height > rh)
				exceeded_raw = true;
			if (f.width > req_w || f.height > req_h)
				exceeded_request = true;
			if (f.width % 2 != 0 || f.height % 2 != 0)
				odd_output = true;
			const bool fits = req_w <= rw && req_h <= rh;
			if (f.clamped == fits) // clamped must be the NEGATION of "already fit"
				wrongly_marked = true;
		}
	}
	CHECK(!exceeded_raw, "output never exceeds the raw stream, at any size swept");
	CHECK(!exceeded_request, "output never exceeds the (aligned) request -- never enlarges");
	CHECK(!odd_output, "output is always even, at any size swept");
	CHECK(!wrongly_marked, "clamped is true exactly when the aligned request did not already fit");
}

int main()
{
	std::printf("=== lores_fit unit tests ===\n");
	test_fits_unchanged();
	test_shrinks_to_raw();
	test_1440x1080_already_fits();
	test_padded_raw_unchanged();
	test_stock_sensors_unaffected();
	test_odd_inputs_come_back_even();
	test_zero_raw_size_rejected();
	test_invariants();

	std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
	if (g_failures == 0)
		std::printf("ALL TESTS PASSED\n");
	return g_failures == 0 ? 0 : 1;
}
