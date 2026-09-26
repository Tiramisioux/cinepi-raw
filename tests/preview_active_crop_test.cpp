// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for mapping the driver-reported active picture rectangle into
// the ScalerCrop rectangle the ISP wants (cinepi/preview_active_crop.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -Wall -Wextra -I. -O1 tests/preview_active_crop_test.cpp -o /tmp/preview_active_crop_test && /tmp/preview_active_crop_test
// (or via meson: `meson test preview_active_crop`).
//
// development/imx283-active-size/ROUND2.md, Defect B ("the preview bands").
// The main case here is the brief's own worked example, reproduced exactly.

#include "cinepi/preview_active_crop.hpp"

#include <cstdio>
#include <stdexcept>

// ── tiny test harness (same shape as dng_output_depth_test.cpp) ────────────
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

static bool same(const PreviewScalerCropRect &r, int left, int top, int width, int height)
{
	if (r.left == left && r.top == top && r.width == width && r.height == height)
		return true;
	std::printf("  got (%d,%d)/%dx%d; expected (%d,%d)/%dx%d\n", r.left, r.top, r.width, r.height, left, top, width,
				height);
	return false;
}

// ROUND2.md Defect B's own worked example: imx283 12-bit 2x2, full width.
// A = (108, 40, 5472, 3648), F = 2784x1828, active = (48, 0, 2736, 1824).
// The brief states the answer is (202, 40, 5378, 3640).
//
// The brief also says "the right and bottom edges land exactly on A's own
// edges, so nothing is clipped" -- true for the right edge (checked below:
// this mode has no trailing OB COLUMNS, so the active picture already runs
// to the transport frame's own right edge), but arithmetically false for the
// bottom edge with these numbers: F = 2784x1828 has 4 trailing OB ROWS below
// the 1824-row active picture (ROUND2.md Defect B's "+16 (or 4) trailing OB
// rows"), so top+height falls short of A's own bottom edge once scaled (3680
// vs A's 3688) -- consistent with there being OB rows left uncompensated on
// that edge, not with a bug in the mapping. Treated as an imprecision in the
// brief's own prose, not a contradiction of its formula or its numbers: the
// formula is verified separately against the real libcamera fork (see this
// test file's header comment and cinepi/preview_active_crop.hpp's file
// comment), and the numeric answer (202, 40, 5378, 3640) is reproduced
// exactly.
static void test_worked_example_imx283_2x2()
{
	std::printf("test_worked_example_imx283_2x2\n");
	PreviewScalerCropRect r = compute_preview_scaler_crop(108, 40, 5472, 3648, 2784, 1828, 48, 0, 2736, 1824);
	CHECK(same(r, 202, 40, 5378, 3640), "worked example reproduced exactly");
	CHECK(r.left + r.width == 108 + 5472, "right edge lands on A's own right edge (no trailing OB columns)");
	CHECK(r.top + r.height < 40 + 3648,
		  "bottom edge falls short of A's own bottom edge, consistent with trailing OB rows, not clipped");
}

// No optical black at all: the active picture covers the whole transport
// frame, so the requested crop must be exactly the sensor's own rect (A
// itself) -- the ISP's pre-existing default, unchanged.
static void test_no_padding_identity()
{
	std::printf("test_no_padding_identity\n");
	PreviewScalerCropRect r = compute_preview_scaler_crop(0, 0, 1920, 1080, 1920, 1080, 0, 0, 1920, 1080);
	CHECK(same(r, 0, 0, 1920, 1080), "active == frame maps to the sensor rect unchanged");
}

// A sensor rect that itself has a non-zero origin (as ScalerCropMaximum
// commonly does on this fork) must have that origin carried through even
// when there is no padding to compensate.
static void test_offset_sensor_rect_identity()
{
	std::printf("test_offset_sensor_rect_identity\n");
	PreviewScalerCropRect r = compute_preview_scaler_crop(108, 40, 3648, 3648, 3648, 3648, 0, 0, 3648, 3648);
	CHECK(same(r, 108, 40, 3648, 3648), "offset A carried through with no padding");
}

// A rounding overshoot must never claim more than the sensor's own reported
// area -- clamp in, not out. Construct a case where naive rounding of the
// width term alone would land 1px past A's right edge.
static void test_clamped_to_sensor_bounds()
{
	std::printf("test_clamped_to_sensor_bounds\n");
	// A = (0,0,100,100), F = 99x99 (so the scale factor is slightly > 1),
	// active = (0,0,99,99) i.e. the whole frame -- sw = 99*100/99 = 100
	// exactly, sx = 0, so this should land exactly on A with no clamp
	// needed; included to document the boundary rather than exercise it.
	PreviewScalerCropRect r = compute_preview_scaler_crop(0, 0, 100, 100, 99, 99, 0, 0, 99, 99);
	CHECK(same(r, 0, 0, 100, 100), "scale-up to a wider sensor rect lands exactly on it");
	CHECK(r.left + r.width <= 100, "never exceeds the sensor rect's right edge");
	CHECK(r.top + r.height <= 100, "never exceeds the sensor rect's bottom edge");
}

// A zero-sized transport frame is nonsensical (nothing to scale the active
// rectangle against) and stays an error, same shape as fit_lores_to_raw's
// zero-raw-stream throw.
static void test_zero_frame_throws()
{
	std::printf("test_zero_frame_throws\n");
	bool threw = false;
	try
	{
		compute_preview_scaler_crop(0, 0, 100, 100, 0, 0, 0, 0, 0, 0);
	}
	catch (const std::invalid_argument &)
	{
		threw = true;
	}
	CHECK(threw, "zero-sized transport frame throws std::invalid_argument");
}

int main()
{
	test_worked_example_imx283_2x2();
	test_no_padding_identity();
	test_offset_sensor_rect_identity();
	test_clamped_to_sensor_bounds();
	test_zero_frame_throws();

	std::printf("%d checks, %d failures\n", g_checks, g_failures);
	return g_failures == 0 ? 0 : 1;
}
