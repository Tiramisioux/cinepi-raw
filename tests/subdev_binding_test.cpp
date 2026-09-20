// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the subdev-binding decision (cinepi/subdev_binding.hpp).
//
// Pure / self-contained: no libcamera, no ioctl. Build & run:
//   c++ -std=c++17 -Wall -Wextra -I. -O1 tests/subdev_binding_test.cpp -o /tmp/subdev_binding_test && /tmp/subdev_binding_test
// (or via meson: `meson test subdev_binding`).
//
// WP-CPR-2 rework / cross-camera regression finding: core/driver_mode_metadata.hpp
// used to accept the first /dev/v4l-subdevN anywhere that exposed the five named
// controls, so two cinepi_raw processes on a dual-sensor rig could converge on the
// SAME subdev and one camera's process would silently adopt the other camera's
// Mode-Binning/Mode-Crop metadata. These cases pin exactly when a candidate is
// trusted, and that an unresolvable multi-candidate situation is reported as
// ambiguous rather than guessed.

#include "cinepi/subdev_binding.hpp"

#include <cstdio>
#include <string>
#include <utility>
#include <vector>

// ── tiny test harness (same shape as sensor_binning_source_test.cpp) ───────
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

static bool same(const SubdevBindingDecision &d, SubdevBindingResult result, int index)
{
	return d.result == result && d.index == index;
}

// No candidate on the whole system: nothing to bind to, hint or not.
static void test_none_found()
{
	std::printf("test_none_found\n");
	std::vector<std::pair<int, std::string>> none;
	CHECK(same(choose_subdev_candidate(none, ""), SubdevBindingResult::kNoneFound, -1),
		  "no candidates, no hint -> none found");
	CHECK(same(choose_subdev_candidate(none, "imx585 6-001a"), SubdevBindingResult::kNoneFound, -1),
		  "no candidates, with hint -> still none found");
}

// The single-sensor rig this package already had to keep working: exactly one
// candidate is always used, whatever the hint says (matching, mismatching, or
// absent), because there is nothing to disambiguate. This is the unchanged
// behaviour for every stock rig and every rig with one imx585/imx283.
static void test_single_candidate_always_used()
{
	std::printf("test_single_candidate_always_used\n");
	std::vector<std::pair<int, std::string>> one = { { 2, "imx585 6-001a" } };
	CHECK(same(choose_subdev_candidate(one, ""), SubdevBindingResult::kUseCandidate, 2),
		  "one candidate, no hint -> use it");
	CHECK(same(choose_subdev_candidate(one, "imx585 6-001a"), SubdevBindingResult::kUseCandidate, 2),
		  "one candidate, matching hint -> use it");
	CHECK(same(choose_subdev_candidate(one, "some other camera id"), SubdevBindingResult::kUseCandidate, 2),
		  "one candidate, NON-matching hint -> still use it (nothing else it could be)");
}

// The defect this package fixes: two candidates (dual-sensor rig, e.g. two
// imx585 units), and a hint that picks out exactly one of them by exact
// libcamera::Camera::id() match must select THAT one, regardless of scan
// order -- not the first one found.
static void test_two_candidates_hint_disambiguates()
{
	std::printf("test_two_candidates_hint_disambiguates\n");
	std::vector<std::pair<int, std::string>> two = {
		{ 2, "imx585 6-001a" }, // cam0's sensor, sorts first
		{ 5, "imx585 8-001a" }, // cam1's sensor
	};
	CHECK(same(choose_subdev_candidate(two, "imx585 8-001a"), SubdevBindingResult::kUseCandidate, 5),
		  "two candidates, hint matches the SECOND -> use it, not the first");
	CHECK(same(choose_subdev_candidate(two, "imx585 6-001a"), SubdevBindingResult::kUseCandidate, 2),
		  "two candidates, hint matches the first -> use it");

	// Order must not matter: same two candidates, reversed scan order, same hint
	// must still bind to the sensor whose name matches, not whichever sorted first.
	std::vector<std::pair<int, std::string>> two_reversed = {
		{ 5, "imx585 8-001a" },
		{ 2, "imx585 6-001a" },
	};
	CHECK(same(choose_subdev_candidate(two_reversed, "imx585 8-001a"), SubdevBindingResult::kUseCandidate, 5),
		  "scan order does not change which candidate a matching hint selects");
}

// The safe fallback: two (or more) candidates and no way to tell them apart
// -- no hint, or a hint that matches none or more than one -- must be
// reported as ambiguous so the caller falls back to its own ratio-derived
// binning instead of guessing (and silently cross-applying the wrong
// camera's geometry, which is exactly the regression this fixes).
static void test_two_candidates_unresolvable_is_ambiguous()
{
	std::printf("test_two_candidates_unresolvable_is_ambiguous\n");
	std::vector<std::pair<int, std::string>> two = {
		{ 2, "imx585 6-001a" },
		{ 5, "imx585 8-001a" },
	};
	CHECK(same(choose_subdev_candidate(two, ""), SubdevBindingResult::kAmbiguous, -1),
		  "two candidates, no hint at all -> ambiguous, not a guess");
	CHECK(same(choose_subdev_candidate(two, "neither of these"), SubdevBindingResult::kAmbiguous, -1),
		  "two candidates, hint matches neither -> ambiguous");

	// A hint that (implausibly, but the header must not assume otherwise)
	// matches BOTH candidates' names is exactly as unresolvable as matching
	// neither: still ambiguous, never "pick the first match".
	std::vector<std::pair<int, std::string>> duplicate_names = {
		{ 2, "same name" },
		{ 5, "same name" },
	};
	CHECK(same(choose_subdev_candidate(duplicate_names, "same name"), SubdevBindingResult::kAmbiguous, -1),
		  "two candidates, hint matches BOTH -> ambiguous, not the first");
}

// Three or more candidates behave the same way as two: a uniquely-matching
// hint wins, anything else is ambiguous.
static void test_more_than_two_candidates()
{
	std::printf("test_more_than_two_candidates\n");
	std::vector<std::pair<int, std::string>> three = {
		{ 1, "a" }, { 2, "b" }, { 3, "c" },
	};
	CHECK(same(choose_subdev_candidate(three, "c"), SubdevBindingResult::kUseCandidate, 3),
		  "three candidates, hint uniquely matches the third -> use it");
	CHECK(same(choose_subdev_candidate(three, "z"), SubdevBindingResult::kAmbiguous, -1),
		  "three candidates, hint matches none -> ambiguous");
	CHECK(same(choose_subdev_candidate(three, ""), SubdevBindingResult::kAmbiguous, -1),
		  "three candidates, no hint -> ambiguous");
}

int main()
{
	std::printf("=== subdev_binding unit tests ===\n");
	test_none_found();
	test_single_candidate_always_used();
	test_two_candidates_hint_disambiguates();
	test_two_candidates_unresolvable_is_ambiguous();
	test_more_than_two_candidates();

	std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
	if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
	return g_failures == 0 ? 0 : 1;
}
