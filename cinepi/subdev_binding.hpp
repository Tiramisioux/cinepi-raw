/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * subdev_binding.hpp - which candidate /dev/v4l-subdevN belongs to THIS
 * camera process, given how many sub-devices on the system expose the
 * "Mode Binning" / "Mode Crop *" controls and (when available) a name to
 * match against the caller's own libcamera::Camera::id().
 *
 * WP-CPR-2 rework (finding: cross-camera regression). Pulled out of
 * core/driver_mode_metadata.hpp so the decision can be tested without Linux
 * headers or a live camera: see tests/subdev_binding_test.cpp. The ioctl
 * scan that produces the candidate list, and the sysfs read that produces
 * each candidate's name, cannot be tested here at all (Linux-only) -- this
 * is the part of that change that CAN be pinned.
 *
 * WHY THIS EXISTS
 * core/driver_mode_metadata.hpp used to accept the FIRST /dev/v4l-subdevN
 * anywhere on the system that happened to expose the five named controls.
 * On a single-sensor rig that is harmless: there is only ever one candidate.
 * On a dual-sensor rig running two cinepi_raw processes (e.g. two imx585
 * units, per Dual HDMI preview), both processes' scans converge on the SAME
 * first-matching subdev in the same i=0..31 order, so whichever camera's
 * subdev happens to sort first supplies its Mode-Binning/Mode-Crop values to
 * BOTH processes. If the two cameras are in different modes, the second
 * camera's process silently adopts the first camera's binning, which can
 * select the wrong (or no) CCMP-12 decompand table for the OTHER stream --
 * the same "preview renders magenta" failure this package exists to fix, now
 * reintroduced across cameras.
 *
 * cinepi_options.cpp's portFromCameraId()/detectCamPort() already extract a
 * cam0/cam1 port from libcamera::Camera::id(), and RPiCamApp::CameraId()
 * exposes that same id() string for the camera THIS process actually opened.
 * For the sensor drivers this package targets (imx585, imx283 -- both
 * v4l2_i2c_subdev_init-based), that id() is the kernel's registered v4l2
 * subdev name (e.g. "imx585 6-001a"), which is also exactly what
 * /sys/class/video4linux/v4l-subdevN/name reports for that same node. So the
 * caller can pass its own camera id() down as a binding hint and this header
 * can match candidates against it by exact string equality.
 *
 * That correlation is a property of how these drivers register themselves,
 * not a guarantee of the V4L2/libcamera APIs in general, so it is used only
 * to DISAMBIGUATE when there is more than one candidate; a hint that matches
 * nothing is never treated as a reason to reject a candidate that would
 * otherwise be the only one. When more than one candidate exists AND the
 * hint fails to pick exactly one of them (no hint given at all, or the hint
 * matches zero or more than one), the caller is told the situation is
 * AMBIGUOUS rather than being handed a guess -- this is the "gated/logged as
 * unsupported" outcome the finding asks for: the caller falls back to its
 * own ratio-derived binning, safely, instead of cross-applying another
 * camera's geometry.
 */

#ifndef CINEPI_SUBDEV_BINDING_HPP
#define CINEPI_SUBDEV_BINDING_HPP

#include <string>
#include <utility>
#include <vector>

enum class SubdevBindingResult
{
	kUseCandidate, /* exactly one candidate is the right one; see `index` */
	kNoneFound,    /* no candidate exposed the controls at all */
	kAmbiguous,    /* more than one candidate, and the hint did not pick one */
};

struct SubdevBindingDecision
{
	SubdevBindingResult result;
	int index; /* into the `candidates` vector passed in; -1 unless kUseCandidate */
};

/*
 * candidates      - every /dev/v4l-subdevN found to expose all five named
 *                    geometry controls with sane values, as (subdev index,
 *                    kernel-registered name) pairs, in scan order.
 * camera_id_hint  - this process's own libcamera::Camera::id(), or empty
 *                    when the caller has none (e.g. a --list-cameras probe
 *                    against a not-yet-opened camera, or an older call site
 *                    that predates the hint).
 *
 * Single-candidate behaviour is unchanged from before this package existed
 * either way: if only one sub-device on the whole system exposes the
 * controls, it is used, hint or no hint, matching or not -- there is nothing
 * to disambiguate. The hint only ever matters when there are two or more.
 */
inline SubdevBindingDecision choose_subdev_candidate(
	const std::vector<std::pair<int, std::string>> &candidates, const std::string &camera_id_hint)
{
	if (candidates.empty())
		return { SubdevBindingResult::kNoneFound, -1 };

	if (candidates.size() == 1)
		return { SubdevBindingResult::kUseCandidate, candidates.front().first };

	if (!camera_id_hint.empty())
	{
		int matched_index = -1;
		int matches = 0;
		for (const auto &c : candidates)
		{
			if (c.second == camera_id_hint)
			{
				matched_index = c.first;
				++matches;
			}
		}
		if (matches == 1)
			return { SubdevBindingResult::kUseCandidate, matched_index };
	}

	return { SubdevBindingResult::kAmbiguous, -1 };
}

#endif // CINEPI_SUBDEV_BINDING_HPP
