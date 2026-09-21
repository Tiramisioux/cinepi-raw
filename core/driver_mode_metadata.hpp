/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * driver_mode_metadata.hpp - probe a sensor sub-device's read-only mode
 * geometry, by control NAME rather than by control id.
 *
 * WP-CPR-2 (finding C2). Shared by core/options.cpp (--list-cameras, which
 * already probed this, but only for the imx585's own control ids) and
 * cinepi_raw.cpp's live CCMP-binning decision, which needs the same answer
 * while the camera is actually running. Both must find the same thing the
 * same way, or the app can disagree with its own --list-cameras output.
 *
 * The contract, per WP-283-5 and ASPECT-RATIOS.md, is the control NAME:
 *
 *   "Mode Binning", "Mode Crop Left", "Mode Crop Top",
 *   "Mode Crop Width", "Mode Crop Height"
 *
 * not any particular control id. The imx585 happens to expose these at a
 * fixed offset from its own custom-control base; a later driver (imx283,
 * WP-283-5) is free to pick different ids as long as the names match. This
 * header walks the sub-device's control list with V4L2_CTRL_FLAG_NEXT_CTRL
 * and matches on name, so no per-model branch is needed here at all. The
 * imx585 ids are kept only as a first guess to skip most of the walk on the
 * sensor we know about — an id match with the wrong name is never accepted,
 * and the walk still runs when the guess misses.
 *
 * Values are reported by the driver in NATIVE SENSOR COORDINATES: binning is
 * the linear factor (1, 2, 3 ... never the squared "pixels summed" factor), and
 * crop_left/top/width/height are the sensor-side readout window. Squaring
 * the binning into the samples-per-pixel factor the CCMP tables key on
 * (cinepi/ccmp_lut.hpp's kT1Effective, 1 or 4) is the CALLER's job — see
 * cinepi/sensor_binning_source.hpp — because callers disagree about which
 * domain they want back (options.cpp prints the linear factor as "2x2").
 *
 * NOT unit-testable here: this opens /dev/v4l-subdevN and issues real
 * ioctls, which is Linux-only and needs a bound sensor driver. It cannot be
 * built on this Mac at all (linux/videodev2.h does not exist here). Keep
 * this file thin for exactly that reason, and put any decision logic that
 * CAN be tested (such as which of two candidate binning values to trust) in
 * its own pure header instead. See tests/sensor_binning_source_test.cpp.
 *
 * CROSS-CAMERA BINDING. On a dual-sensor rig (two cinepi_raw processes, e.g.
 * two imx585 units per Dual HDMI preview) more than one /dev/v4l-subdevN can
 * expose the five named controls at once, and without a camera-id hint this
 * probe cannot tell which one belongs to THIS process. Pass the calling
 * process's own libcamera::Camera::id() (RPiCamApp::CameraId(), the same
 * signal cinepi_options.cpp's portFromCameraId()/detectCamPort() already
 * uses) as `camera_id_hint` wherever it is available. The decision of which
 * candidate to trust given that hint is pure and tested separately: see
 * cinepi/subdev_binding.hpp and tests/subdev_binding_test.cpp. On a
 * single-sensor rig, or when no hint is available at all (the
 * --list-cameras probe against a not-yet-opened camera), behaviour is
 * unchanged: there being only one candidate is what makes today's fast path
 * safe, not the absence of a hint.
 */

#ifndef CINEPI_DRIVER_MODE_METADATA_HPP
#define CINEPI_DRIVER_MODE_METADATA_HPP

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "cinepi/subdev_binding.hpp"

/* Upper bound on a believable linear binning factor. A sanity rail, not a
 * statement about any sensor: sensors in this stack program 1, 2 or 3, and
 * the point of the check is to reject a control that was never updated (0),
 * a negative stray, or a wild read -- not to enumerate what drivers may do.
 * Set it above what is plausible rather than at what is currently shipped,
 * because the failure mode of setting it too low is silent (see the
 * kMaxSaneBinning note on driver_mode_metadata() below). */
static constexpr int kMaxSaneBinning = 8;

/* The driver's answer for the currently active mode, in native sensor
 * coordinates. `valid` is false until a sub-device has been found that
 * exposes all five controls by name with sane values. */
struct DriverModeMetadata
{
	int binning = 0;      /* linear factor: 1, 2, 3 ... NOT squared */
	int crop_left = 0;
	int crop_top = 0;
	int crop_width = 0;
	int crop_height = 0;
	bool valid = false;
};

namespace driver_mode_metadata_detail
{

/* Deliberately NOT called `xioctl`. A translation unit that includes this header
 * may have its own file-scope `xioctl` -- core/options.cpp does -- and because the
 * v4l2 argument types live in the global namespace, argument-dependent lookup adds
 * that one beside this one at every unqualified call, which is ambiguous and does
 * not compile. The name carries the namespace so the two can never collide. */
inline int metadata_ioctl(int fd, unsigned long ctl, void *arg)
{
	int ret, num_tries = 10;
	do
	{
		ret = ioctl(fd, ctl, arg);
	} while (ret == -1 && errno == EINTR && num_tries-- > 0);
	return ret;
}

/* The imx585's own control ids. Fast-path hints only — see the file
 * comment. A future driver with different ids still works, just via the
 * full walk below. */
#ifndef V4L2_CID_USER_IMX585_BASE
#define V4L2_CID_USER_IMX585_BASE (V4L2_CID_USER_BASE + 0x2000)
#endif
constexpr __u32 kImx585Binning    = V4L2_CID_USER_IMX585_BASE + 10;
constexpr __u32 kImx585CropLeft   = V4L2_CID_USER_IMX585_BASE + 11;
constexpr __u32 kImx585CropTop    = V4L2_CID_USER_IMX585_BASE + 12;
constexpr __u32 kImx585CropWidth  = V4L2_CID_USER_IMX585_BASE + 13;
constexpr __u32 kImx585CropHeight = V4L2_CID_USER_IMX585_BASE + 14;

inline std::string control_name(const v4l2_queryctrl &q)
{
	const char *name = reinterpret_cast<const char *>(q.name);
	return std::string(name, strnlen(name, sizeof(q.name)));
}

/* Read back the value of an already-resolved control id. */
inline bool read_control_value(int fd, __u32 id, int &value)
{
	v4l2_ext_control ctrl{};
	ctrl.id = id;

	v4l2_ext_controls ctrls{};
	ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(id);
	ctrls.count = 1;
	ctrls.controls = &ctrl;

	if (metadata_ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0)
	{
		value = ctrl.value;
		return true;
	}

	// Some drivers only answer the legacy user-control ioctl for a custom
	// control; try it before giving up.
	v4l2_control legacy{ id, 0 };
	if (metadata_ioctl(fd, VIDIOC_G_CTRL, &legacy) == 0)
	{
		value = legacy.value;
		return true;
	}

	return false;
}

/* Resolve `name` to a control id on this sub-device. Tries `hint_id` first
 * (a single VIDIOC_QUERYCTRL, when the caller has a guess), but only ever
 * accepts it after checking the name; otherwise walks every control with
 * V4L2_CTRL_FLAG_NEXT_CTRL until the name matches or the list ends. */
inline bool find_control_by_name(int fd, const char *name, __u32 hint_id, __u32 &out_id)
{
	if (hint_id != 0)
	{
		v4l2_queryctrl q{};
		q.id = hint_id;
		if (metadata_ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 &&
			!(q.flags & V4L2_CTRL_FLAG_DISABLED) &&
			control_name(q) == name)
		{
			out_id = q.id;
			return true;
		}
	}

	v4l2_queryctrl q{};
	q.id = V4L2_CTRL_FLAG_NEXT_CTRL;
	while (metadata_ioctl(fd, VIDIOC_QUERYCTRL, &q) == 0)
	{
		if (!(q.flags & V4L2_CTRL_FLAG_DISABLED) && control_name(q) == name)
		{
			out_id = q.id;
			return true;
		}
		q.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
	}

	return false;
}

/* The kernel-registered name of /dev/v4l-subdevN, e.g. "imx585 6-001a", read
 * from sysfs rather than an ioctl (no fd needed, works even if the caller's
 * open of the device node itself is about to be closed). This is the same
 * string libcamera exposes as Camera::id() for these i2c-registered sensor
 * drivers (v4l2_i2c_subdev_init sets both from the i2c_client name), which
 * is what makes it usable as a binding hint — see the file comment. Returns
 * empty on any failure; an empty name never matches a non-empty hint. */
inline std::string subdev_sysfs_name(int index)
{
	std::string path = "/sys/class/video4linux/v4l-subdev" + std::to_string(index) + "/name";
	std::ifstream f(path);
	if (!f.good())
		return {};
	std::string name;
	std::getline(f, name);
	while (!name.empty() && (name.back() == '\n' || name.back() == '\r'))
		name.pop_back();
	return name;
}

} // namespace driver_mode_metadata_detail

/*
 * Probe /dev/v4l-subdev0..31 for the five geometry controls, by name, and
 * fill `m` from the sub-device that belongs to THIS camera process.
 *
 * Query the controls themselves before accepting a subdev, rather than
 * assuming subdev 0: on a Pi 5 with multiple sensor sub-devices,
 * /dev/v4l-subdevN is not a stable sensor identity, and the first subdev is
 * not necessarily the sensor this app is talking to.
 *
 * `camera_id_hint`, when non-empty, is expected to be this process's own
 * libcamera::Camera::id() (RPiCamApp::CameraId()). It disambiguates between
 * MULTIPLE sub-devices that all expose the controls (a dual-sensor rig): see
 * the file comment and cinepi/subdev_binding.hpp. On a rig with only one
 * such sub-device the hint changes nothing — that candidate is used either
 * way, exactly as before this parameter existed.
 *
 * A sane binning is 1..kMaxSaneBinning. Anything else — 0 (never updated),
 * a negative stray, or something implausibly large — is treated as "no
 * metadata" so a caller falls back to its own derivation rather than
 * trusting a bogus value.
 *
 * This used to read `binning > 2`, on the stated grounds that "imx585 and
 * its imx283 equivalent only ever program 1x1 or 2x2". That was true of
 * the imx585 and never true of the imx283: IMX283_MODE_3 is a 3x3 readout,
 * it is not experimental, and it is listed. The bound was in fact the
 * imx585 control's own .max = 2 copied into a shared reader, and the cost
 * was silent — a rejected candidate drops the WHOLE annotation, so on the
 * first imx283 hardware run its 1856x1220 mode came out of --list-cameras
 * as a bare `[60.36 fps - (0, 0)/5472x3648 crop]` with no binning or
 * mode-crop clause at all, while the other 20 modes carried theirs. It
 * read as a driver gap; it was this line.
 *
 * KNOWN GAP, not fixed here: "Mode Binning" is a single integer, so it
 * cannot express an asymmetric ratio. The imx283's 3x1 subsampling modes
 * (MODE_4, MODE_5) report 3 per DEC-4's "report the horizontal ratio",
 * which a consumer will read as 3x3. Both are gated off behind that
 * driver's experimental_modes param today. Expressing them needs a second
 * control and a contract change on both sides.
 *
 * When more than one sub-device qualifies and the hint does not pick out
 * exactly one of them (no hint given, or it matches zero or more than one),
 * this returns false rather than guessing: a wrong guess here silently
 * cross-applies one camera's geometry to another's stream, which is worse
 * than the caller's own fallback. One line is logged to stderr so the
 * situation is visible instead of a silent "no metadata".
 */
inline bool read_driver_mode_metadata(DriverModeMetadata &m, const std::string &camera_id_hint = std::string())
{
	using namespace driver_mode_metadata_detail;

	struct Candidate
	{
		int index;
		DriverModeMetadata meta;
	};
	std::vector<Candidate> candidates;
	std::vector<std::pair<int, std::string>> candidate_names;

	for (int i = 0; i < 32; ++i)
	{
		std::string dev = "/dev/v4l-subdev" + std::to_string(i);
		int fd = open(dev.c_str(), O_RDWR);
		if (fd < 0)
			continue;

		__u32 id_binning = 0, id_left = 0, id_top = 0, id_width = 0, id_height = 0;
		const bool found =
			find_control_by_name(fd, "Mode Binning", kImx585Binning, id_binning) &&
			find_control_by_name(fd, "Mode Crop Left", kImx585CropLeft, id_left) &&
			find_control_by_name(fd, "Mode Crop Top", kImx585CropTop, id_top) &&
			find_control_by_name(fd, "Mode Crop Width", kImx585CropWidth, id_width) &&
			find_control_by_name(fd, "Mode Crop Height", kImx585CropHeight, id_height);

		if (!found)
		{
			close(fd);
			continue;
		}

		int binning = 0, left = 0, top = 0, width = 0, height = 0;
		const bool ok =
			read_control_value(fd, id_binning, binning) &&
			read_control_value(fd, id_left, left) &&
			read_control_value(fd, id_top, top) &&
			read_control_value(fd, id_width, width) &&
			read_control_value(fd, id_height, height);

		close(fd);

		if (!ok || binning < 1 || binning > kMaxSaneBinning ||
			left < 0 || top < 0 || width <= 0 || height <= 0)
			continue;

		DriverModeMetadata candidate_meta;
		candidate_meta.binning = binning;
		candidate_meta.crop_left = left;
		candidate_meta.crop_top = top;
		candidate_meta.crop_width = width;
		candidate_meta.crop_height = height;
		candidate_meta.valid = true;

		candidates.push_back({ i, candidate_meta });
		candidate_names.push_back({ i, subdev_sysfs_name(i) });

		// Keep walking the whole range rather than stopping at the first (or
		// second) match: the hint might identify a LATER candidate as this
		// process's own camera, and reporting an accurate total candidate
		// count is what makes the single-vs-ambiguous decision below
		// correct. The spec's own item 2 already accepts this as a
		// per-reconfigure cost, not a per-frame one.
	}

	const SubdevBindingDecision decision = choose_subdev_candidate(candidate_names, camera_id_hint);

	if (decision.result == SubdevBindingResult::kAmbiguous)
	{
		std::fprintf(stderr,
					 "cinepi-raw: %zu sensor sub-devices expose Mode Binning/Mode Crop "
					 "controls and the camera id%s did not identify exactly one; "
					 "ignoring driver mode metadata for this camera (falling back to "
					 "the ratio-derived binning) rather than risking a cross-camera "
					 "mismatch.\n",
					 candidate_names.size(), camera_id_hint.empty() ? " hint was empty" : " matched none/more than one");
		return false;
	}

	if (decision.result == SubdevBindingResult::kNoneFound)
		return false;

	for (const auto &c : candidates)
	{
		if (c.index == decision.index)
		{
			m = c.meta;
			return true;
		}
	}

	// Unreachable: choose_subdev_candidate() only ever returns an index that
	// was present in candidate_names, which is built 1:1 with `candidates`.
	return false;
}

#endif // CINEPI_DRIVER_MODE_METADATA_HPP
