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
 * the linear factor (1 or 2, never the squared "pixels summed" factor), and
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
 */

#ifndef CINEPI_DRIVER_MODE_METADATA_HPP
#define CINEPI_DRIVER_MODE_METADATA_HPP

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>

/* The driver's answer for the currently active mode, in native sensor
 * coordinates. `valid` is false until a sub-device has been found that
 * exposes all five controls by name with sane values. */
struct DriverModeMetadata
{
	int binning = 0;      /* linear factor: 1 or 2, NOT squared */
	int crop_left = 0;
	int crop_top = 0;
	int crop_width = 0;
	int crop_height = 0;
	bool valid = false;
};

namespace driver_mode_metadata_detail
{

inline int xioctl(int fd, unsigned long ctl, void *arg)
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

	if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) == 0)
	{
		value = ctrl.value;
		return true;
	}

	// Some drivers only answer the legacy user-control ioctl for a custom
	// control; try it before giving up.
	v4l2_control legacy{ id, 0 };
	if (xioctl(fd, VIDIOC_G_CTRL, &legacy) == 0)
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
		if (xioctl(fd, VIDIOC_QUERYCTRL, &q) == 0 &&
			!(q.flags & V4L2_CTRL_FLAG_DISABLED) &&
			control_name(q) == name)
		{
			out_id = q.id;
			return true;
		}
	}

	v4l2_queryctrl q{};
	q.id = V4L2_CTRL_FLAG_NEXT_CTRL;
	while (xioctl(fd, VIDIOC_QUERYCTRL, &q) == 0)
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

} // namespace driver_mode_metadata_detail

/*
 * Probe /dev/v4l-subdev0..31 for the five geometry controls, by name, and
 * fill `m` from whichever sub-device has all five with sane values.
 *
 * Query the controls themselves before accepting a subdev, rather than
 * assuming subdev 0: on a Pi 5 with multiple sensor sub-devices,
 * /dev/v4l-subdevN is not a stable sensor identity, and the first subdev is
 * not necessarily the sensor this app is talking to.
 *
 * A sane binning is 1 or 2 (imx585_program_window and its imx283 equivalent
 * only ever program 1x1 or 2x2). Anything else — 0 (never updated), a
 * negative stray, or something larger — is treated as "no metadata" so a
 * caller falls back to its own derivation rather than trusting a bogus
 * value.
 */
inline bool read_driver_mode_metadata(DriverModeMetadata &m)
{
	using namespace driver_mode_metadata_detail;

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

		if (!ok || binning < 1 || binning > 2 ||
			left < 0 || top < 0 || width <= 0 || height <= 0)
			continue;

		m.binning = binning;
		m.crop_left = left;
		m.crop_top = top;
		m.crop_width = width;
		m.crop_height = height;
		m.valid = true;
		return true;
	}

	return false;
}

#endif // CINEPI_DRIVER_MODE_METADATA_HPP
