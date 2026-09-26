/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * preview_active_crop.hpp - map the driver-reported active picture rectangle
 * (transport-frame pixels) into the ScalerCrop rectangle the ISP wants
 * (native sensor pixels), so the ISP scales only the picture and the leading
 * optical-black band the sensor cannot drop at the source never reaches the
 * lores/viewfinder buffer.
 *
 * development/imx283-active-size/ROUND2.md, Defect B ("the preview bands").
 *
 * THE DEFECT THIS FIXES
 * ScalerCrop defaults to the whole sensor frame (Rpi::CameraData's
 * ScalerCropMaximum), which on this fork's sensors INCLUDES the leading
 * optical-black columns the sensor always sends (imx283.c's own comment: "HTRIMMING
 * selects the active sensor window, but it does not remove the [optical
 * black]"). Every consumer of the lores stream -- the HDMI DRM plane and any
 * web/MJPEG preview built from the same buffer -- shows that band as a dark
 * strip down one side. The DNG writer already has the fix for its own path
 * (cinepi/ifd_builder.hpp's computeDngCropRect(), fed by
 * core/driver_mode_metadata.hpp's active_left/top/width/height); this header
 * is the same idea applied to ScalerCrop instead of a DNG tag.
 *
 * THE MAPPING, AND WHY IT IS THIS DIRECTION
 * Confirmed against the actual libcamera fork
 * (src/libcamera/pipeline/rpi/common/pipeline_base.cpp), not just derived:
 * CameraData::applyScalerCrop() takes the app's ScalerCrop request
 * (`nativeCrop`, in the same coordinate space as ScalerCropMaximum -- native
 * sensor pixels, already offset so (0,0) is the sensor's active area, per
 * CameraSensor's analogCrop) and turns it into the ISP's own crop with:
 *
 *   ispCrop = (nativeCrop - sensorInfo_.analogCrop.topLeft())
 *             .scaledBy(sensorInfo_.outputSize, sensorInfo_.analogCrop.size())
 *
 * i.e. ispCrop = (nativeCrop - A.topLeft()) * F / A.size(), where A is the
 * sensor's analogCrop (== ScalerCropMaximum, what this header calls the
 * "sensor rect") and F is the transport/output frame size. This header wants
 * the INVERSE: given a desired rectangle in F's own pixels (the active
 * picture, in the same domain as DriverModeMetadata's active_left/top/width/
 * height), find the nativeCrop to request so that inverting the formula
 * above reproduces it:
 *
 *   nativeCrop = desired * A.size() / F  +  A.topLeft()
 *
 * which is exactly the brief's worked mapping (ROUND2.md Defect B):
 *
 *   sx = A.left + active_left   * A.width  / F.width
 *   sy = A.top  + active_top    * A.height / F.height
 *   sw =          active_width  * A.width  / F.width
 *   sh =          active_height * A.height / F.height
 *
 * Worked example (imx283 12-bit 2x2, full width), reproduced exactly by
 * compute_preview_scaler_crop() -- see tests/preview_active_crop_test.cpp:
 *   A = (108, 40, 5472, 3648), F = 2784x1828, active = (48, 0, 2736, 1824)
 *   -> (202, 40, 5378, 3640), whose right/bottom edge lands exactly on A's
 *   own right/bottom edge (the active picture already runs to the transport
 *   frame's own right/bottom edge on this mode, so nothing is clipped).
 *
 * Rounding is round-to-nearest (not floor/truncate): the worked example's sw
 * is 2736*5472/2784 = 5377.655..., and the brief's own answer is 5378, not
 * 5377. A pure floor of every term would silently reproduce a slightly
 * different (still plausible-looking) rectangle and never be caught by eye.
 *
 * PURE / SELF-CONTAINED so the worked example can be pinned without a camera
 * or libcamera at all: no libcamera includes here on purpose, same shape as
 * cinepi/lores_fit.hpp and cinepi/sensor_binning_source.hpp. The caller
 * (cinepi_controller.cpp) wraps the result in a libcamera::Rectangle.
 */

#ifndef CINEPI_PREVIEW_ACTIVE_CROP_HPP
#define CINEPI_PREVIEW_ACTIVE_CROP_HPP

#include <cmath>
#include <cstdlib>
#include <stdexcept>

struct PreviewScalerCropRect
{
	int left;
	int top;
	int width;
	int height;
};

/* sensorLeft/Top/Width/Height   - A: the sensor's own reported crop
 *                                 rectangle in native sensor coordinates
 *                                 (libcamera's ScalerCropMaximum property /
 *                                 analogCrop, read live -- never hard-coded,
 *                                 it differs per sensor mode).
 * frameWidth/Height             - F: the transport (raw stream) frame size
 *                                 that active* below is measured against.
 * activeLeft/Top/Width/Height   - the active picture's rectangle inside the
 *                                 transport frame, transport-frame pixels --
 *                                 the same domain as
 *                                 DriverModeMetadata::active_left/top/width/
 *                                 height.
 *
 * Returns the rectangle to request as ScalerCrop, in the SAME native-sensor
 * coordinate space as sensorLeft/Top/Width/Height, so libcamera's own
 * A->F scaling (see this header's file comment) lands back on exactly the
 * active picture and nothing else.
 *
 * The result is clamped to stay within [sensorLeft, sensorLeft+sensorWidth)
 * x [sensorTop, sensorTop+sensorHeight) -- a rounding overshoot must never
 * ask the ISP for more than the sensor's own reported area, which libcamera
 * would otherwise clip in a way this header did not choose. On every mode
 * this campaign measured, the requested rectangle's own edges already land
 * exactly on the sensor rect's edges (see the worked example above), so the
 * clamp is a safety rail, not a case this fork's modes are expected to hit.
 */
inline PreviewScalerCropRect compute_preview_scaler_crop(int sensorLeft, int sensorTop, unsigned sensorWidth,
														   unsigned sensorHeight, unsigned frameWidth,
														   unsigned frameHeight, unsigned activeLeft,
														   unsigned activeTop, unsigned activeWidth,
														   unsigned activeHeight)
{
	if (frameWidth == 0 || frameHeight == 0)
		throw std::invalid_argument("compute_preview_scaler_crop: transport frame has zero size");

	auto scale_round = [](double value, double num, double den) {
		return static_cast<long long>(std::llround(value * num / den));
	};

	long long sx = sensorLeft + scale_round(activeLeft, sensorWidth, frameWidth);
	long long sy = sensorTop + scale_round(activeTop, sensorHeight, frameHeight);
	long long sw = scale_round(activeWidth, sensorWidth, frameWidth);
	long long sh = scale_round(activeHeight, sensorHeight, frameHeight);

	const long long sensorRight = static_cast<long long>(sensorLeft) + sensorWidth;
	const long long sensorBottom = static_cast<long long>(sensorTop) + sensorHeight;

	if (sx < sensorLeft)
		sx = sensorLeft;
	if (sy < sensorTop)
		sy = sensorTop;
	if (sx > sensorRight)
		sx = sensorRight;
	if (sy > sensorBottom)
		sy = sensorBottom;
	if (sx + sw > sensorRight)
		sw = sensorRight - sx;
	if (sy + sh > sensorBottom)
		sh = sensorBottom - sy;
	if (sw < 0)
		sw = 0;
	if (sh < 0)
		sh = 0;

	return { static_cast<int>(sx), static_cast<int>(sy), static_cast<int>(sw), static_cast<int>(sh) };
}

#endif /* CINEPI_PREVIEW_ACTIVE_CROP_HPP */
