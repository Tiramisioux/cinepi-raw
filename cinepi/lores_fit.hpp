/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * lores_fit.hpp - fit a requested low-resolution (lores/preview) stream size
 * inside the raw stream's delivered size, without ever refusing the request.
 *
 * Pulled out of RPiCamApp::ConfigureVideo (core/rpicam_app.cpp) so the
 * arithmetic can be tested without a live Camera/StreamConfiguration -- same
 * reasoning and shape as cinepi/dng_output_depth.hpp. See
 * tests/lores_fit_test.cpp.
 *
 * THE DEFECT THIS REPLACES
 * With a raw stream present, cinepi-raw aliases its low-resolution stream
 * onto video stream 0, and ConfigureVideo used to refuse any lores request
 * that did not fit entirely inside the raw stream:
 *
 *   if (lores.width > raw.width || lores.height > raw.height)
 *       throw std::runtime_error("Low res image larger than raw image");
 *
 * CineMate always asks for a 720-line preview. Any sensor mode shorter than
 * 720 rows, or narrower than the requested width, aborted at configure --
 * before any of the rest of the pipeline was even reached.
 *
 * THE RULE
 * Fit the request inside the raw stream instead: keep the request's own
 * aspect ratio, never enlarge past what was asked for, and keep everything
 * even (the alignment the removed code already applied). A raw stream with
 * zero area is the one case that stays genuinely impossible -- see the throw
 * below -- everything else gets a (possibly shrunk) size, not an exception.
 *
 * A stock sensor is unaffected: imx477's 1332x990 and imx296's 1456x1088
 * both already hold a 1280x720 request, so the fit is a no-op for them (the
 * "already fits" branch returns the request unchanged).
 */

#ifndef CINEPI_LORES_FIT_HPP
#define CINEPI_LORES_FIT_HPP

#include <algorithm>
#include <stdexcept>

struct LoresFit
{
	unsigned width;
	unsigned height;
	bool clamped; /* true when the request did not fit and was shrunk */
};

/* Align a dimension down to the nearest even number -- exactly what
 * libcamera::Size::alignDownTo(2, 2) does per axis, restated here so this
 * header stays libcamera-free. */
inline unsigned lores_fit_align_down_even(unsigned v)
{
	return v & ~1u;
}

/* requested_w/h - the lores stream a caller (CineMate, today always 1280x720)
 *                 asked for.
 * raw_w/h       - the raw stream's delivered size (configuration_->at(1).size
 *                 in ConfigureVideo).
 *
 * Returns the size to actually configure the lores/video stream 0 at.
 *
 * Throws std::invalid_argument for a zero-sized raw stream: there is nothing
 * to fit anything inside, and that stays an error exactly as the throw being
 * replaced was one. ConfigureVideo no longer needs its own copy of it --
 * calling this does it, for the one case that remains genuinely impossible. */
inline LoresFit fit_lores_to_raw(unsigned requested_w, unsigned requested_h, unsigned raw_w, unsigned raw_h)
{
	if (raw_w == 0 || raw_h == 0)
		throw std::invalid_argument("fit_lores_to_raw: raw stream has zero size");

	const unsigned req_w = lores_fit_align_down_even(requested_w);
	const unsigned req_h = lores_fit_align_down_even(requested_h);

	if (req_w <= raw_w && req_h <= raw_h)
		return { req_w, req_h, false };

	/* Shrink uniformly so the request's own aspect ratio survives, using
	 * whichever axis is tighter. Both factors here are necessarily < 1.0 --
	 * req_w > raw_w or req_h > raw_h, or the branch above would have taken
	 * this request already -- so this can only shrink, never enlarge. */
	const double scale =
		std::min(raw_w / static_cast<double>(req_w), raw_h / static_cast<double>(req_h));

	unsigned fit_w = lores_fit_align_down_even(static_cast<unsigned>(req_w * scale));
	unsigned fit_h = lores_fit_align_down_even(static_cast<unsigned>(req_h * scale));

	/* Never round a real request down to nothing. */
	if (fit_w == 0)
		fit_w = 2;
	if (fit_h == 0)
		fit_h = 2;

	return { fit_w, fit_h, true };
}

#endif /* CINEPI_LORES_FIT_HPP */
