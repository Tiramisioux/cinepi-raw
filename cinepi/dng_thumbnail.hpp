#pragma once
/* ------------------------------------------------------------------ */
/*  DNG embedded-thumbnail (IFD1) geometry -- the single formula for   */
/*  the per-frame cost of the lores thumbnail chained after IFD0.      */
/* ------------------------------------------------------------------ */
/* DngEncoder::setup_encoder() calls thumbnail_geometry() to size the
 * reservation it adds to dng_info.buffer_size for this take, and
 * DngEncoder::dng_save() calls it again to get the width/height/samples-
 * per-pixel it actually writes into IFD1 -- one formula instead of the two
 * that used to compute the same numbers separately in each function and
 * could drift apart. cinemate's sensor_detect.thumbnail_plane_bytes()
 * mirrors this formula in Python for the file_size / minutes-remaining
 * estimate; see that function's docstring for the cross-reference back
 * here, and see this file's own comment for the reverse pointer.
 *
 * Pure and header-only on purpose, like ifd_builder.hpp and dng_pack.hpp:
 * no libcamera, no spdlog, so a test translation unit can #include this
 * with nothing but the standard library (tests/dng_thumbnail_test.cpp). */
#include <algorithm>
#include <cstddef>
#include <cstdint>

/* One row of geometry for the embedded thumbnail: the dimensions and byte
 * count a given (lores size, shift, mode) combination produces. width and
 * height are reported even when mode is 0 (bytes is then 0), so a caller
 * building a log line does not need a second branch to know what the
 * thumbnail WOULD have been. */
struct ThumbGeometry
{
    uint32_t width;
    uint32_t height;
    uint16_t spp;
    size_t   bytes;
};

/* lores_w / lores_h: the lores stream's own dimensions -- lo_cfg.size on
 * this side, cinemate's _calc_lores() on the Python side.
 * shift: right-shift applied to each dimension, 0..12 (thumbnail_size).
 * mode: 0 off, 1 mono (spp 1), 2 colour (spp 3) -- the caller clamps
 * options_->thumbnail to this range before passing it in.
 *
 * width/height floor at 1, never 0: this is what makes shift 12 collapse a
 * >=4096px-wide plane to a 1x1 thumbnail instead of an empty one. That
 * floor is also why cinepi_controller.cpp's sync() refuses a shift that
 * would collapse the plane below 16px and re-seeds the default instead --
 * a 1x1 IFD1 is legal TIFF but useless, and the guard exists to keep an
 * operator from reaching it by accident. */
inline ThumbGeometry thumbnail_geometry(uint32_t lores_w, uint32_t lores_h, int shift, int mode)
{
    ThumbGeometry g;
    g.width  = std::max<uint32_t>(1, lores_w  >> shift);
    g.height = std::max<uint32_t>(1, lores_h >> shift);
    g.spp    = static_cast<uint16_t>(mode == 2 ? 3 : 1);
    g.bytes  = (mode == 0) ? size_t{0} : static_cast<size_t>(g.width) * g.height * g.spp;
    return g;
}
