#pragma once
/* ------------------------------------------------------------------ */
/*  DNG embedded-thumbnail (IFD1) geometry AND tag layout -- the       */
/*  single formula for the per-frame cost and the single place that    */
/*  decides what IFD1 says, for every mode including JPEG.             */
/* ------------------------------------------------------------------ */
/* DngEncoder::setup_encoder() calls thumbnail_geometry() to size the
 * reservation it adds to dng_info.buffer_size for this take, and
 * DngEncoder::dng_save() calls it again to get the width/height/samples-
 * per-pixel/reservation it builds IFD1 from, then calls
 * add_thumbnail_ifd1_entries() to write that IFD1's tags -- one formula
 * and one tag layout instead of the several separately-written copies
 * that used to compute or emit the same facts and could drift apart.
 * cinemate's sensor_detect.thumbnail_plane_bytes() mirrors the byte
 * formula in Python for the file_size / minutes-remaining estimate; see
 * that function's docstring for the cross-reference back here, and see
 * below for the one number it does NOT mirror (the JPEG reservation) and
 * why.
 *
 * Four modes: 0 off, 1 mono (uncompressed, spp 1), 2 colour (uncompressed,
 * spp 3), 3 colour JPEG (spp 3, baseline JPEG YCbCr 4:2:0 quality 85 --
 * kThumbnailJpegQuality in dng_encoder.cpp). Modes 1 and 2 are unchanged
 * by mode 3's addition: same geometry, same tag values, same bytes -- see
 * tests/dng_thumbnail_test.cpp's IFD1-tag cases, which pin that the three
 * uncompressed layouts (0 has none, 1, 2) are byte-identical to what this
 * header replaced.
 *
 * Pure and header-only on purpose, like ifd_builder.hpp and dng_pack.hpp:
 * no libcamera, no spdlog, no jpeglib -- so a test translation unit can
 * #include this with nothing but the standard library plus ifd_builder.hpp
 * (tests/dng_thumbnail_test.cpp). The JPEG *encode* itself (libjpeg) stays
 * in dng_encoder.cpp, which cannot be compiled off-device anyway
 * (libcamera) -- this header only describes what a JPEG thumbnail's IFD1
 * must say, not how to produce its bytes. */
#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "ifd_builder.hpp"

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
    /* True only for mode 3. For every other mode `bytes` is the strip's
     * actual, exact size -- dng_save() writes precisely that many bytes,
     * every time. For mode 3 `bytes` is instead a RESERVATION: the
     * uncompressed worst case (width*height*3, identical to what mode 2
     * would need at the same geometry) that setup_encoder() sizes the
     * take's buffer against and dng_save() must not exceed. A JPEG strip's
     * real size is almost always far smaller (FINDINGS.md §2b: 9-16 KB at
     * 640x360 against a 691,200 B reservation) and is only known once
     * libjpeg has actually finished compressing a given frame -- `bytes`
     * cannot report that per-frame number, because this function runs
     * before any pixel is encoded. cinemate's
     * sensor_detect.thumbnail_plane_bytes() mirrors `bytes` for modes 0-2
     * exactly, and for mode 3 carries its own separately-named ESTIMATE
     * constant instead of this reservation, because a file-size /
     * minutes-remaining figure built from the reservation would be
     * wrong by roughly the same factor the compression buys -- see that
     * function's docstring. */
    bool     compressed;
};

/* lores_w / lores_h: the lores stream's own dimensions -- lo_cfg.size on
 * this side, cinemate's _calc_lores() on the Python side.
 * shift: right-shift applied to each dimension, 0..12 (thumbnail_size).
 * mode: 0 off, 1 mono (spp 1), 2 colour (spp 3), 3 colour JPEG (spp 3,
 * compressed) -- the caller clamps options_->thumbnail to this range
 * before passing it in.
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
    g.width      = std::max<uint32_t>(1, lores_w  >> shift);
    g.height     = std::max<uint32_t>(1, lores_h >> shift);
    g.spp        = static_cast<uint16_t>((mode == 2 || mode == 3) ? 3 : 1);
    g.compressed = (mode == 3);
    /* mode 3's bytes is the same w*h*spp formula as mode 2 at the same
     * geometry -- deliberately: a JPEG must fit in what the uncompressed
     * plane would have taken (dng_save()'s guard before it ever writes the
     * strip), so the reservation IS the uncompressed number, not a
     * separately-computed estimate. See ThumbGeometry::compressed above
     * for why this same field means something different for mode 3. */
    g.bytes = (mode == 0) ? size_t{0} : static_cast<size_t>(g.width) * g.height * g.spp;
    return g;
}

/* Writes IFD1's eleven core tags (thirteen for JPEG) into `ifd1`, given the
 * geometry this take's thumbnail already has (from thumbnail_geometry())
 * and the strip's actual offset/size once dng_save() has written it.
 * `ifd1.sortEntries()` and `ifd1.build()` stay the caller's job, same as
 * before this function existed -- this only decides which tags exist and
 * what they say, not how IFDBuilder lays out the bytes.
 *
 * Tag values are ordinary (non-static, non-thread_local) locals: dng_save()
 * runs on a pool of encoder threads (see thumbRow's own thread_local
 * comment in dng_encoder.cpp), so a function-local `static` here would be
 * one mutable object shared, unsynchronised, across every one of them.
 *
 * Uncompressed modes (mono spp 1, colour spp 3) are byte-identical to what
 * dng_save() wrote inline before this function existed: same eleven tags,
 * same values, same add order (sortEntries() still runs after, in the
 * caller) -- tests/dng_thumbnail_test.cpp's IFD1 cases pin this with the
 * same independent-parser technique tests/ifd_builder_test.cpp uses.
 *
 * JPEG (mode 3, ThumbGeometry::compressed) adds the two tags a DNG/TIFF
 * reader needs to decode photometric YCbCr correctly: 530 YCbCrSubSampling
 * {2,2} -- matches libjpeg's default 4:2:0 subsampling for JCS_RGB input
 * (dng_encoder.cpp's kThumbnailJpegQuality block) -- and 531
 * YCbCrPositioning 1 (centered), the value every other DNG/TIFF writer in
 * this ecosystem uses for a JPEG preview. Tag 258 (BitsPerSample) is still
 * written for JPEG too, 8 per sample: it describes the SAMPLE depth, which
 * is unrelated to whether the strip itself is compressed, and DNG readers
 * expect it present regardless. */
inline void add_thumbnail_ifd1_entries(IFDBuilder &ifd1, const ThumbGeometry &g,
                                        uint32_t stripOffset, uint32_t stripBytes)
{
    constexpr uint16_t kCompressionUncompressed = 1;
    constexpr uint16_t kCompressionJpeg         = 7;
    constexpr uint16_t kPhotometricMinIsBlack   = 1;   /* mono */
    constexpr uint16_t kPhotometricRgb          = 2;   /* colour, uncompressed */
    constexpr uint16_t kPhotometricYCbCr        = 6;   /* JPEG only */

    uint32_t subfileType  = 1;                /* thumbnail/reduced-res image */
    uint16_t bitsArr[3]   = {8, 8, 8};         /* one entry per sample, TIFF-spec count */
    uint16_t compression  = g.compressed ? kCompressionJpeg : kCompressionUncompressed;
    uint16_t photometric  = g.compressed ? kPhotometricYCbCr
                          : (g.spp == 3   ? kPhotometricRgb : kPhotometricMinIsBlack);
    uint16_t planar       = 1;
    uint16_t ycbcrSubSampling[2] = {2, 2};
    uint16_t ycbcrPositioning    = 1;

    ifd1.addEntry(254, TIFF_LONG , 1     , &subfileType);
    ifd1.addEntry(256, TIFF_LONG , 1     , &g.width);
    ifd1.addEntry(257, TIFF_LONG , 1     , &g.height);
    ifd1.addEntry(258, TIFF_SHORT, g.spp , bitsArr);
    ifd1.addEntry(259, TIFF_SHORT, 1     , &compression);
    ifd1.addEntry(262, TIFF_SHORT, 1     , &photometric);
    ifd1.addEntry(273, TIFF_LONG , 1     , &stripOffset);
    ifd1.addEntry(277, TIFF_SHORT, 1     , &g.spp);
    ifd1.addEntry(278, TIFF_LONG , 1     , &g.height);
    ifd1.addEntry(279, TIFF_LONG , 1     , &stripBytes);
    ifd1.addEntry(284, TIFF_SHORT, 1     , &planar);
    if (g.compressed)
    {
        ifd1.addEntry(530, TIFF_SHORT, 2, ycbcrSubSampling);
        ifd1.addEntry(531, TIFF_SHORT, 1, &ycbcrPositioning);
    }
}
