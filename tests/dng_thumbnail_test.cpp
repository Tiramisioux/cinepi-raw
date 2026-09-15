// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the DNG embedded-thumbnail geometry formula AND the IFD1
// tag layout (cinepi/dng_thumbnail.hpp).
//
// Pure / self-contained: no libcamera, no Redis, no libjpeg (the JPEG
// *encode* itself lives in dng_encoder.cpp, which cannot be compiled off
// this header -- see that file's kThumbnailJpegQuality block). Build & run:
//   c++ -std=c++17 -O2 -I.. tests/dng_thumbnail_test.cpp -o /tmp/dng_thumbnail_test && /tmp/dng_thumbnail_test
// (or via meson: `meson test dng_thumbnail`).
//
// thumbnail_geometry() is the single formula DngEncoder::setup_encoder()
// calls to size its per-take buffer reservation and DngEncoder::dng_save()
// calls again to get IFD1's actual width/height/samples-per-pixel/
// reservation -- these cases pin the numbers FINDINGS.md measured on real
// files (development/dng-thumbnail-cost/FINDINGS.md, the 2026-09-13
// hardware-log entry) so a change to the formula shows up here before it
// shows up as a silently wrong byte count in a shipped DNG. cinemate's own
// _test/test_frame_size_model.py pins the same cases through the Python
// mirror, sensor_detect.thumbnail_plane_bytes().
//
// add_thumbnail_ifd1_entries() is the single place that decides IFD1's tag
// values; the second half of this file builds real IFD1 bytes with it and
// walks them with an INDEPENDENT parser (no code shared with IFDBuilder or
// dng_thumbnail.hpp itself), the same technique tests/ifd_builder_test.cpp
// uses to check IFDBuilder's own chaining contract -- this checks the tag
// VALUES a reader would actually see, not dng_thumbnail.hpp's own
// bookkeeping. Modes 0-2 (no compression) must come out byte-for-byte what
// Phase 1 shipped; mode 3 (JPEG) adds exactly two tags, 530 and 531.

#include "cinepi/dng_thumbnail.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

// ── tiny test harness (same shape as ifd_builder_test.cpp) ───────────────
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

// ── geometry cases ────────────────────────────────────────────────────────

static void check_one(uint32_t lores_w, uint32_t lores_h, int shift, int mode,
                       uint32_t want_w, uint32_t want_h, uint16_t want_spp, size_t want_bytes,
                       bool want_compressed, const char *label)
{
    const ThumbGeometry g = thumbnail_geometry(lores_w, lores_h, shift, mode);
    char msg[256];

    std::snprintf(msg, sizeof msg, "%s: width", label);
    CHECK(g.width == want_w, msg);

    std::snprintf(msg, sizeof msg, "%s: height", label);
    CHECK(g.height == want_h, msg);

    std::snprintf(msg, sizeof msg, "%s: spp", label);
    CHECK(g.spp == want_spp, msg);

    std::snprintf(msg, sizeof msg, "%s: bytes", label);
    CHECK(g.bytes == want_bytes, msg);

    std::snprintf(msg, sizeof msg, "%s: compressed", label);
    CHECK(g.compressed == want_compressed, msg);
}

// ── an independent IFD1 tag reader ────────────────────────────────────────
// Deliberately does not reuse IFDBuilder's own Pending/finalDir bookkeeping
// -- it reads the bytes build() actually wrote, the way a TIFF/DNG reader
// would, matching tests/ifd_builder_test.cpp's approach for the two-IFD
// chain.
namespace indep
{

struct RawEntry { uint16_t tag; uint16_t type; uint32_t count; uint32_t value; };

static uint16_t rd16(const uint8_t *p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
static uint32_t rd32(const uint8_t *p) { uint32_t v; std::memcpy(&v, p, 4); return v; }

static std::vector<RawEntry> read_ifd(const uint8_t *buf, uint32_t bufSize, uint32_t offset)
{
    std::vector<RawEntry> out;
    if (offset + 2 > bufSize) return out;
    uint16_t n = rd16(buf + offset);
    uint32_t p = offset + 2;
    for (uint16_t i = 0; i < n; ++i)
    {
        if (p + 12 > bufSize) break;
        RawEntry e;
        e.tag   = rd16(buf + p);
        e.type  = rd16(buf + p + 2);
        e.count = rd32(buf + p + 4);
        e.value = rd32(buf + p + 8);
        out.push_back(e);
        p += 12;
    }
    return out;
}

static const RawEntry *find(const std::vector<RawEntry> &entries, uint16_t tag)
{
    for (const auto &e : entries)
        if (e.tag == tag) return &e;
    return nullptr;
}

// SHORT values for a TIFF_SHORT entry, inline (count*2 <= 4 bytes, packed
// little-endian into `value`) or out-of-line (value is a byte offset into
// the same buffer) -- IFDBuilder's own two storage shapes, read back
// independently of it.
static std::vector<uint16_t> read_shorts(const uint8_t *buf, uint32_t bufSize, const RawEntry &e)
{
    std::vector<uint16_t> out;
    const size_t len = 2 * static_cast<size_t>(e.count);
    if (len <= 4)
    {
        for (uint32_t i = 0; i < e.count; ++i)
            out.push_back(static_cast<uint16_t>((e.value >> (16 * i)) & 0xFFFFu));
    }
    else
    {
        for (uint32_t i = 0; i < e.count; ++i)
        {
            const uint32_t off = e.value + i * 2;
            out.push_back(off + 2 <= bufSize ? rd16(buf + off) : 0xFFFFu);
        }
    }
    return out;
}

} // namespace indep

// Builds one IFD1 with add_thumbnail_ifd1_entries() for a given mode and
// checks every tag a reader would see against what that mode must mean --
// tags 254/256/257/273/277/278/279/284 (present, correct value, every
// mode), 258 (BitsPerSample: spp entries of 8), 259/262 (compression /
// photometric, mode-dependent), and 530/531 (present only when compressed,
// with the exact {2,2}/1 values, absent otherwise) -- plus the total entry
// count, so an accidentally-added or dropped tag fails even if every value
// checked individually still passed.
static void check_ifd1_tags(int mode, const char *label)
{
    // Geometry is incidental to this test (the geometry cases above already
    // pin it) -- shift 1 on a 1280x720 plane just gives a plausible, always
    // in-range width/height/spp/bytes combination for every mode.
    const ThumbGeometry tg = thumbnail_geometry(1280, 720, 1, mode);

    std::vector<uint8_t> mem(4096, 0xCC);   // poison, so unwritten bytes show up
    MemoryBuffer buf{mem.data(), 0, 0, static_cast<uint32_t>(mem.size())};

    const uint32_t stripOffset = 0x40;   // arbitrary: this test never writes a real strip
    const uint32_t stripBytes  = 12345;  // arbitrary, distinct from tg.bytes on purpose

    IFDBuilder ifd1;
    ifd1.baseOffset = buf.usedSize;
    add_thumbnail_ifd1_entries(ifd1, tg, stripOffset, stripBytes);
    ifd1.sortEntries();
    ifd1.build(buf);

    const auto entries = indep::read_ifd(mem.data(), buf.usedSize, ifd1.baseOffset);
    char msg[256];

    auto want_u32 = [&](uint16_t tag, uint32_t value, const char *what)
    {
        const indep::RawEntry *e = indep::find(entries, tag);
        std::snprintf(msg, sizeof msg, "%s: tag %u (%s) present", label, tag, what);
        CHECK(e != nullptr, msg);
        if (e)
        {
            std::snprintf(msg, sizeof msg, "%s: tag %u (%s) value", label, tag, what);
            CHECK(e->value == value, msg);
        }
    };

    want_u32(254, 1,          "SubfileType");
    want_u32(256, tg.width,   "ImageWidth");
    want_u32(257, tg.height,  "ImageLength");
    want_u32(273, stripOffset, "StripOffsets");
    want_u32(277, tg.spp,     "SamplesPerPixel");
    want_u32(278, tg.height,  "RowsPerStrip");
    want_u32(279, stripBytes, "StripByteCounts");
    want_u32(284, 1,          "PlanarConfiguration");

    // Compression: 1 uncompressed, 7 JPEG -- the tag mode 3 exists to add.
    const indep::RawEntry *comp = indep::find(entries, 259);
    std::snprintf(msg, sizeof msg, "%s: tag 259 (Compression) present", label);
    CHECK(comp != nullptr, msg);
    if (comp)
    {
        std::snprintf(msg, sizeof msg, "%s: tag 259 (Compression) value", label);
        CHECK(comp->value == (tg.compressed ? 7u : 1u), msg);
    }

    // PhotometricInterpretation: 1 mono, 2 RGB (uncompressed colour), 6
    // YCbCr (JPEG) -- matches dng_save()'s pre-Phase-2 values for modes 0-2
    // exactly, so this also pins that Phase 1's byte-identical claim holds.
    const indep::RawEntry *phot = indep::find(entries, 262);
    std::snprintf(msg, sizeof msg, "%s: tag 262 (PhotometricInterpretation) present", label);
    CHECK(phot != nullptr, msg);
    if (phot)
    {
        const uint32_t want_phot = tg.compressed ? 6u : (tg.spp == 3 ? 2u : 1u);
        std::snprintf(msg, sizeof msg, "%s: tag 262 (PhotometricInterpretation) value", label);
        CHECK(phot->value == want_phot, msg);
    }

    // BitsPerSample: one 8 per sample, spp of them -- inline for spp 1
    // (mono/off), out-of-line for spp 3 (colour/JPEG); read_shorts() covers
    // both storage shapes the same way.
    const indep::RawEntry *bits = indep::find(entries, 258);
    std::snprintf(msg, sizeof msg, "%s: tag 258 (BitsPerSample) present", label);
    CHECK(bits != nullptr, msg);
    if (bits)
    {
        const auto bitvals = indep::read_shorts(mem.data(), buf.usedSize, *bits);
        std::snprintf(msg, sizeof msg, "%s: tag 258 (BitsPerSample) count", label);
        CHECK(bitvals.size() == tg.spp, msg);
        for (size_t i = 0; i < bitvals.size(); ++i)
        {
            std::snprintf(msg, sizeof msg, "%s: tag 258 (BitsPerSample)[%zu] == 8", label, i);
            CHECK(bitvals[i] == 8, msg);
        }
    }

    // 530 (YCbCrSubSampling) / 531 (YCbCrPositioning): JPEG only. The
    // uncompressed layouts (0, 1, 2) must carry NEITHER -- Phase 1's tag
    // set, unchanged.
    const indep::RawEntry *subsamp = indep::find(entries, 530);
    const indep::RawEntry *pos     = indep::find(entries, 531);
    std::snprintf(msg, sizeof msg, "%s: tag 530 (YCbCrSubSampling) presence matches compressed", label);
    CHECK((subsamp != nullptr) == tg.compressed, msg);
    std::snprintf(msg, sizeof msg, "%s: tag 531 (YCbCrPositioning) presence matches compressed", label);
    CHECK((pos != nullptr) == tg.compressed, msg);
    if (subsamp)
    {
        const auto sub = indep::read_shorts(mem.data(), buf.usedSize, *subsamp);
        std::snprintf(msg, sizeof msg, "%s: tag 530 (YCbCrSubSampling) == {2,2}", label);
        CHECK(sub.size() == 2 && sub[0] == 2 && sub[1] == 2, msg);
    }
    if (pos)
    {
        std::snprintf(msg, sizeof msg, "%s: tag 531 (YCbCrPositioning) == 1", label);
        CHECK(pos->value == 1, msg);
    }

    // Exactly 11 tags uncompressed, 13 for JPEG -- catches an accidental
    // extra or missing tag that individual per-tag checks above would miss.
    std::snprintf(msg, sizeof msg, "%s: entry count", label);
    CHECK(entries.size() == (tg.compressed ? 13u : 11u), msg);
}

int main()
{
    std::printf("=== dng_thumbnail unit tests ===\n");

    // 16:9 lores plane (1280x720), shift 0, colour: the full-size cost
    // FINDINGS.md §2 measured on 4K/HD SDR takes -- 2,764,800 B.
    check_one(1280, 720, 0, 2, 1280, 720, 3, 2764800, false, "1280x720 s0 colour");

    // ClearHDR lores plane (1256x720), shift 0, colour: the operator's
    // 2026-09-13 example take, 2,712,960 B -- exactly what dng_ifd_dump.py
    // reported for CINEPI_26-09-13_192414_F07_C00000_cam0's frames.
    check_one(1256, 720, 0, 2, 1256, 720, 3, 2712960, false, "1256x720 s0 colour");

    // THE SHIPPED DEFAULT, shift 1: half the lores plane, 640x360 colour,
    // 691,200 B (~0.69 MB) -- FINDINGS.md §2's shift-1 column, and what
    // cinepi_controller.cpp's CP_DEF_THUMBNAIL/CP_DEF_THUMBNAIL_SIZE pair
    // produces on a standalone launch.
    check_one(1280, 720, 1, 2, 640, 360, 3, 691200, false, "1280x720 s1 colour");

    // Shift 2: quarter plane, 320x180 colour, 172,800 B -- an interim
    // default during the 2026-09-13 session, kept as a case because it is
    // what the operator's first gate takes were recorded at (the
    // hardware-log entries measure this size, not the shipped one).
    check_one(1280, 720, 2, 2, 320, 180, 3, 172800, false, "1280x720 s2 colour");

    // Mono (mode 1): same plane, spp 1, half the colour byte count.
    check_one(1280, 720, 0, 1, 1280, 720, 1, 921600, false, "1280x720 s0 mono");

    // Mode 0 (off): bytes is 0, but width/height are still reported (for
    // the "disabled" log line, which names what WOULD have been written).
    check_one(1280, 720, 3, 0, 160, 90, 1, 0, false, "1280x720 s3 mode-off");

    // Shift 12 collapses a sub-4096px plane to 1x1 -- the floor that keeps
    // an over-large thumbnail_size from ever producing a 0x0 IFD1 (the
    // same floor cinepi_controller.cpp's sync() guard exists to keep an
    // operator away from in practice).
    check_one(1272, 720, 12, 2, 1, 1, 3, 3, false, "1272x720 s12 colour (floor)");

    // Mode 3 (colour JPEG): same geometry and same `bytes` formula as mode
    // 2 at the same shift -- w*h*3 is the RESERVATION (the uncompressed
    // worst case a JPEG strip must fit inside), not the JPEG's real size,
    // which dng_save() cannot know until libjpeg has actually encoded a
    // given frame. `compressed` is the only field that tells modes 2 and 3
    // apart at this level.
    check_one(1280, 720, 0, 3, 1280, 720, 3, 2764800, true, "1280x720 s0 jpeg (reservation)");
    check_one(1256, 720, 0, 3, 1256, 720, 3, 2712960, true, "1256x720 s0 jpeg (reservation)");
    check_one(1280, 720, 1, 3, 640, 360, 3, 691200, true, "1280x720 s1 jpeg (reservation)");
    check_one(1280, 720, 2, 3, 320, 180, 3, 172800, true, "1280x720 s2 jpeg (reservation, shipped size)");

    std::printf("\n%d geometry checks so far, %d failures\n", g_checks, g_failures);

    // IFD1 tag layout, all four modes -- the uncompressed three (0, 1, 2)
    // must be exactly what dng_save() wrote inline before
    // add_thumbnail_ifd1_entries() existed; mode 3 adds exactly 530/531.
    check_ifd1_tags(0, "IFD1 tags, mode 0 (off)");
    check_ifd1_tags(1, "IFD1 tags, mode 1 (mono)");
    check_ifd1_tags(2, "IFD1 tags, mode 2 (colour)");
    check_ifd1_tags(3, "IFD1 tags, mode 3 (jpeg)");

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
