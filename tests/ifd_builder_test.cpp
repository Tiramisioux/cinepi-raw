// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for IFDBuilder's chaining contract (cinepi/ifd_builder.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -O2 -I.. tests/ifd_builder_test.cpp -o /tmp/ifd_builder_test && /tmp/ifd_builder_test
// (or via meson: `meson test ifd_builder`).
//
// C9 Phase 0 (the embedded DNG thumbnail) gave IFDBuilder a public chaining
// contract: build() records where the "next IFD" field it always wrote (and
// always left at 0) landed, in nextIfdFieldOffset, so a caller can build a
// second IFD afterwards and patch that field to point at it -- the same
// after-the-fact idiom dng_save() already used to patch the TIFF header's
// own IFD0 offset. Nothing exercised that contract off-device before this
// file: the eight other g++ tests cover unrelated pure-function tiers, and
// dng_encoder.cpp itself cannot be compiled without libcamera/redis++.
//
// This test builds real IFD0+IFD1 bytes into a MemoryBuffer and walks them
// with an INDEPENDENT parser (no code shared with IFDBuilder itself) to
// verify the chain a DNG/TIFF reader would actually see: two IFDs, tags in
// ascending order in each, the next-IFD pointer resolving to exactly where
// IFD1 was built, and every offset it points at inside the buffer.

#include "cinepi/ifd_builder.hpp"

#include <cstdio>
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <optional>
#include <vector>

// ── tiny test harness (same shape as dng_pack_test.cpp) ──────────────────────
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

// ── an independent TIFF/IFD reader ────────────────────────────────────────
// Deliberately does not call anything in ifd_builder.hpp -- it exists to
// check IFDBuilder's OUTPUT the way a real DNG reader would, not to agree
// with IFDBuilder's own bookkeeping.
namespace indep {

struct Entry
{
    uint16_t tag, type;
    uint32_t count, value;
};

struct Ifd
{
    uint32_t offset;
    std::vector<Entry> entries;
    uint32_t nextIfdOffset;
};

static uint16_t rd16(const uint8_t *p) { uint16_t v; std::memcpy(&v, p, 2); return v; }
static uint32_t rd32(const uint8_t *p) { uint32_t v; std::memcpy(&v, p, 4); return v; }

// Walks the next-IFD chain starting at `firstOffset`, same as any TIFF
// reader would. Returns every IFD found, in file order.
static std::vector<Ifd> walk_chain(const uint8_t *buf, uint32_t bufSize, uint32_t firstOffset)
{
    std::vector<Ifd> out;
    uint32_t off = firstOffset;
    while (off != 0)
    {
        if (off + 2 > bufSize) break;
        Ifd ifd;
        ifd.offset = off;
        uint16_t n = rd16(buf + off);
        uint32_t p = off + 2;
        for (uint16_t i = 0; i < n; ++i)
        {
            if (p + 12 > bufSize) break;
            Entry e;
            e.tag   = rd16(buf + p);
            e.type  = rd16(buf + p + 2);
            e.count = rd32(buf + p + 4);
            e.value = rd32(buf + p + 8);
            ifd.entries.push_back(e);
            p += 12;
        }
        ifd.nextIfdOffset = (p + 4 <= bufSize) ? rd32(buf + p) : 0xFFFFFFFFu;
        out.push_back(ifd);
        off = ifd.nextIfdOffset;
        if (off == 0xFFFFFFFFu) break;   // truncated buffer -- stop, don't loop
    }
    return out;
}

} // namespace indep

// ── fixtures ───────────────────────────────────────────────────────────────

// Builds a single IFD0 with a handful of tags, some inline (<=4 bytes),
// some out-of-line (a string and an array), mirroring dng_save()'s own mix.
static void build_ifd0(IFDBuilder &ifd, MemoryBuffer &buf)
{
    ifd.baseOffset = buf.usedSize;
    static uint32_t width = 1920, height = 1080;
    static const char make[] = "Raspberry Pi";
    static uint16_t bits[4] = {12, 12, 12, 12};   // out-of-line: 4*2=8 bytes
    ifd.addEntry(256, TIFF_LONG, 1, &width);
    ifd.addEntry(257, TIFF_LONG, 1, &height);
    ifd.addEntry(271, TIFF_ASCII, sizeof(make), make);       // out-of-line
    ifd.addEntry(258, TIFF_SHORT, 4, bits);                  // out-of-line
    ifd.sortEntries();
    ifd.build(buf);
}

static void test_single_ifd_unchanged()
{
    // A one-IFD build must still leave next-IFD at 0 and be independent of
    // whether the caller ever looks at nextIfdFieldOffset at all -- the
    // regression this whole feature must not cause.
    std::vector<uint8_t> mem(4096, 0xCC);   // poison, so unwritten bytes show up
    MemoryBuffer buf{mem.data(), 0, 0, static_cast<uint32_t>(mem.size())};
    write_uint32(buf, 0);   // stand-in for a TIFF header's IFD0-offset slot

    IFDBuilder ifd0;
    build_ifd0(ifd0, buf);

    auto chain = indep::walk_chain(mem.data(), buf.usedSize, ifd0.baseOffset);
    CHECK(chain.size() == 1, "single build() produces exactly one IFD in the chain");
    CHECK(chain[0].nextIfdOffset == 0, "next-IFD field is 0 when nothing chains after it");

    // Tags ascending (sortEntries() was called).
    bool ascending = true;
    for (size_t i = 1; i < chain[0].entries.size(); ++i)
        if (chain[0].entries[i].tag <= chain[0].entries[i - 1].tag) ascending = false;
    CHECK(ascending, "IFD0 entries are in ascending tag order");
    CHECK(chain[0].entries.size() == 4, "IFD0 has the 4 entries added");
}

static void test_two_ifd_chain()
{
    // A single-IFD reference build, for the byte-identical comparison
    // below -- built BEFORE any chaining, from the exact same fixture.
    std::vector<uint8_t> memSingle(4096, 0xCC);
    MemoryBuffer bufSingle{memSingle.data(), 0, 0, static_cast<uint32_t>(memSingle.size())};
    write_uint32(bufSingle, 0);
    IFDBuilder ifd0Single;
    build_ifd0(ifd0Single, bufSingle);

    std::vector<uint8_t> mem(4096, 0xCC);
    MemoryBuffer buf{mem.data(), 0, 0, static_cast<uint32_t>(mem.size())};
    write_uint32(buf, 0);   // TIFF header IFD0-offset slot

    IFDBuilder ifd0;
    build_ifd0(ifd0, buf);

    // Before any chaining, IFD0's bytes must be identical to the
    // single-IFD build -- same fixture, same calls, nothing about
    // building IFD0 itself may depend on what happens afterwards.
    CHECK(buf.usedSize == bufSingle.usedSize, "IFD0 alone is the same size with or without a later chain");
    bool identicalBeforeChain = buf.usedSize == bufSingle.usedSize &&
        std::memcmp(mem.data(), memSingle.data(), buf.usedSize) == 0;
    CHECK(identicalBeforeChain, "IFD0's bytes (including its still-0 next-IFD field) match the single-IFD build");

    // Chain a second IFD after it -- exactly dng_save()'s own pattern:
    // build IFD1, then patch ifd0's recorded next-IFD field offset.
    IFDBuilder ifd1;
    ifd1.baseOffset = buf.usedSize;
    static uint32_t thumbW = 128, thumbH = 64;
    static uint32_t subfileType = 1;
    ifd1.addEntry(254, TIFF_LONG, 1, &subfileType);
    ifd1.addEntry(256, TIFF_LONG, 1, &thumbW);
    ifd1.addEntry(257, TIFF_LONG, 1, &thumbH);
    ifd1.sortEntries();
    ifd1.build(buf);

    CHECK(ifd0.nextIfdFieldOffset != 0, "build() recorded a non-zero next-IFD field location for IFD0");
    CHECK(ifd0.nextIfdFieldOffset < ifd1.baseOffset, "the recorded field offset lands inside IFD0's own region");

    *reinterpret_cast<uint32_t*>(buf.buffer + ifd0.nextIfdFieldOffset) = ifd1.baseOffset;

    // After patching, the ONLY bytes that may differ from the single-IFD
    // build, anywhere in IFD0's region, are the 4 at nextIfdFieldOffset --
    // which is what makes `thumbnail=0` (never reaching this patch at
    // all) byte-identical to a build with no thumbnail code present.
    const size_t nifo = ifd0.nextIfdFieldOffset;
    CHECK(std::memcmp(mem.data(), memSingle.data(), nifo) == 0,
          "no byte before the next-IFD field differs after chaining+patching");
    CHECK(std::memcmp(mem.data() + nifo, memSingle.data() + nifo, 4) != 0,
          "the next-IFD field itself is exactly where patching changed something");
    CHECK(std::memcmp(mem.data() + nifo + 4, memSingle.data() + nifo + 4, ifd1.baseOffset - nifo - 4) == 0,
          "no byte after the next-IFD field differs after chaining+patching");

    auto chain = indep::walk_chain(mem.data(), buf.usedSize, ifd0.baseOffset);
    CHECK(chain.size() == 2, "two build() calls, chained, produce exactly two IFDs");
    if (chain.size() == 2)
    {
        CHECK(chain[0].nextIfdOffset == ifd1.baseOffset,
              "IFD0's next-IFD pointer resolves to exactly where IFD1 was built");
        CHECK(chain[1].offset == ifd1.baseOffset, "the second IFD found by walking is IFD1 itself");
        CHECK(chain[1].nextIfdOffset == 0, "IFD1 (the last IFD) leaves next-IFD at 0");

        bool ascending = true;
        for (size_t i = 1; i < chain[1].entries.size(); ++i)
            if (chain[1].entries[i].tag <= chain[1].entries[i - 1].tag) ascending = false;
        CHECK(ascending, "IFD1 entries are in ascending tag order");
        CHECK(chain[1].entries.size() == 3, "IFD1 has the 3 entries added");

        // Every out-of-line value offset must land inside the buffer that
        // was actually written (usedSize), for both IFDs -- a reader
        // dereferencing e.value for a >4-byte entry must not walk off the
        // end of the file.
        for (const auto &ifd : chain)
            for (const auto &e : ifd.entries)
            {
                size_t unitSize = (e.type == TIFF_SHORT) ? 2 : (e.type == TIFF_ASCII || e.type == TIFF_BYTE) ? 1 : 4;
                size_t len = unitSize * e.count;
                if (len > 4)
                    CHECK(static_cast<size_t>(e.value) + len <= buf.usedSize,
                          "out-of-line entry value+length stays inside the written buffer");
            }
    }
}

// ── WP-CPR-3 (finding C4): DefaultCropOrigin/Size/ActiveArea ─────────────
//
// computeDngCropRect() (cinepi/ifd_builder.hpp) decides whether a frame
// needs the three DNG crop tags at all, and what they say, from the
// transport size already written under tags 256/257 and the active
// picture size the driver metadata helper (WP-CPR-2's
// core/driver_mode_metadata.hpp, output-domain: crop_width/crop_height
// divided by the driver's linear binning) reports. Absent when there is
// no such metadata (every stock sensor today) -- tags are then omitted,
// exactly as before this package.

static const uint8_t *find_out_of_line(const uint8_t *buf, uint32_t bufSize, uint32_t offset, uint32_t len)
{
    if (static_cast<size_t>(offset) + len > bufSize) return nullptr;
    return buf + offset;
}

static void test_padded_raw16_crop_tags()
{
    // The WORK-PACKAGES.md headline case: a 3840x2200 RAW16 ClearHDR
    // transport carrying a 3840x2160 active picture (40 OB rows split
    // evenly top/bottom).
    DngCropRect crop = computeDngCropRect(3840, 2200, 3840u, 2160u);
    CHECK(crop.present, "padded RAW16 frame produces crop tags");
    CHECK(crop.origin_x == 0, "padded RAW16 frame: origin_x is 0 (no horizontal padding)");
    CHECK(crop.origin_y == 20, "padded RAW16 frame: origin_y is 20 (40 OB rows split evenly)");
    CHECK(crop.width == 3840, "padded RAW16 frame: crop width is the active width");
    CHECK(crop.height == 2160, "padded RAW16 frame: crop height is the active height");

    // Build it the way dng_save() will and read it back with the
    // independent parser: tags present, right type, right values.
    std::vector<uint8_t> mem(4096, 0xCC);
    MemoryBuffer buf{mem.data(), 0, 0, static_cast<uint32_t>(mem.size())};
    write_uint32(buf, 0);

    IFDBuilder ifd;
    ifd.baseOffset = buf.usedSize;
    static uint32_t w = 3840, h = 2200;
    ifd.addEntry(256, TIFF_LONG, 1, &w);
    ifd.addEntry(257, TIFF_LONG, 1, &h);
    uint32_t origin[2] = { crop.origin_x, crop.origin_y };
    uint32_t size[2]   = { crop.width, crop.height };
    uint32_t activeArea[4] = { crop.origin_y, crop.origin_x,
                                crop.origin_y + crop.height, crop.origin_x + crop.width };
    ifd.addEntry(0xC61F, TIFF_LONG, 2, origin);
    ifd.addEntry(0xC620, TIFF_LONG, 2, size);
    ifd.addEntry(0xC68D, TIFF_LONG, 4, activeArea);
    ifd.sortEntries();
    ifd.build(buf);

    auto chain = indep::walk_chain(mem.data(), buf.usedSize, ifd.baseOffset);
    CHECK(chain.size() == 1, "crop-tag IFD builds as a single IFD");
    if (chain.size() != 1) return;

    auto find = [&](uint16_t tag) -> const indep::Entry * {
        for (const auto &e : chain[0].entries)
            if (e.tag == tag) return &e;
        return nullptr;
    };

    const indep::Entry *originEntry = find(0xC61F);
    const indep::Entry *sizeEntry   = find(0xC620);
    const indep::Entry *areaEntry   = find(0xC68D);
    CHECK(originEntry && originEntry->type == TIFF_LONG && originEntry->count == 2,
          "DefaultCropOrigin (0xC61F) is a 2-element LONG");
    CHECK(sizeEntry && sizeEntry->type == TIFF_LONG && sizeEntry->count == 2,
          "DefaultCropSize (0xC620) is a 2-element LONG");
    CHECK(areaEntry && areaEntry->type == TIFF_LONG && areaEntry->count == 4,
          "ActiveArea (0xC68D) is a 4-element LONG");

    if (originEntry)
    {
        const uint8_t *p = find_out_of_line(mem.data(), buf.usedSize, originEntry->value, 8);
        CHECK(p != nullptr, "DefaultCropOrigin value offset lands inside the written buffer");
        if (p)
        {
            CHECK(indep::rd32(p) == 0, "DefaultCropOrigin.x == 0");
            CHECK(indep::rd32(p + 4) == 20, "DefaultCropOrigin.y == 20");
        }
    }
    if (sizeEntry)
    {
        const uint8_t *p = find_out_of_line(mem.data(), buf.usedSize, sizeEntry->value, 8);
        CHECK(p != nullptr, "DefaultCropSize value offset lands inside the written buffer");
        if (p)
        {
            CHECK(indep::rd32(p) == 3840, "DefaultCropSize.width == 3840");
            CHECK(indep::rd32(p + 4) == 2160, "DefaultCropSize.height == 2160");
        }
    }
    if (areaEntry)
    {
        const uint8_t *p = find_out_of_line(mem.data(), buf.usedSize, areaEntry->value, 16);
        CHECK(p != nullptr, "ActiveArea value offset lands inside the written buffer");
        if (p)
        {
            // DNG order: top, left, bottom, right.
            CHECK(indep::rd32(p) == 20, "ActiveArea.top == 20");
            CHECK(indep::rd32(p + 4) == 0, "ActiveArea.left == 0");
            CHECK(indep::rd32(p + 8) == 2180, "ActiveArea.bottom == 2180");
            CHECK(indep::rd32(p + 12) == 3840, "ActiveArea.right == 3840");
        }
    }
}

static void test_unpadded_frame_no_crop_tags()
{
    // A 12-bit (or any non-padded) mode: the driver's active size equals
    // the transport size exactly. No crop tags -- byte-identical to every
    // DNG this stack wrote before WP-CPR-3.
    DngCropRect crop = computeDngCropRect(1440, 1080, 1440u, 1080u);
    CHECK(!crop.present, "unpadded frame: no crop tags (matches today's DNGs)");

    std::vector<uint8_t> mem(4096, 0xCC);
    MemoryBuffer buf{mem.data(), 0, 0, static_cast<uint32_t>(mem.size())};
    write_uint32(buf, 0);
    IFDBuilder ifd0;
    build_ifd0(ifd0, buf);   // the existing fixture, untouched by this package

    auto chain = indep::walk_chain(mem.data(), buf.usedSize, ifd0.baseOffset);
    CHECK(chain.size() == 1, "unpadded fixture still builds as one IFD");
    if (chain.size() != 1) return;
    bool has_crop_tag = false;
    for (const auto &e : chain[0].entries)
        if (e.tag == 0xC61F || e.tag == 0xC620 || e.tag == 0xC68D)
            has_crop_tag = true;
    CHECK(!has_crop_tag, "no crop tags appear when the mode carries no padding");
    CHECK(chain[0].entries.size() == 4, "entry count unchanged from the pre-WP-CPR-3 fixture");
}

static void test_oversized_crop_refused()
{
    // A crop rectangle must never claim more picture than the delivered
    // buffer actually holds -- pixel data is never touched, so this would
    // otherwise describe pixels that were never written.
    DngCropRect crop = computeDngCropRect(1280, 720, 1920u, 1080u);
    CHECK(!crop.present, "a crop rectangle larger than the delivered frame on both axes is refused");
    CHECK(crop.width == 0 && crop.height == 0, "a refused crop carries no geometry");

    DngCropRect crop2 = computeDngCropRect(1920, 1080, 1920u, 1200u);
    CHECK(!crop2.present, "a crop taller than the frame on one axis alone is still refused");

    DngCropRect crop3 = computeDngCropRect(1920, 1080, 2000u, 1080u);
    CHECK(!crop3.present, "a crop wider than the frame on one axis alone is still refused");
}

static void test_absent_metadata_no_crop_tags()
{
    // Every stock sensor today, and any driver whose metadata probe
    // failed: no active-size answer at all, so tags stay absent, one of
    // the two behaviours the spec explicitly allows for this case.
    DngCropRect crop = computeDngCropRect(3840, 2200, std::nullopt, std::nullopt);
    CHECK(!crop.present, "no driver metadata: tags absent, exactly as today");
}

// ── WP-CPR-3 review round 3 (finding C4-followup-2): windowed crop is NOT
// refused ──────────────────────────────────────────────────────────────
//
// crop_left/crop_top (core/driver_mode_metadata.hpp, native sensor
// coordinates per WP-585-1) locate the readout WINDOW on the physical
// sensor -- e.g. WP-585-1's own worked example, a 1440x1080 2x2 window
// reported as crop_left=480/crop_top=0 native (240,0 once divided by
// binning=2). They are NOT the active picture's origin within this
// frame's own delivered buffer, and must never be used as one -- see
// ifd_builder.hpp's file comment; that part of the finding this test
// once encoded still stands. A prior rework round, however, went further
// and made computeDngCropRect() refuse the crop tags outright for ANY
// windowed mode (crop_left != 0 || crop_top != 0), on the theory that a
// windowed mode's buffer-local OB-padding geometry was unestablished. Per
// ASPECT-RATIOS.md it is established: the RAW16 padding convention
// (delivered buffer = active + 40 rows, split 20/20) is vertical-only and
// applies identically to every ratio row, windowed or full-field alike --
// so computeDngCropRect() no longer takes a window flag at all, and
// centres for a windowed mode exactly as it does for a full-field one.

static void test_windowed_crop_still_centred()
{
    // WP-585-1's own 1440x1080 2x2 window, carrying the same +40-row
    // RAW16 OB padding the full-field mode does (1440x1120 transport for
    // a 1440x1080 active picture). No window flag exists any more to
    // refuse it -- it centres exactly like the full-field case.
    DngCropRect crop = computeDngCropRect(1440, 1120, 1440u, 1080u);
    CHECK(crop.present, "a windowed sensor readout's padded mode still produces crop tags");
    CHECK(crop.origin_x == 0 && crop.origin_y == 20,
          "windowed padded mode centres identically to the full-field case (uniform vertical-only OB padding)");
    CHECK(crop.width == 1440 && crop.height == 1080,
          "windowed padded mode: crop size is the active picture size");

    // A mode with NO padding at all (its own local buffer already equals
    // its active size) still gets no tags, windowed or not -- nothing to
    // crop either way.
    DngCropRect unpadded = computeDngCropRect(1440, 1080, 1440u, 1080u);
    CHECK(!unpadded.present, "an unpadded mode has nothing to crop, windowed sensor or not");
}

int main() {
    std::printf("=== ifd_builder unit tests ===\n");
    test_single_ifd_unchanged();
    test_two_ifd_chain();
    test_padded_raw16_crop_tags();
    test_unpadded_frame_no_crop_tags();
    test_oversized_crop_refused();
    test_absent_metadata_no_crop_tags();
    test_windowed_crop_still_centred();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
