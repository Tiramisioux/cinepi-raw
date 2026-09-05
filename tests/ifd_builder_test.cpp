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

    IFDBuilder ifd0(0, 0);
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
    IFDBuilder ifd0Single(0, 0);
    build_ifd0(ifd0Single, bufSingle);

    std::vector<uint8_t> mem(4096, 0xCC);
    MemoryBuffer buf{mem.data(), 0, 0, static_cast<uint32_t>(mem.size())};
    write_uint32(buf, 0);   // TIFF header IFD0-offset slot

    IFDBuilder ifd0(0, 0);
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
    IFDBuilder ifd1(0, 0);
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

int main() {
    std::printf("=== ifd_builder unit tests ===\n");
    test_single_ifd_unchanged();
    test_two_ifd_chain();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
