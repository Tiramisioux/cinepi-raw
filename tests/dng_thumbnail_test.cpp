// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the DNG embedded-thumbnail geometry formula
// (cinepi/dng_thumbnail.hpp).
//
// Pure / self-contained: no libcamera, no Redis. Build & run:
//   c++ -std=c++17 -O2 -I.. tests/dng_thumbnail_test.cpp -o /tmp/dng_thumbnail_test && /tmp/dng_thumbnail_test
// (or via meson: `meson test dng_thumbnail`).
//
// thumbnail_geometry() is the single formula DngEncoder::setup_encoder()
// calls to size its per-take buffer reservation and DngEncoder::dng_save()
// calls again to get IFD1's actual width/height/samples-per-pixel -- these
// cases pin the numbers FINDINGS.md measured on real files (development/
// dng-thumbnail-cost/FINDINGS.md, the 2026-09-13 hardware-log entry) so a
// change to the formula shows up here before it shows up as a silently
// wrong byte count in a shipped DNG. cinemate's own
// _test/test_frame_size_model.py pins the same seven cases through the
// Python mirror, sensor_detect.thumbnail_plane_bytes().

#include "cinepi/dng_thumbnail.hpp"

#include <cstdio>

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

static void check_one(uint32_t lores_w, uint32_t lores_h, int shift, int mode,
                       uint32_t want_w, uint32_t want_h, uint16_t want_spp, size_t want_bytes,
                       const char *label)
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
}

int main()
{
    std::printf("=== dng_thumbnail unit tests ===\n");

    // 16:9 lores plane (1280x720), shift 0, colour: the full-size cost
    // FINDINGS.md §2 measured on 4K/HD SDR takes -- 2,764,800 B.
    check_one(1280, 720, 0, 2, 1280, 720, 3, 2764800, "1280x720 s0 colour");

    // ClearHDR lores plane (1256x720), shift 0, colour: the operator's
    // 2026-09-13 example take, 2,712,960 B -- exactly what dng_ifd_dump.py
    // reported for CINEPI_26-09-13_192414_F07_C00000_cam0's frames.
    check_one(1256, 720, 0, 2, 1256, 720, 3, 2712960, "1256x720 s0 colour");

    // Shipped default, shift 1: half the lores plane, 640x360 colour,
    // 691,200 B (~0.69 MB) -- FINDINGS.md §2's shift-1 column.
    check_one(1280, 720, 1, 2, 640, 360, 3, 691200, "1280x720 s1 colour");

    // Shift 2: quarter plane, 320x180 colour, 172,800 B -- the low-cost
    // alternative named in PLAN.md §2.
    check_one(1280, 720, 2, 2, 320, 180, 3, 172800, "1280x720 s2 colour");

    // Mono (mode 1): same plane, spp 1, half the colour byte count.
    check_one(1280, 720, 0, 1, 1280, 720, 1, 921600, "1280x720 s0 mono");

    // Mode 0 (off): bytes is 0, but width/height are still reported (for
    // the "disabled" log line, which names what WOULD have been written).
    check_one(1280, 720, 3, 0, 160, 90, 1, 0, "1280x720 s3 mode-off");

    // Shift 12 collapses a sub-4096px plane to 1x1 -- the floor that keeps
    // an over-large thumbnail_size from ever producing a 0x0 IFD1 (the
    // same floor cinepi_controller.cpp's sync() guard exists to keep an
    // operator away from in practice).
    check_one(1272, 720, 12, 2, 1, 1, 3, 3, "1272x720 s12 colour (floor)");

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
