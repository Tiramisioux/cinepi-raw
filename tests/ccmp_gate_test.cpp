// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the pure CCMP12 attach gate (cinepi/ccmp_gate.hpp). No
// libcamera deps. Build & run:
//   c++ -std=c++17 -O2 -I.. tests/ccmp_gate_test.cpp -o /tmp/ccmp_gate_test && /tmp/ccmp_gate_test
// (or via meson: `meson test ccmp_gate`).
//
// THE POINT OF THIS TEST. Round 2 of the mono ClearHDR fix pair added
// `sensor_mode_trusted` to the gate dng_encoder.cpp's setup_encoder() checks
// before attaching a CCMP LinearizationTable. The case that matters: a
// 12-bit ClearHDR request that lands on a sensor mode the camera actually
// configured differently (dims mismatch) must never attach a table, even
// though sensor_mode_bit_depth_ (frozen from the REQUEST) still reads 12 --
// the observed hardware failure was exactly this, a 12-bit request landing
// on the real 16-bit sensor mode. Untrusted must refuse regardless of hdr
// mode or bit depth; trusted must reproduce the pre-round-2 gate exactly.

#include "cinepi/ccmp_gate.hpp"

#include <cstdio>
#include <string>

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

int main()
{
    // Trusted: reproduces the pre-round-2 gate exactly (hdr scope x bit depth).
    CHECK(ccmp_gate_should_consider("sensor", 12, true), "sensor + 12-bit + trusted considers");
    CHECK(ccmp_gate_should_consider("auto", 12, true), "auto + 12-bit + trusted considers");
    CHECK(!ccmp_gate_should_consider("off", 12, true), "hdr off never considers");
    CHECK(!ccmp_gate_should_consider("sensor", 16, true), "16-bit ClearHDR (linear) never considers");
    CHECK(!ccmp_gate_should_consider("sensor", 10, true), "10-bit mode never considers");

    // Untrusted: the round-2 rule. Refuses even though hdr/bit-depth alone
    // would have considered it -- the exact hardware failure (12-bit
    // request, real 16-bit stream, sensor_mode_bit_depth_ still says 12).
    CHECK(!ccmp_gate_should_consider("sensor", 12, false), "sensor + 12-bit but untrusted refuses");
    CHECK(!ccmp_gate_should_consider("auto", 12, false), "auto + 12-bit but untrusted refuses");
    // Untrusted must not incidentally start mattering for a case that was
    // already excluded on hdr/bit-depth grounds -- still excluded, not a
    // different reason.
    CHECK(!ccmp_gate_should_consider("off", 12, false), "hdr off + untrusted still refuses");
    CHECK(!ccmp_gate_should_consider("sensor", 16, false), "16-bit + untrusted still refuses");

    std::printf(g_failures ? "\n%d/%d checks failed\n" : "\nall %d checks passed\n",
                g_failures ? g_failures : g_checks, g_checks);
    return g_failures ? 1 : 0;
}
