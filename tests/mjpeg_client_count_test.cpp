// SPDX-License-Identifier: BSD-2-Clause
//
// Unit test for MJPEGStreamer::clientCount(), added on fix/mjpeg-worker-
// exhaustion (cinepi/third_party/nadjieb/mjpeg_streamer.hpp) so
// mjpegPreviewStage.cpp can log the registered-client count periodically --
// see that file's Process() and the header's own "Second local patch"
// comment block near the top.
//
// Pure / self-contained: no libcamera, no Redis, no sockets opened, start()
// is never called. Build & run (from the repo root -- matches what
// .github/workflows/checks.yml's build_and_run() invokes for the sibling
// mjpeg_static_response_test.cpp):
//   c++ -std=c++17 -O2 -I. tests/mjpeg_client_count_test.cpp -o /tmp/mjpeg_client_count_test && /tmp/mjpeg_client_count_test
// (or via meson: `meson test mjpeg_client_count`).
//
// THE POINT OF THIS TEST, and its limit. clientCount() is the one piece of
// the worker-exhaustion fix that is pure enough to unit test this way: it is
// a read-only accessor over Publisher::topics_ that answers 0 for a path
// nobody has published() to yet rather than throwing, crashing, or (per its
// own header comment) inserting a Topic as a side effect the way
// topics_[path] elsewhere in this file would. What this test actually pins
// is the "compiles at all" property: it fails to compile against the pre-fix
// header with "no member named 'clientCount'", the same
// fail-before-fix-at-compile-time shape mjpeg_static_response_test.cpp uses
// for setStaticResponse().
//
// What it does NOT and CANNOT cover, because it never calls start(): the
// actual worker-exhaustion fix -- the Listener's accept-to-first-byte
// timeout, and Publisher::worker() acting on a failed send() or a
// POLLHUP/POLLERR revents instead of discarding/throwing on it. Those need a
// live socket and a real clock, which is exactly why the handbook's
// working/testing.md says socket lifecycle isn't unit-testable here; that
// was verified at the desk with a standalone (uncommitted) smoke harness
// instead -- see the PR description for what it exercised and what it did
// not.

#include "cinepi/third_party/nadjieb/mjpeg_streamer.hpp"

#include <cstdio>

// ── tiny test harness (same shape as the other tests/*_test.cpp files) ──────
static int g_failures = 0;
static int g_checks   = 0;
#define CHECK(cond, msg)                                                      \
    do {                                                                      \
        ++g_checks;                                                           \
        if (!(cond)) {                                                        \
            ++g_failures;                                                     \
            std::printf("  FAIL: %s  (%s:%d)\n", (msg), __FILE__, __LINE__);  \
        }                                                                     \
    } while (0)

static void test_unknown_path_is_zero_not_an_insert()
{
    std::printf("=== clientCount() on a path nobody has published() to ===\n");

    nadjieb::MJPEGStreamer streamer;
    CHECK(!streamer.isRunning(), "a freshly constructed streamer is not running");

    // No publish()/setStaticResponse() call at all: "/stream" does not exist
    // as a topic yet. This pins the observable half of "asking must not
    // insert" -- it reads 0, not garbage, and does not throw or terminate.
    // It cannot, through the public API alone, distinguish "no Topic was
    // created" from "an empty Topic was created and correctly reports 0
    // clients" -- clientCount()'s own comment states the intent (find(),
    // not operator[]) as the actual guarantee; a reviewer diffing that one
    // line is what actually confirms it.
    CHECK(streamer.clientCount("/stream") == 0,
          "an unregistered path reports 0 clients rather than throwing/crashing");

    CHECK(!streamer.isRunning(),
          "clientCount() does not itself start the listener or publisher");
}

int main()
{
    std::printf("=== mjpeg_client_count unit tests ===\n");
    test_unknown_path_is_zero_not_an_insert();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
