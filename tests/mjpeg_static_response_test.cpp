// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for the static-response branch patched into the vendored MJPEG
// streamer (cinepi/third_party/nadjieb/mjpeg_streamer.hpp).
//
// Pure / self-contained: no libcamera, no Redis, no sockets opened. Build &
// run (from the repo root -- this is what .github/workflows/checks.yml's
// build_and_run() actually invokes; the "-I.." seen in some older test
// header comments here does not work from the repo root and should not be
// copied):
//   c++ -std=c++17 -O2 -I. tests/mjpeg_static_response_test.cpp -o /tmp/mjpeg_static_response_test && /tmp/mjpeg_static_response_test
// (or via meson: `meson test mjpeg_static_response`).
//
// THE POINT OF THIS TEST. mjpegPreviewStage.cpp's Configure() registers the
// clean-preview index page with MJPEGStreamer::setStaticResponse(), a method
// this vendoring patch added because upstream nadjieb 3.0.0's request
// handler (on_message_cb_) has exactly three answers -- the shutdown target,
// 405 for non-GET, 404 for an unknown target, else the multipart stream --
// and no hook for a static (non-multipart) reply. The new branch in
// on_message_cb_ (see the header's "Local patch" comment block near the
// top) builds an nadjieb::net::HTTPResponse the same way for any registered
// target: 200, the given Content-Type, a Content-Length computed from the
// body that is about to be set, Cache-Control: no-store, Connection: close,
// then the body. This test reproduces exactly that sequence -- not by
// driving a real HTTP request through MJPEGStreamer (which would need a
// live socket, out of scope for a pure test; that path is exercised on the
// Pi instead, with `curl -si http://cinepi.local:8000/`) -- and checks the
// serialized bytes are what a browser actually needs: a 200 status line,
// the right Content-Type, and a Content-Length that matches the body that
// follows it byte for byte. A Content-Length that drifts from the actual
// body is the specific, silent failure mode this guards: some browsers
// render a truncated document with no error at all when the two disagree.
//
// Whether the vendored header can even be compiled with a plain `g++` and
// no sockets/libcamera was checked by hand before writing this file (it
// can: HTTPResponse is pure string-building, and the Listener/Publisher
// classes elsewhere in the header compile fine unused -- nothing here opens
// a socket or starts a thread).

#include "cinepi/third_party/nadjieb/mjpeg_streamer.hpp"

#include <cstdio>
#include <string>

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

// Builds a static response exactly the way on_message_cb_'s new branch does
// (see the header's "Local patch" comment), given a target's registered
// {content_type, body} pair. Kept separate from the header so this test
// exercises the same call sequence a reviewer can diff against the patch by
// eye, without needing setStaticResponse()'s private map or a live request.
static nadjieb::net::HTTPResponse build_static_response(const std::string &content_type,
                                                          const std::string &body)
{
    nadjieb::net::HTTPResponse res;
    res.setVersion("HTTP/1.1");
    res.setStatusCode(200);
    res.setStatusText("OK");
    res.setValue("Content-Type", content_type);
    res.setValue("Content-Length", std::to_string(body.size()));
    res.setValue("Cache-Control", "no-store");
    res.setValue("Connection", "close");
    res.setBody(body);
    return res;
}

static void test_index_page_response()
{
    std::printf("=== index page static response ===\n");

    const std::string body =
        "<!doctype html><html><head><title>CinePi preview</title></head>"
        "<body style=\"margin:0;background:#000\"><img src=\"/stream\"></body></html>";

    const std::string serialized = build_static_response("text/html", body).serialize();

    CHECK(serialized.rfind("HTTP/1.1 200", 0) == 0,
          "serialized response starts with the status line \"HTTP/1.1 200\"");
    CHECK(serialized.find("Content-Type: text/html") != std::string::npos,
          "carries Content-Type: text/html");

    const std::string want_length = "Content-Length: " + std::to_string(body.size());
    CHECK(serialized.find(want_length) != std::string::npos,
          "carries a Content-Length matching the body's actual byte count");

    CHECK(serialized.find("Cache-Control: no-store") != std::string::npos,
          "carries Cache-Control: no-store (an index page is not cached across deploys)");
    CHECK(serialized.find("Connection: close") != std::string::npos,
          "carries Connection: close (matches the existing 404/405 branches)");

    // The body itself must follow, byte for byte -- this is what a
    // Content-Length that silently drifted from the body would break.
    CHECK(serialized.size() >= body.size() &&
              serialized.compare(serialized.size() - body.size(), body.size(), body) == 0,
          "serialized response ends with the exact body bytes");
}

static void test_empty_body()
{
    std::printf("=== empty body (defensive: Content-Length must still be exact) ===\n");

    const std::string serialized = build_static_response("text/plain", "").serialize();

    CHECK(serialized.rfind("HTTP/1.1 200", 0) == 0, "still 200 with an empty body");
    CHECK(serialized.find("Content-Length: 0") != std::string::npos,
          "Content-Length is 0, not omitted or stale");
}

// The two tests above only exercise nadjieb::net::HTTPResponse, which already
// existed upstream -- they would pass unchanged even without this patch, so
// on their own they do not fail against the unfixed code. This one calls
// MJPEGStreamer::setStaticResponse(), the method the header patch actually
// adds: revert the patch (drop the "Local patch" block from
// cinepi/third_party/nadjieb/mjpeg_streamer.hpp) and this whole test file
// fails to *compile* -- "no member named 'setStaticResponse' in
// 'nadjieb::MJPEGStreamer'" -- which is the fail-before-fix half of the
// house rule, just at compile time rather than at an assertion. It
// deliberately never calls start(): Listener/Publisher only open a socket or
// spawn threads from start()/runAsync(), so constructing MJPEGStreamer and
// registering a target stays within "no sockets" for a plain g++ run, and
// isRunning() should read back false throughout.
static void test_static_response_registration()
{
    std::printf("=== setStaticResponse is registerable without starting the listener ===\n");

    nadjieb::MJPEGStreamer streamer;
    CHECK(!streamer.isRunning(), "a freshly constructed streamer is not running");

    streamer.setStaticResponse("/", "text/html", "<!doctype html><title>CinePi preview</title>");

    CHECK(!streamer.isRunning(),
          "registering a static target does not itself start the listener or publisher");
}

int main()
{
    std::printf("=== mjpeg_static_response unit tests ===\n");
    test_index_page_response();
    test_empty_body();
    test_static_response_registration();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
