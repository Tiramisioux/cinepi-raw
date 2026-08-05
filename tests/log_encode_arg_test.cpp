// SPDX-License-Identifier: BSD-2-Clause
//
// Unit tests for --log-encode argv parsing (cinepi/log_encode_arg.hpp).
//
// Pure / self-contained: no libcamera, no boost, no Redis. Build & run:
//   c++ -std=c++17 -O2 -Wall -Wextra -I. tests/log_encode_arg_test.cpp \
//       -o /tmp/log_encode_arg_test && /tmp/log_encode_arg_test
// (or via meson: `meson test log_encode_arg`).
//
// Why this file exists: CinePiOptions::Parse walks argv by hand, so a bare flag
// that may or may not be followed by its value has to decide whether to consume
// the NEXT argv. Get that wrong and cinepi-raw silently eats an unrelated
// argument, or silently records the wrong depth. Neither shows up in a build.
// The rules under test:
//
//   --log-encode        -> 12, next argv untouched
//   --log-encode 10/12  -> that depth, next argv consumed
//   --log-encode=10/12  -> that depth
//   --log-encode 11     -> 12, "11" forwarded (a bare flag cannot claim an
//                          unknown token — it may belong to rpicam-apps)
//   --log-encode=11     -> hard error (with '=' there is no ambiguity, so a
//                          typo must not quietly fall back to 12)
//
// Tier 1 — the token classifier      (which strings are values / are our flag)
// Tier 2 — one token at a time       (every form, incl. end-of-argv)
// Tier 3 — a whole argv walk         (the ++i interaction; what forwards on)
// Tier 4 — --keep16 is really gone   (must forward now, not be swallowed)

#include "cinepi/log_encode_arg.hpp"

#include <cstdio>
#include <string>
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

// ── Tier 1: the token classifier ─────────────────────────────────────────────
static void test_tokens()
{
    std::printf("=== tokens ===\n");

    CHECK(log_encode_bits_from_token("10") == 10, "\"10\" is 10 bit");
    CHECK(log_encode_bits_from_token("12") == 12, "\"12\" is 12 bit");

    // Exact match only: everything below must forward, not be guessed at.
    const char *not_values[] = { "", "0", "1", "8", "11", "14", "16", "010", "12.0",
                                 "12 ", " 12", "12abc", "twelve", "-10", "+12" };
    for (const char *t : not_values) {
        char msg[64];
        std::snprintf(msg, sizeof(msg), "\"%s\" is not a --log-encode value", t);
        CHECK(log_encode_bits_from_token(t) == 0, msg);
    }

    CHECK(log_encode_bits_supported(10), "10 bit supported");
    CHECK(log_encode_bits_supported(12), "12 bit supported");
    CHECK(!log_encode_bits_supported(16), "16 bit NOT supported (that was --keep16)");
    CHECK(!log_encode_bits_supported(11), "11 bit not supported");
    CHECK(!log_encode_bits_supported(8), "8 bit not supported");
    CHECK(!log_encode_bits_supported(0), "0 not supported");
    CHECK(kLogEncodeDefaultBits == 12, "bare flag defaults to 12 bit");
}

// ── Tier 2: one token at a time ──────────────────────────────────────────────
static void expect(const char *arg, const char *next, bool matched, int bits,
                   bool consumed, bool is_error, const char *what)
{
    const LogEncodeArg r = parse_log_encode_arg(arg, next);
    char msg[160];

    std::snprintf(msg, sizeof(msg), "%s: matched", what);
    CHECK(r.matched == matched, msg);
    std::snprintf(msg, sizeof(msg), "%s: target_bits == %d", what, bits);
    CHECK(r.target_bits == bits, msg);
    std::snprintf(msg, sizeof(msg), "%s: consumed_next == %d", what, (int)consumed);
    CHECK(r.consumed_next == consumed, msg);
    std::snprintf(msg, sizeof(msg), "%s: error %s", what, is_error ? "set" : "empty");
    CHECK(r.error.empty() != is_error, msg);
}

static void test_forms()
{
    std::printf("=== single-token forms ===\n");

    //     arg                 next        matched bits consumed error
    expect("--log-encode",     nullptr,    true,   12,  false,   false, "bare, end of argv");
    expect("--log-encode",     "--width",  true,   12,  false,   false, "bare, flag follows");
    expect("--log-encode",     "10",       true,   10,  true,    false, "space form 10");
    expect("--log-encode",     "12",       true,   12,  true,    false, "space form 12");
    expect("--log-encode=10",  nullptr,    true,   10,  false,   false, "equals form 10");
    expect("--log-encode=12",  "1920",     true,   12,  false,   false, "equals form 12");
    expect("--log-encode=",    "10",       true,   12,  false,   false, "empty equals is bare");

    // The gate case: an unknown value after the bare flag is NOT ours.
    expect("--log-encode",     "11",       true,   12,  false,   false, "space form 11 forwards");
    expect("--log-encode",     "16",       true,   12,  false,   false, "space form 16 forwards");
    expect("--log-encode",     "010",      true,   12,  false,   false, "space form 010 forwards");
    expect("--log-encode",     "-1",       true,   12,  false,   false, "space form -1 forwards");

    // ...but with '=' it is unambiguously ours and unambiguously wrong.
    expect("--log-encode=11",  nullptr,    true,   0,   false,   true,  "equals form 11 errors");
    expect("--log-encode=16",  nullptr,    true,   0,   false,   true,  "equals form 16 errors");
    expect("--log-encode=abc", nullptr,    true,   0,   false,   true,  "equals form abc errors");
    expect("--log-encode=010", nullptr,    true,   0,   false,   true,  "equals form 010 errors");

    // Not our flag at all.
    const char *others[] = { "--log-encodex", "--log-encode-foo", "--log", "-log-encode",
                             "log-encode", "--keep16", "--width", "1920", "", "-" };
    for (const char *a : others) {
        char msg[96];
        std::snprintf(msg, sizeof(msg), "\"%s\" is not our flag", a);
        const LogEncodeArg r = parse_log_encode_arg(a, "10");
        CHECK(!r.matched && r.target_bits == 0 && !r.consumed_next, msg);
    }

    // The error text must name the offending value, or it is useless at launch.
    const LogEncodeArg bad = parse_log_encode_arg("--log-encode=11", nullptr);
    CHECK(bad.error.find("11") != std::string::npos, "error text quotes the bad value");
    CHECK(bad.error.find("10 or 12") != std::string::npos, "error text lists valid values");
}

// ── Tier 3: a whole argv walk ────────────────────────────────────────────────
// Mirrors the CinePiOptions::Parse loop: matched tokens are pulled out (and may
// eat the next argv), everything else is forwarded to rpicam-apps.
struct WalkResult
{
    int log_encode = 0;
    std::vector<std::string> forwarded;
    std::string error;
};

static WalkResult walk(const std::vector<const char *> &argv)
{
    WalkResult out;
    const int argc = static_cast<int>(argv.size());
    for (int i = 1; i < argc; ++i) {
        const LogEncodeArg r =
            parse_log_encode_arg(argv[i], i + 1 < argc ? argv[i + 1] : nullptr);
        if (r.matched) {
            if (!r.error.empty()) { out.error = r.error; return out; }
            out.log_encode = r.target_bits;
            if (r.consumed_next)
                ++i;
            continue;
        }
        out.forwarded.push_back(argv[i]);
    }
    return out;
}

static void expect_walk(const std::vector<const char *> &argv, int bits,
                        const std::vector<std::string> &forwarded, const char *what)
{
    const WalkResult r = walk(argv);
    char msg[160];

    std::snprintf(msg, sizeof(msg), "%s: log_encode == %d", what, bits);
    CHECK(r.log_encode == bits, msg);
    std::snprintf(msg, sizeof(msg), "%s: forwards %zu token(s)", what, forwarded.size());
    CHECK(r.forwarded == forwarded, msg);
    std::snprintf(msg, sizeof(msg), "%s: no error", what);
    CHECK(r.error.empty(), msg);
}

static void test_walk()
{
    std::printf("=== argv walk ===\n");

    expect_walk({ "cinepi-raw", "--width", "1920", "--height", "1080" },
                0, { "--width", "1920", "--height", "1080" },
                "no flag leaves everything alone");

    expect_walk({ "cinepi-raw", "--log-encode", "10", "--width", "1920" },
                10, { "--width", "1920" },
                "value consumed, rest forwarded");

    expect_walk({ "cinepi-raw", "--log-encode=10", "--width", "1920" },
                10, { "--width", "1920" },
                "equals form, rest forwarded");

    expect_walk({ "cinepi-raw", "--width", "1920", "--log-encode" },
                12, { "--width", "1920" },
                "bare flag last");

    expect_walk({ "cinepi-raw", "--log-encode", "--width", "1920" },
                12, { "--width", "1920" },
                "bare flag does not eat the next FLAG");

    // The gate case, end to end: "11" is not swallowed, it forwards.
    expect_walk({ "cinepi-raw", "--log-encode", "11" },
                12, { "11" },
                "bare flag does not eat an unknown VALUE");

    expect_walk({ "cinepi-raw", "--log-encode", "10", "--log-encode", "12" },
                12, {},
                "last occurrence wins");

    const WalkResult bad = walk({ "cinepi-raw", "--log-encode=11", "--width", "1920" });
    CHECK(!bad.error.empty(), "walk stops on an invalid equals value");
    CHECK(bad.log_encode == 0, "walk leaves log_encode off on error");
}

// ── Tier 4: --keep16 is really gone ──────────────────────────────────────────
// It was removed in the same commit that added --log-encode. The walk must no
// longer recognise it — which means it forwards to rpicam-apps and gets a proper
// "unrecognised option" instead of being silently accepted, and, critically,
// that "true" after it is no longer swallowed.
static void test_keep16_removed()
{
    std::printf("=== keep16 removed ===\n");

    expect_walk({ "cinepi-raw", "--keep16" },
                0, { "--keep16" },
                "--keep16 forwards");

    expect_walk({ "cinepi-raw", "--keep16", "true" },
                0, { "--keep16", "true" },
                "--keep16 no longer swallows its value");

    expect_walk({ "cinepi-raw", "--keep16=true", "--width", "1920" },
                0, { "--keep16=true", "--width", "1920" },
                "--keep16=true forwards");
}

int main()
{
    std::printf("=== log_encode_arg unit tests ===\n");
    test_tokens();
    test_forms();
    test_walk();
    test_keep16_removed();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    if (g_failures == 0) std::printf("ALL TESTS PASSED\n");
    return g_failures == 0 ? 0 : 1;
}
