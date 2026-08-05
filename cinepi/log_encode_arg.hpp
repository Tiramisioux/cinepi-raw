/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * log_encode_arg.hpp - parsing the --log-encode command-line token.
 *
 * Pure string handling: no libcamera, no boost, no spdlog — so
 * tests/log_encode_arg_test.cpp exercises the *same* code the option walk runs.
 *
 * CinePiOptions::Parse pulls CinePi flags out of argv by hand; the boost
 * add_options() block exists for `-h` text only and never sees them, so boost's
 * implicit_value() cannot supply the bare form's default. It is written here.
 *
 *   --log-encode        -> target 12 (the default), next argv untouched
 *   --log-encode 10     -> target 10, next argv consumed
 *   --log-encode=10     -> target 10
 *   --log-encode 11     -> target 12, "11" NOT consumed — it forwards on
 *   --log-encode=11     -> error
 *
 * The space form and the '=' form differ on a bad value on purpose. A bare flag
 * can only coexist with a following positional if the lookahead consumes the
 * next token *only when it is a known value*; anything else has to be assumed
 * to belong to someone else (this is the pattern --keep16 used). With '=' there
 * is no ambiguity, so a bad value is a hard error instead of a silent fall-back
 * to the default — a typo'd --log-encode=11 must not quietly record 12-bit.
 *
 * Values are matched as exact strings, not parsed as numbers: "010", "12 " and
 * "12abc" are not the flag's values and must forward rather than be guessed at.
 */

#ifndef CINEPI_LOG_ENCODE_ARG_HPP
#define CINEPI_LOG_ENCODE_ARG_HPP

#include <string>

/* Target depth the bare flag means. */
constexpr int kLogEncodeDefaultBits = 12;

/* Depths the flag accepts. 16 is deliberately absent: a 16-bit target would mean
 * "no companding", which is what the removed --keep16 did. */
inline bool log_encode_bits_supported(int bits)
{
    return bits == 10 || bits == 12;
}

/* Exact-match token -> target bits, or 0 if the token is not one of ours. */
inline int log_encode_bits_from_token(const std::string &token)
{
    if (token == "10")
        return 10;
    if (token == "12")
        return 12;
    return 0;
}

struct LogEncodeArg
{
    bool matched = false;        /* the token is --log-encode in some form       */
    bool consumed_next = false;  /* the following argv was eaten as the value    */
    int target_bits = 0;         /* resolved target; 0 only when !matched or err */
    std::string error;           /* non-empty -> caller must reject the argv     */
};

/* `next` is argv[i+1] or nullptr at the end of argv. Never reads past it. */
inline LogEncodeArg parse_log_encode_arg(const std::string &arg, const char *next)
{
    static const std::string kFlag = "--log-encode";
    static const std::string kFlagEq = "--log-encode=";

    LogEncodeArg out;

    if (arg.rfind(kFlagEq, 0) == 0)
    {
        out.matched = true;
        const std::string value = arg.substr(kFlagEq.size());
        if (value.empty())          /* "--log-encode=" — as good as bare */
        {
            out.target_bits = kLogEncodeDefaultBits;
            return out;
        }
        const int bits = log_encode_bits_from_token(value);
        if (!log_encode_bits_supported(bits))
        {
            out.error = "unsupported --log-encode value '" + value + "' (expected 10 or 12)";
            return out;
        }
        out.target_bits = bits;
        return out;
    }

    if (arg != kFlag)
        return out;                 /* not our flag — matched stays false */

    out.matched = true;
    out.target_bits = kLogEncodeDefaultBits;
    if (next)
    {
        const int bits = log_encode_bits_from_token(next);
        if (log_encode_bits_supported(bits))
        {
            out.target_bits = bits;
            out.consumed_next = true;
        }
    }
    return out;
}

#endif /* CINEPI_LOG_ENCODE_ARG_HPP */
