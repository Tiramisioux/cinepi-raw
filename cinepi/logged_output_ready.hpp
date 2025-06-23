/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * logged_output_ready.hpp – wrap Output::OutputReady so that, *if* any of the
 *                           parameters are convertible to std::string_view
 *                           (typical for DNG/JPEG still pipelines), the file
 *                           name is printed to the console.  Otherwise we
 *                           stay silent and never break the build.
 *
 *   Usage:
 *      #include "logged_output_ready.hpp"
 *      …
 *      app.SetEncodeOutputReadyCallback(
 *              cinepi::make_logged_output_ready(output.get()));
 */

#pragma once

#include "output/output.hpp"
#include <spdlog/spdlog.h>
#include <string_view>
#include <type_traits>
#include <utility>

namespace cinepi {

/* --------------------------------------------------------------------- */
/*  Helper: forward everything to Output::OutputReady, then – *only if*   */
/*  one of the arguments is string-like – log that argument.              */
/* --------------------------------------------------------------------- */
template <typename OutputPtr>
auto make_logged_output_ready(OutputPtr *out)
{
        /* create only once even if the header is included in many TU’s */
        static auto log = spdlog::stdout_color_mt("encode_cb");

        return [out](auto &&...args) {
                /* 1. keep the original behaviour intact */
                out->OutputReady(std::forward<decltype(args)>(args)...);

                /* 2.  try to find a filename among the arguments      */
                bool printed = false;
                (void)std::initializer_list<int>{
                        ([&] {
                                /* “string-like” = convertible
                                   to std::string_view                 */
                                using ArgT = std::decay_t<decltype(args)>;
                                if constexpr (std::is_convertible_v<ArgT,
                                                                  std::string_view>)
                                {
                                        log->info("DNG written: {}", args);
                                        printed = true;
                                }
                        }(),
                         0)...};

                /* 3.  If no filename parameter exists we do nothing.
                       (Your pipeline is probably one that only passes
                        {void*, size_t, int64_t, bool}.)                */
                if (!printed)
                        return;
        };
}

} // namespace cinepi
