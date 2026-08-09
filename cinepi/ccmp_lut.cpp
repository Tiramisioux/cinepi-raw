/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * ccmp_lut.cpp - the process-wide CCMP decompand table cache.
 *
 * Split from ccmp_lut.hpp so the curve math stays dependency-free and
 * tests/ccmp_lut_test.cpp can build with nothing but the standard library.
 * There is no spec file to load — unlike the CineMate Log curve, this one is
 * fully determined by the register readbacks and the two measured anchors in
 * the header, so there is nothing here but the cache.
 */

#include "cinepi/ccmp_lut.hpp"

#include <map>
#include <mutex>
#include <sstream>
#include <utility>

namespace
{

struct CacheEntry
{
    CcmpLut lut;
    bool ok = false;
    std::string err;
};

std::mutex g_cache_mutex;
/* std::map, not unordered_map: node-based, so a pointer handed out earlier stays
 * valid when a later binning factor is inserted. */
std::map<double, CacheEntry> g_cache;

} // namespace

const CcmpLut *get_ccmp_lut(double binning, std::string &err)
{
    /* Held across the build (well under a millisecond, first use only) so two
     * encode workers racing on the same binning cannot both build it. */
    std::lock_guard<std::mutex> lock(g_cache_mutex);

    auto it = g_cache.find(binning);
    if (it == g_cache.end())
    {
        CacheEntry entry;
        CcmpParams params;
        if (!ccmp_params_for_binning(binning, params))
        {
            /* No measured anchor for this binning. That is an unvalidated mode,
             * not a missing convenience: the register curve would build happily
             * and be wrong by 21 L through the mid-tones, which looks entirely
             * plausible in a render. Refuse, and let the caller fall back to the
             * ordinary linear path rather than emit a mislabelled file. */
            std::ostringstream m;
            m << "no measured CCMP anchor for binning " << binning
              << " — only the two validated 12-bit ClearHDR modes have one";
            entry.err = m.str();
        }
        else
        {
            entry.ok = entry.lut.build(params, &entry.err);
        }
        it = g_cache.emplace(binning, std::move(entry)).first;
    }

    if (!it->second.ok)
    {
        err = it->second.err;
        return nullptr;
    }
    return &it->second.lut;
}
