/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * log_lut.cpp - loading CineMate Log curve specs from resources/log_luts.
 *
 * Split from log_lut.hpp so the pure curve math stays dependency-free and
 * tests/log_lut_test.cpp can build with nothing but the standard library. This
 * half owns the jsoncpp parse, the search path, and the cross-check that the
 * rebuilt inverse table still matches the generator's.
 */

#include "cinepi/log_lut.hpp"

#include <json/json.h>

#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <utility>

namespace
{

/* Searched in order; first readable spec wins. The env override is for testing a
 * regenerated curve without reinstalling. The datadir entries are where
 * meson.build installs the shipped specs; the /home/pi entry covers running an
 * uninstalled build straight out of the Pi's repo clone. */
const char *kSpecDirEnv = "CINEPI_LOG_LUT_DIR";
const char *const kSpecDirs[] = {
    "/usr/local/share/cinepi-raw/log_luts",
    "/usr/share/cinepi-raw/log_luts",
    "/home/pi/cinepi-raw/resources/log_luts",
    "resources/log_luts",
};

bool readable(const std::string &path)
{
    std::ifstream f(path);
    return f.good();
}

} // namespace

std::string log_lut_spec_filename(int source_bits, int target_bits)
{
    return "cinemate_log_" + std::to_string(source_bits) + "to" + std::to_string(target_bits) + ".json";
}

std::string find_log_lut_spec(int source_bits, int target_bits)
{
    const std::string name = log_lut_spec_filename(source_bits, target_bits);

    if (const char *env = std::getenv(kSpecDirEnv))
    {
        const std::string p = std::string(env) + "/" + name;
        if (readable(p))
            return p;
    }
    for (const char *dir : kSpecDirs)
    {
        const std::string p = std::string(dir) + "/" + name;
        if (readable(p))
            return p;
    }
    return {};
}

bool load_log_lut_spec(const std::string &path, LogLutParams &params,
                       std::vector<uint16_t> *table, std::string &err)
{
    std::ifstream f(path);
    if (!f.good())
    {
        err = "cannot open log LUT spec '" + path + "'";
        return false;
    }

    Json::Value root;
    Json::CharReaderBuilder builder;
    std::string parse_err;
    if (!Json::parseFromStream(builder, f, &root, &parse_err))
    {
        err = "malformed log LUT spec '" + path + "': " + parse_err;
        return false;
    }

    const Json::Value &p = root["params"];
    if (!p.isObject())
    {
        err = "log LUT spec '" + path + "' has no params object";
        return false;
    }

    LogLutParams out;
    out.mu = p.get("mu", 0.0).asDouble();
    out.black_level = p.get("black_level", -1).asInt();
    out.white_level = p.get("white_level", -1).asInt();
    out.source_bits = p.get("source_bits", 0).asInt();
    out.target_bits = p.get("target_bits", 0).asInt();
    out.footroom_codes = p.get("footroom_codes", 0).asInt();
    out.footroom_lsb = p.get("footroom_lsb", 0).asInt();

    if (!out.valid())
    {
        err = "log LUT spec '" + path + "' has invalid params: " + out.describe();
        return false;
    }
    params = out;

    if (table)
    {
        table->clear();
        const Json::Value &t = root["linearization_table"];
        if (t.isArray())
        {
            table->reserve(t.size());
            for (const Json::Value &v : t)
                table->push_back(static_cast<uint16_t>(v.asUInt()));
        }
    }
    return true;
}

bool load_log_lut(int source_bits, int target_bits, LogLut &lut, std::string &err)
{
    const std::string path = find_log_lut_spec(source_bits, target_bits);
    if (path.empty())
    {
        err = "no log LUT spec " + log_lut_spec_filename(source_bits, target_bits) +
              " found (set " + kSpecDirEnv + " to override the search path)";
        return false;
    }

    LogLutParams params;
    std::vector<uint16_t> spec_table;
    if (!load_log_lut_spec(path, params, &spec_table, err))
        return false;

    if (!lut.build(params))
    {
        err = "cannot build log LUT from '" + path + "'";
        return false;
    }

    /* The spec ships the generator's own table. If ours differs, the C++ curve has
     * drifted from resources/log_luts/gen_cinemate_log.py — the DNG would carry a
     * LinearizationTable that does not invert its own pixels. Refuse to load. */
    if (!spec_table.empty())
    {
        if (spec_table.size() != lut.inverse_size())
        {
            std::ostringstream m;
            m << "log LUT spec '" << path << "' table has " << spec_table.size()
              << " entries, expected " << lut.inverse_size();
            err = m.str();
            return false;
        }
        for (size_t i = 0; i < spec_table.size(); ++i)
        {
            if (spec_table[i] != lut.inverse()[i])
            {
                std::ostringstream m;
                m << "log LUT spec '" << path << "' disagrees with the built curve at code " << i
                  << ": spec " << spec_table[i] << " vs built " << lut.inverse()[i];
                err = m.str();
                return false;
            }
        }
    }
    return true;
}

/* ── the process-wide cache ─────────────────────────────────────────────────── */

const int kLogLutSourceBits[2] = { 16, 12 };

namespace
{

struct CacheEntry
{
    LogLut lut;
    bool ok = false;
    std::string err;
};

std::mutex g_cache_mutex;
/* std::map, not unordered_map: node-based, so a reference handed out earlier
 * stays valid when a later pair is inserted. */
std::map<std::pair<int, int>, CacheEntry> g_cache;

} // namespace

const LogLut *get_log_lut(int source_bits, int target_bits, std::string &err)
{
    const std::pair<int, int> key(source_bits, target_bits);

    /* Held across the build (a few ms, first use only) so two encode workers
     * racing on the same pair cannot both build it. */
    std::lock_guard<std::mutex> lock(g_cache_mutex);

    auto it = g_cache.find(key);
    if (it == g_cache.end())
    {
        CacheEntry entry;
        entry.ok = load_log_lut(source_bits, target_bits, entry.lut, entry.err);
        it = g_cache.emplace(key, std::move(entry)).first;
    }

    if (!it->second.ok)
    {
        err = it->second.err;
        return nullptr;
    }
    return &it->second.lut;
}

int preload_log_luts(int target_bits, std::string &summary)
{
    std::ostringstream out;
    std::string first_err;
    int usable = 0;

    for (int source_bits : kLogLutSourceBits)
    {
        std::string err;
        const LogLut *lut = get_log_lut(source_bits, target_bits, err);
        if (!lut)
        {
            if (first_err.empty())
                first_err = err;
            continue;
        }
        out << (usable++ ? " | " : "") << lut->params().describe()
            << " table=" << lut->inverse_size();
    }

    if (!usable)
    {
        std::ostringstream m;
        m << "no usable CineMate Log spec for target " << target_bits << " bit";
        if (!first_err.empty())
            m << " (" << first_err << ")";
        summary = m.str();
        return 0;
    }

    summary = out.str();
    return usable;
}
