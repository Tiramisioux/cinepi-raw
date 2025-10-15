/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * raw_options.hpp – common run-time options for cinepi-raw
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "core/video_options.hpp"
#include "sync_policy.hpp"

struct RawOptions : public VideoOptions
{
    RawOptions()
        : VideoOptions()
        , keep16(false)                     // NEW → default = pack to 12-bit
    {
        using namespace boost::program_options;
        options_.add_options();
    }

    /* ─── generic / redis ─────────────────────────────────────── */
    std::optional<std::string> redis;

    /* ─── clip organisation ───────────────────────────────────── */
    uint32_t       clip_number{};
    std::string    mediaDest;
    std::string    folder;

    /* sensor-local port name (“cam0” / “cam1”) – NEW */
    std::string    camPort;                 // filled by CinePiProcess

    /* ─── capture parameters ──────────────────────────────────── */
    bool        awbEn{};
    int         compression{};
    int         thumbnail{};
    int         thumbnailSize{};
    uint16_t    rawCrop[4]{};

    uint8_t     mic_gain{};

    float       wb{};
    std::string sensor;
    std::string model;
    std::string make;
    std::optional<std::string> ucm;
    std::string serial;

    float       clipping{};

    /* keep full 16-bit raw instead of packing – NEW */
    bool keep16;                           // set by --keep16 in CinePiOptions

    /* worker pool sizing + tuning */
    using SyncPolicy = RawSyncPolicy;

    uint32_t encode_workers { 4 };
    uint32_t disk_workers   { 2 };
    std::optional<std::vector<int>> encode_affinity;
    std::optional<std::vector<int>> disk_affinity;
    std::optional<int> encode_nice;
    std::optional<int> disk_nice;

    /* startup gating ---------------------------------------------------- */
    uint32_t preroll_ms          { 300 };
    uint32_t start_queue_frames  { 6 };
    uint32_t ignore_start_frames { 12 };

    /* disk synchronisation ---------------------------------------------- */
    SyncPolicy sync_policy { SyncPolicy::Never };
    uint32_t   sync_interval { 0 };      // used when policy == Interval
    bool       drop_cache_after_close { false };

    /* diagnostics / maintenance ---------------------------------------- */
    bool       selftest { false };
    uint32_t   selftest_seconds { 1 };   // duration for --selftest mode
};
