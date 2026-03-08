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

struct RawOptions : public VideoOptions
{
    enum class RecordingPerfMode
    {
        Off,
        Balanced,
        Max,
    };

    static const char *RecordingPerfModeToString(RecordingPerfMode mode)
    {
        switch (mode)
        {
        case RecordingPerfMode::Off: return "off";
        case RecordingPerfMode::Balanced: return "balanced";
        case RecordingPerfMode::Max: return "max";
        }
        return "balanced";
    }

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
    uint32_t encode_workers { 1 };
    uint32_t disk_workers   { 1 };
    std::optional<std::vector<int>> encode_affinity;
    std::optional<std::vector<int>> disk_affinity;
    std::optional<int> encode_nice;
    std::optional<int> disk_nice;

    /* observability / logging tuning */
    uint32_t latency_sample_interval { 20 };
    bool per_frame_logs { false };

    /* recording hot-path CPU/jitter tuning */
    RecordingPerfMode recording_perf_mode { RecordingPerfMode::Balanced };
};
