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
    RawOptions()
        : VideoOptions()
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

    /* CineMate Log target depth: 0 = off, else 10 or 12. Set by --log-encode in
     * CinePiOptions and read by the DNG encoder. Startup-only — the redis thread
     * mutates this struct live, and the encode path cannot swap curves mid-clip. */
    int log_encode { 0 };

    /* worker pool sizing + tuning */
    uint32_t encode_workers { 2 };
    uint32_t disk_workers   { 8 };
    std::optional<std::vector<int>> encode_affinity;
    std::optional<std::vector<int>> disk_affinity;
    std::optional<int> encode_nice;
    std::optional<int> disk_nice;

    int plain_arecord_timecode_offset_frames { 0 };
    int audio_timecode_offset_frames { 0 };
};
