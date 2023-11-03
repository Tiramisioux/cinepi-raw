/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * still_video.hpp - video capture program options
 */

#pragma once

#include <cstdio>

#include <string>

#include "core/video_options.hpp"

struct RawOptions : public VideoOptions
{   
    RawOptions() : VideoOptions()
	{
		using namespace boost::program_options;
		options_.add_options();
	}

	std::optional<std::string> redis;

	uint32_t clip_number;
	std::string mediaDest;
	std::string folder;

	bool awbEn;
	int compression;

	int thumbnail;
	int thumbnailSize;

	uint16_t rawCrop[4];

	uint8_t mic_gain;

	float wb;
	std::string sensor;
	std::string model;
	std::string make;
	std::optional<std::string> ucm;
	std::string serial;

	float clipping;

};
