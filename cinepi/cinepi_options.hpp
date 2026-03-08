/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Extended CinePi options – now supports per-stream digital-zoom
 * via --scaler-crops x,y,w,h[:x,y,w,h...]
 * and separate RAW-stream zoom via --zoom-raw
 *
 * Fractions are 0-1 of the active sensor area; they are converted
 * to pixels later, once the camera mode is known.
 */

#pragma once

#include "raw_options.hpp"        // base class
#include <array>
#include <vector>
#include <string>

class CinePiOptions : public RawOptions
{
public:
    CinePiOptions();

    // parse CLI, returns false when only --help / --version was requested
    bool Parse(int argc, char *argv[]);

    /* ------------------------------------------------------------------
     * Accessors
     * ------------------------------------------------------------------ */
    const std::string &CamPort()        const { return camPort; }
    int                HdmiPort()       const { return hdmi_port; }
    bool               SameHdmi()       const { return same_hdmi; }
    bool               Keep16()         const { return keep16;   }

    // digital zoom for preview and low-res streams
    float Zoom()        const { return zoom_factor; }
    void  SetZoom(float z) { zoom_factor = z; }

    // separate flag to apply zoom crop to the RAW stream too
    bool  ZoomRaw()     const { return zoom_raw; }
    void  SetZoomRaw(bool v) { zoom_raw = v; }

    RawOptions::RecordingPerfMode RecordingPerfModeValue() const { return recording_perf_mode; }

    /* Vector of crop rectangles (fractions) in stream order.        */
    const std::vector<std::array<float,4>> &ScalerCrops() const
    { return scaler_crops_rects; }

private:
    /* CinePi-specific flags */
    std::string  camPort;                       // e.g. "cam0"
    bool         same_hdmi    = false;
    bool         keep16       = false;
    int          hdmi_port    = -1;

    std::vector<std::array<float,4>> scaler_crops_rects;

    // preview & lo-res zoom
    float zoom_factor = 1.0f;

    // raw-stream zoom (disable 12-bit packing)
    bool  zoom_raw     = false;
};
