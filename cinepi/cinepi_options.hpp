/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Extended CinePi options – now supports per-stream digital-zoom
 * via --scaler-crops x,y,w,h[:x,y,w,h...]
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
        float Zoom()  const { return zoom_factor; }
        void  SetZoom(float z) { zoom_factor = z; }
        bool  ZoomRaw() const { return zoom_raw; }   // <-- bring this back
        void  SetZoomRaw(bool v) { zoom_raw = v; }

        
        /* Vector of crop rectangles (fractions) in stream order.        */
        const std::vector<std::array<float,4>> &ScalerCrops() const
        { return scaler_crops_rects; }

private:
        /* CinePi-specific flags */
        std::string  camPort;
        bool         same_hdmi;
        bool         keep16;
        int          hdmi_port;
        std::vector<std::array<float,4>> scaler_crops_rects;    
        float zoom_factor { 1.0f };  
        bool         zoom_raw    { false };   
};
