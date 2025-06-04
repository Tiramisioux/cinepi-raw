#pragma once
#include <cstdint>
#include <optional>

/* Return the DRM connector‑ID for HDMI‑0 or HDMI‑1.
 * If the requested port does not exist (e.g. Pi Zero has only one),
 * std::nullopt is returned so the caller can fall back to libcamera’s default. */
std::optional<uint32_t> drm_connector_id_for_port(int hdmi_port);
