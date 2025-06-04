/* preview/hdmi_utils.cpp
 * ---------------------------------------------------------------------------
 * Translate a CinePi  “HDMI port index”  (0 = first socket, 1 = second …)
 * into the corresponding DRM connector‑id so the preview can be forced to
 * the correct screen.
 * ------------------------------------------------------------------------- */
#include <optional>
#include <string>

#include <xf86drm.h>
#include <xf86drmMode.h>

using std::optional;
using std::string;

optional<uint32_t> drm_connector_id_for_port(int port)   // −1 → use default
{
    if (port < 0)                         // user didn’t ask for anything
        return std::nullopt;

    int fd = drmOpen("vc4", nullptr);     // driver for Pi 4 / Pi 5
    if (fd < 0)
        return std::nullopt;

    optional<uint32_t> connector;         // value we’ll return
    int hdmi_index = 0;                   // count HDMIs as we iterate

    if (drmModeRes *res = drmModeGetResources(fd))
    {
        for (int i = 0; i < res->count_connectors; ++i)
        {
            drmModeConnector *con = drmModeGetConnector(fd, res->connectors[i]);
            if (!con)
                continue;

            bool is_hdmi =
                (con->connector_type == DRM_MODE_CONNECTOR_HDMIA) ||
                (con->connector_type == DRM_MODE_CONNECTOR_HDMIB);

            if (is_hdmi && hdmi_index++ == port)          // found the one we want
                connector = con->connector_id;

            drmModeFreeConnector(con);
            if (connector)                                // we’re done – break early
                break;
        }
        drmModeFreeResources(res);
    }

    drmClose(fd);
    return connector;                                     // maybe still empty
}
