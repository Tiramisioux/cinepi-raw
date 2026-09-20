#pragma once
/* ------------------------------------------------------------------ */
/*  Simple TIFF/IFD builder – revised so that data offsets are        */
/*  calculated only once we know the final directory size.            */
/* ------------------------------------------------------------------ */
#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <algorithm>
#include <optional>

/* ------------------- DNG crop geometry (WP-CPR-3) ------------------
 *
 * Finding C4: dng_encoder.cpp writes tags 256/257 (ImageWidth/Length) as
 * the raw stream's own transport size and nothing else -- no
 * DefaultCropOrigin (0xC61F), DefaultCropSize (0xC620) or ActiveArea
 * (0xC68D). Every mode whose delivered buffer carries optical-black
 * padding around the real picture (pre-existing for the 3840x2200 RAW16
 * ClearHDR mode, and every padded crop the aspect-ratio family adds)
 * therefore shows that padding as picture in any DNG reader.
 *
 * computeDngCropRect() decides the fix: given the transport size already
 * written under tags 256/257, and the active picture size WP-CPR-2's
 * driver metadata helper (core/driver_mode_metadata.hpp) reports --
 * OUTPUT-domain, i.e. the caller has already divided crop_width/
 * crop_height by the driver's linear "Mode Binning" -- it returns the
 * crop rectangle to write, or `present == false` when none should be
 * written at all.
 *
 * WHY CENTRING, AND NOT THE DRIVER'S crop_left/crop_top (rework note,
 * WP-CPR-3 review round 2, finding C4-followup; narrowed in round 3,
 * finding C4-followup-2 -- see below):
 *
 * DriverModeMetadata.crop_left/crop_top (core/driver_mode_metadata.hpp)
 * look like the obvious "real" origin to thread through here instead of
 * guessing by centring -- WP-CPR-2 already probes them and cinepi_raw.cpp
 * has them in scope right next to the setActivePictureSize() call. They
 * are NOT usable for that, because they answer a different question.
 * Per WP-585-1 (WORK-PACKAGES.md) those fields are the sensor's readout
 * WINDOW position in native pixel-array coordinates -- what libcamera
 * needs to compute ScalerCrop/zoom and what `--list-cameras` prints as
 * e.g. `(480, 0)/2880x2160` for the 1440x1080 2x2 window (WP-585-1's own
 * worked example, also the imx585 experimental-crop-modes case this
 * package's own open_questions #2 flagged). That number is non-zero
 * precisely for a WINDOWED mode -- one reading fewer than the full
 * 3840x2160 array -- and is 0,0 for every full-field mode, INCLUDING the
 * one mode this campaign ships today (3840x2200 RAW16 ClearHDR): its
 * crop.top is 0 per WP-585-1's own invariant table ("non-windowed entries
 * keep the full active area"), even though the delivered buffer's real
 * picture starts 20 rows in. Using crop_top as the DNG origin would
 * therefore write origin_y = 0 for that shipped mode -- wrong, and a
 * regression against the tested, Pi-gated behaviour (test_padded_raw16_
 * crop_tags below expects 20). The two origins are unrelated: crop_left/
 * crop_top locate the readout window ON THE SENSOR; DefaultCropOrigin
 * locates the active picture WITHIN THIS FRAME'S OWN DELIVERED BUFFER.
 *
 * round 2's rework stopped there and drew the wrong conclusion: it made
 * crop_left/crop_top != 0 (a WINDOWED readout) refuse the crop tags
 * outright, on the theory that a windowed mode's buffer-local padding
 * geometry was unestablished. It is established, and round 3 removes the
 * refusal: per ASPECT-RATIOS.md, the RAW16 OB-padding convention -- the
 * delivered buffer is `active + 40` rows, split 20/20 top and bottom --
 * is VERTICAL-ONLY and applies IDENTICALLY to every ratio row in every
 * table, windowed or not (imx585 1x1 table: "RAW16 advertised" is
 * `active + 40`; the 2x2-binned WINDOWED table: exactly the same
 * `output + 40` column, entry for entry). The padding is a property of
 * how RAW16 packs a frame into its own buffer, not of where that frame's
 * readout window sits on the physical sensor array -- so the
 * transport-vs-active centring formula below, which only ever reasons
 * about this frame's OWN buffer, is exactly as valid for a windowed mode
 * as for the full-field one. crop_left/crop_top remain unusable as the
 * DNG origin itself (that diagnosis stands) -- they are simply not
 * consulted here at all any more, for either purpose.
 *
 * The active picture IS assumed CENTRED in the delivered buffer: true for
 * every padded mode in this campaign, windowed or full-field alike
 * (imx585.c's own mode table crops the OB rows evenly regardless of
 * window position, see WP-585-* and ASPECT-RATIOS.md), and the only
 * geometry the transport-vs-active SIZE difference alone can imply
 * without a separate per-axis offset from the driver. No pixel data is
 * ever touched by this -- only which sub-rectangle of the buffer already
 * written is the real picture.
 *
 * `active_width`/`active_height` are std::nullopt for a stock sensor
 * (every one of them today: none expose the five named geometry
 * controls) or when the probe failed. Per the spec, that case is left
 * exactly as before this package -- tags simply absent -- rather than
 * writing a crop equal to the full frame; the two are equivalent to a
 * reader (DefaultCropOrigin/Size default to (0,0)/the full image when
 * absent) and "absent" is the smaller diff against every DNG this stack
 * wrote before today.
 *
 * A crop that would claim MORE picture than the delivered buffer holds is
 * refused outright (never written) rather than clamped: clamping would
 * silently hide a wrong metadata reading behind plausible-looking tags,
 * where refusing leaves the file exactly as it would have been with no
 * metadata at all -- the safe, already-allowed fallback. */
struct DngCropRect
{
    bool     present  = false;
    uint32_t origin_x = 0;
    uint32_t origin_y = 0;
    uint32_t width    = 0;
    uint32_t height   = 0;
};

inline DngCropRect computeDngCropRect(uint32_t transport_width, uint32_t transport_height,
                                       std::optional<uint32_t> active_width,
                                       std::optional<uint32_t> active_height)
{
    DngCropRect r;

    if (!active_width || !active_height || *active_width == 0 || *active_height == 0)
        return r;   /* no driver metadata: tags absent, exactly as today */

    if (*active_width > transport_width || *active_height > transport_height)
        return r;   /* refused: would claim more than the delivered frame */

    if (*active_width == transport_width && *active_height == transport_height)
        return r;   /* no padding: crop == full frame, same bytes as today */

    r.width    = *active_width;
    r.height   = *active_height;
    r.origin_x = (transport_width  - *active_width)  / 2;
    r.origin_y = (transport_height - *active_height) / 2;
    r.present  = true;
    return r;
}

/* --------------------------- Memory writer ------------------------ */
struct MemoryBuffer
{
    uint8_t *buffer{};
    uint32_t offset{};      /* current write position                */
    uint32_t usedSize{};    /* highest written byte                  */
    uint32_t totalSize{};   /* size of the backing allocation        */
};

inline void write_pod(MemoryBuffer &memBuf, const void *src, size_t len)
{
    if (memBuf.offset + len > memBuf.totalSize)
        throw std::runtime_error("MemoryBuffer overflow");

    std::memcpy(memBuf.buffer + memBuf.offset, src, len);
    memBuf.offset += static_cast<uint32_t>(len);
    memBuf.usedSize = std::max(memBuf.usedSize, memBuf.offset);
}

inline void write_uint16(MemoryBuffer &memBuf, uint16_t v)
{
    write_pod(memBuf, &v, sizeof(v));
}

inline void write_uint32(MemoryBuffer &memBuf, uint32_t v)
{
    write_pod(memBuf, &v, sizeof(v));
}

/* ---------------------------- TIFF types -------------------------- */
enum TIFFType : uint16_t
{
    TIFF_BYTE      = 1,
    TIFF_ASCII     = 2,
    TIFF_SHORT     = 3,
    TIFF_LONG      = 4,
    TIFF_RATIONAL  = 5,
    TIFF_UNDEFINED = 7,
    TIFF_SSHORT    = 8,
    TIFF_SLONG     = 9,
    TIFF_SRATIONAL = 10,
};

/* how big is one element of each TIFF type? (0 → impossible) */
inline constexpr size_t tiffUnit(TIFFType t)
{
    switch (t)
    {
    case TIFF_BYTE:
    case TIFF_ASCII:
    case TIFF_UNDEFINED:             return 1;
    case TIFF_SHORT:
    case TIFF_SSHORT:                return 2;
    case TIFF_LONG:
    case TIFF_SLONG:                 return 4;
    case TIFF_RATIONAL:
    case TIFF_SRATIONAL:             return 8;
    default:                         return 0;
    }
}

/* ----------------------------- IFD entry -------------------------- */
#pragma pack(push, 1)
struct IFDEntry
{
    uint16_t tag;
    uint16_t type;
    uint32_t count;
    uint32_t value;          /* inline value OR offset into extra area        */
};
#pragma pack(pop)

/* --------------------------- IFD builder -------------------------- */
class IFDBuilder
{
    /* one *pending* entry with its payload (copied)                  */
    struct Pending
    {
        uint16_t      tag;
        TIFFType      type;
        uint32_t      count;
        std::vector<uint8_t> data;        /* may be empty                */
    };

public:
    /* No geometry ctor on purpose. This builds a TIFF directory out of the
     * entries you add and nothing else -- image dimensions are the caller's
     * to state, as tags 256/257, which every caller already does (see
     * dng_encoder.cpp's IFD0 and add_thumbnail_ifd1_entries() for IFD1).
     * An earlier signature took width/height and stored them in fields no
     * member ever read; they were dead from this file's first commit and
     * clang flagged them (-Wunused-private-field). Having build() emit
     * 256/257 from them instead would have written those tags twice. */

    /* add one tag – payload is copied immediately into `data`         */
    void addEntry(uint16_t tag,
                  TIFFType type,
                  uint32_t count,
                  const void *ptr = nullptr)
    {
        Pending p{};
        p.tag   = tag;
        p.type  = type;
        p.count = count;

        const size_t len = tiffUnit(type) * count;
        if (ptr && len)
            p.data.assign(reinterpret_cast<const uint8_t *>(ptr),
                          reinterpret_cast<const uint8_t *>(ptr) + len);

        entries_.push_back(std::move(p));
    }

    /* Sort entries if you really want ascending tag order             */
    void sortEntries()
    {
        std::sort(entries_.begin(), entries_.end(),
                  [](const Pending &a, const Pending &b)
                  {
                      return a.tag < b.tag;
                  });
    }

    /* materialise the directory + data into the MemoryBuffer          */
    void build(MemoryBuffer &memBuf)
    {
        const uint16_t n = static_cast<uint16_t>(entries_.size());

        /* remember where the directory starts in the file             */
        const uint32_t dirStart = memBuf.offset;

        /* we’ll fill the count field now, but skip the directory body
           until we have computed every value/offset                   */
        write_uint16(memBuf, n);
        memBuf.offset += n * sizeof(IFDEntry);   /* 12 bytes each      */
        /* Remember where the next-IFD field landed so a caller chaining
           a second IFD after this one can patch it once that IFD's own
           baseOffset is known -- same after-the-fact pattern already used
           to patch the TIFF header's IFD-0 offset. Left at 0 (no next
           IFD) unless a caller does that patch. */
        nextIfdFieldOffset = memBuf.offset;
        write_uint32(memBuf, 0);                 /* next-IFD = 0       */

        /* where does the extra area begin?                            */
        uint32_t extraCursor =
            dirStart + 2 + n * sizeof(IFDEntry) + 4;

        /* we keep the finished directory here before writing it out   */
        std::vector<IFDEntry> finalDir;
        finalDir.reserve(n);

        std::vector<uint8_t> extraData;

        for (const Pending &p : entries_)
        {
            const size_t len = tiffUnit(p.type) * p.count;

            IFDEntry e{};
            e.tag   = p.tag;
            e.type  = p.type;
            e.count = p.count;

            if (len == 0)                    /* no payload             */
            {
                e.value = 0;
            }
            /* --- inside IFDBuilder::build()  (inline-value branch) ----------- */
            else if (len <= 4)                 // payload fits in the value field
            {
                e.value = 0;                   // clear the 32-bit cell
                std::memcpy(&e.value, p.data.data(), len);
            }
            else                             /* store in extra area    */
            {
                /* 4-byte align                                          */
                if (extraCursor & 3)
                {
                    const uint32_t pad = 4 - (extraCursor & 3);
                    extraData.insert(extraData.end(), pad, 0);
                    extraCursor += pad;
                }

                e.value = extraCursor;

                extraData.insert(extraData.end(),
                                 p.data.begin(), p.data.end());
                extraCursor += static_cast<uint32_t>(len);

                /* pad to next 4-byte boundary                          */
                if (extraCursor & 3)
                {
                    const uint32_t pad = 4 - (extraCursor & 3);
                    extraData.insert(extraData.end(), pad, 0);
                    extraCursor += pad;
                }
            }

            finalDir.push_back(e);
        }

        /* ----------------------------------------------------------------
           Now that every entry is ready, go back and write the directory   */
        const uint32_t saveOffset = memBuf.offset;       /* remember pos.  */
        memBuf.offset = dirStart + 2;                    /* after count    */

        for (const IFDEntry &e : finalDir)
            write_pod(memBuf, &e, sizeof(e));

        /* skip the “next IFD” field we pre-allocated                       */
        memBuf.offset = saveOffset;

        /* finally append the extra data                                   */
        if (!extraData.empty())
            write_pod(memBuf, extraData.data(), extraData.size());
    }

    uint32_t baseOffset{0};  /* for callers who still want to patch it */
    uint32_t nextIfdFieldOffset{0};  /* set by build(); see above */

private:
    std::vector<Pending> entries_;
};
