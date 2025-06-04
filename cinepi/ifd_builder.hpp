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
    explicit IFDBuilder(uint32_t width = 0, uint32_t height = 0)
        : w(width), h(height) {}

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

private:
    uint32_t w{}, h{};
    std::vector<Pending> entries_;
};
