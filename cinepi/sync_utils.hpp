#pragma once

#include <cstdint>
#include "sync_policy.hpp"

constexpr inline bool should_sync_frame(RawSyncPolicy policy,
                                        std::uint32_t interval,
                                        std::uint64_t frame_number) noexcept
{
    return (policy == RawSyncPolicy::Interval) &&
           (interval != 0) &&
           (frame_number % interval == 0);
}
