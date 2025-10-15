#pragma once

#include "sync_policy.hpp"

inline bool should_sync_frame(RawSyncPolicy policy,
                              uint32_t interval,
                              uint64_t frame_number)
{
    if (policy == RawSyncPolicy::Interval && interval)
#include "raw_options.hpp"

inline bool should_sync_frame(RawOptions::SyncPolicy policy,
                              uint32_t interval,
                              uint64_t frame_number)
{
    if (policy == RawOptions::SyncPolicy::Interval && interval)
        return (frame_number % interval) == 0;
    return false;
}
