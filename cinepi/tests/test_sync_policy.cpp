#include "sync_utils.hpp"
#include "sync_policy.hpp"

#include <cassert>

int main()
{
    assert(!should_sync_frame(RawSyncPolicy::Never, 0, 1));
    assert(!should_sync_frame(RawSyncPolicy::Take, 0, 10));
    assert(!should_sync_frame(RawSyncPolicy::Interval, 0, 5));
    assert( should_sync_frame(RawSyncPolicy::Interval, 3, 3));
    assert(!should_sync_frame(RawSyncPolicy::Interval, 3, 4));
    assert( should_sync_frame(RawSyncPolicy::Interval, 1, 7));
    return 0;
}
