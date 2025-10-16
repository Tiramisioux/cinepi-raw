#include "dng_encoder.hpp"
#include "raw_options.hpp"

#include <cassert>
#include <string>

// The real getHwId() implementation lives in utils.cpp and queries the
// running system for hardware details.  Pulling that object file (and the
// transitive libcamera dependencies) into this tiny regression test adds a lot
// of weight and still fails in the trimmed-down CI environment.  Provide a
// lightweight stub instead so the DngEncoder constructor can link while the
// test focuses on exercising the pre-configuration guard we added to
// buffer_full().
std::string getHwId()
{
    return "TEST-HW";
}

int main()
{
    RawOptions options;
    DngEncoder encoder(&options);

    assert(!encoder.initialized());
    assert(!encoder.buffer_full());

    return 0;
}