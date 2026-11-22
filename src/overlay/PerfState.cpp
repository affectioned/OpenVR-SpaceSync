// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-11-22.

#include "PerfState.h"

namespace perf {

State& Global()
{
    static State g;
    return g;
}

} // namespace perf
