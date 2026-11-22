// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-11-22. Shared perf counters for the overlay;
// the counters themselves live in PerfCounter.h, this file ties specific
// hot paths to window-sized buffers and exposes them for the UI.

#pragma once

#include "PerfCounter.h"

namespace perf {

// Overlay-side counters. Ring size chosen to hold ~1 s of samples at the
// relevant cadence:
//   - MainFrame     : main loop runs at ~90 Hz -> 128
//   - CalibrationTick: once per ~50 ms gate   -> 32
//   - Ipc tick (GetDriverStatus etc)           -> 32
//   - Kabsch (one-shot, during calibration)   -> 8
//   - Continuous recal solve (periodic)       -> 8
//   - OverlayLog I/O                           -> 32
struct State
{
    Counter<128> mainFrame;
    Counter<32>  calibrationTick;
    Counter<32>  ipcTick;
    Counter<8>   lastKabsch;
    Counter<8>   lastContRecal;
    Counter<32>  overlayLog;
};

State& Global();

} // namespace perf
