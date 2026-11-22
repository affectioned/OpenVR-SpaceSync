// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-11-22. Part of SpaceSync, a modified version
// of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md
//
// Lightweight perf counter: ring buffer of recent sample durations,
// cheap to Push to, cheap to Avg / P95 / Last. Header-only, no deps
// outside the standard library + Windows QueryPerformanceCounter for
// the timing helpers.

#pragma once

#include <windows.h>
#include <algorithm>
#include <cstdint>
#include <cstddef>

namespace perf {

// Convert QPC ticks to milliseconds using a cached frequency.
inline double TicksToMs(LONGLONG ticks)
{
    static LARGE_INTEGER freq = [] { LARGE_INTEGER f; QueryPerformanceFrequency(&f); return f; }();
    return (double)ticks * 1000.0 / (double)freq.QuadPart;
}

inline LONGLONG NowTicks()
{
    LARGE_INTEGER t;
    QueryPerformanceCounter(&t);
    return t.QuadPart;
}

// Ring buffer of recent millisecond samples. N controls window length.
template <size_t N>
struct Counter
{
    double buf[N] = {};
    size_t head = 0;
    size_t filled = 0;
    double lastMs = 0.0;

    void Push(double ms)
    {
        buf[head] = ms;
        head = (head + 1) % N;
        if (filled < N) filled++;
        lastMs = ms;
    }

    double Avg() const
    {
        if (filled == 0) return 0.0;
        double s = 0;
        for (size_t i = 0; i < filled; i++) s += buf[i];
        return s / (double)filled;
    }

    double P95() const
    {
        if (filled == 0) return 0.0;
        double sorted[N];
        for (size_t i = 0; i < filled; i++) sorted[i] = buf[i];
        std::sort(sorted, sorted + filled);
        size_t idx = (size_t)((double)filled * 0.95);
        if (idx >= filled) idx = filled - 1;
        return sorted[idx];
    }

    double Last() const { return lastMs; }
    size_t Count() const { return filled; }
};

// RAII scope guard that pushes elapsed ms into a counter when destroyed.
template <size_t N>
struct Scope
{
    Counter<N>& c;
    LONGLONG start;
    explicit Scope(Counter<N>& cc) : c(cc), start(NowTicks()) {}
    ~Scope() { c.Push(TicksToMs(NowTicks() - start)); }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
};

} // namespace perf
