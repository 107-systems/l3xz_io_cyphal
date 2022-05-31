#pragma once

#include <cstdlib>
#include <cmath>
#include <cassert>

/// In C++20 this should accept std::span<> instead.
template <typename T>
inline T interpolatePiecewise(const float ref, const T* samples, const std::size_t size)
{
    const auto back = size - 1U;
    const auto scaled = std::clamp<float>(ref * back, 0.0F, back);
    const auto ilo = std::clamp<std::int64_t>(static_cast<std::int64_t>(std::floor(scaled)), 0, back);
    const auto ihi = std::clamp<std::int64_t>(static_cast<std::int64_t>(std::ceil(scaled)), 0, back);
    if (ihi == ilo)  // edge case
    {
        return samples[ilo];
    }
    const T& lo = samples[ilo];
    const T& hi = samples[ihi];
    return lo * (ihi - scaled) + hi * (scaled - ilo);
}

/// This is like the simple case above but the samples form a closed trajectory (back is interpolated with the front).
/// Negative values interpolate from the end (1+x).
template <typename T>
inline T interpolatePiecewiseClosed(const float ref, const T* samples, const std::size_t size)
{
    const auto scaled = std::clamp<float>(((ref < 0) ? (1 + ref) : ref) * size, 0.0F, size);
    const auto ilo = static_cast<std::size_t>(std::clamp<std::int64_t>(static_cast<std::int64_t>(std::floor(scaled)), 0, size));
    const auto ihi = static_cast<std::size_t>(std::clamp<std::int64_t>(static_cast<std::int64_t>(std::ceil(scaled)), 0, size));
    const auto get = [&](const auto idx){ return samples[(idx >= size) ? 0 : idx]; };
    if (ihi == ilo)  // edge case
    {
        return get(ilo);
    }
    assert(ilo < size);
    return get(ilo) * (ihi - scaled) + get(ihi) * (scaled - ilo);
}

/// Wraps phase angle in [0, 1).
inline float wrapPhase(const float p) noexcept
{
    if (p >= 1)
    {
        return p - 1;
    }
    if (p < 0)
    {
        return p + 1;
    }
    return p;
}
