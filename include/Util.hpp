#pragma once

#include <cstdlib>
#include <cmath>

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
