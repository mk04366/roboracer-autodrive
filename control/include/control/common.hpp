#pragma once

template <typename T>
T clamp(const T &v, const T &lo, const T &hi)
{
    return (v < lo) ? lo : (v > hi) ? hi
                                    : v;
}
