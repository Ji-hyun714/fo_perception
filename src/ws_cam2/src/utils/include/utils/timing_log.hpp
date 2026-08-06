#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

namespace utils {

    template <typename Fn>
inline double measureBlockMs(Fn&& fn)
{
    const auto t0 = std::chrono::steady_clock::now();
    std::forward<Fn>(fn)();
    const auto t1 = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}


    inline void printTimingMs(const std::string& tag, double ms, bool enabled = true)
{
    if (!enabled) {
        return;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << ms;
    std::cout << "[debug] " << tag << " : " << oss.str() << " ms" << std::endl;
}

} // namespace utils
