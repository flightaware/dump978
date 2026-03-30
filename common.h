// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef UAT_COMMON_H
#define UAT_COMMON_H

#ifndef VERSION
#define VERSION "unknown version compiled " __DATE__ " " __TIME__
#endif

#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

namespace flightaware {
  namespace uat {
    typedef std::vector<std::uint8_t> Bytes;
    typedef std::vector<std::uint16_t> PhaseBuffer;

    inline static double RoundN(double value, unsigned dp) {
        const double scale = std::pow(10, dp);
        return std::round(value * scale) / scale;
    }

    const auto unix_epoch = std::chrono::system_clock::from_time_t(0);

    inline static std::uint64_t now_millis() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - unix_epoch).count(); }
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
