// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_EXCEPTION_H
#define DUMP978_EXCEPTION_H

#include <stdexcept>

namespace flightaware {
  namespace uat {
    class config_error : public std::runtime_error {
      public:
        explicit config_error(const std::string &what_arg) : runtime_error(what_arg) {}
        explicit config_error(const char *what_arg) : runtime_error(what_arg) {}
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
