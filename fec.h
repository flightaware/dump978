// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef UAT_FEC_H
#define UAT_FEC_H

#include <tuple>

#include "common.h"

namespace flightaware {
  namespace uat {
    // Deinterleaving and error-correction of UAT messages.
    // This delegates to the "fec" library (in fec/) for the actual Reed-Solomon
    // error-correction work.
    class FEC {
      public:
        FEC();
        ~FEC();

        // Given DOWNLINK_LONG_BYTES of demodulated data, returns a tuple of:
        //    bool     - true if the message is good, false if it was uncorrectable.
        //    Bytes    - a buffer containing the corrected data with FEC bits removed;
        //               this will be either DOWNLINK_SHORT_DATA_BYTES or
        //               DOWNLINK_LONG_DATA_BYTES in size depending on the detected
        //               message type. Empty if the message was uncorrectable.
        //    unsigned - the number of errors corrected. 0 if the message was
        //    uncorrectable
        // `erasures` is an optional vector of indexes into `raw` that should be
        // handled as erasures
        std::tuple<bool, Bytes, unsigned> CorrectDownlink(const Bytes &raw, const std::vector<std::size_t> &erasures = {});

        // Given UPLINK_BYTES of demodulated data, returns a tuple of:
        //    bool     - true if the message is good, false if it was uncorrectable.
        //    Bytes    - a buffer containing the deinterleaved, corrected data with
        //               FEC bits removed; this will be exactly UPLINK_DATA_BYTES
        //               in size.  Empty if the message was uncorrectable.
        //    unsigned - the number of errors corrected. 0 if the message was
        //    uncorrectable
        // `erasures` is an optional vector of indexes into `raw` that should be
        // handled as erasures
        std::tuple<bool, Bytes, unsigned> CorrectUplink(const Bytes &raw, const std::vector<std::size_t> &erasures = {});

      private:
        void *rs_uplink_;
        void *rs_downlink_short_;
        void *rs_downlink_long_;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
