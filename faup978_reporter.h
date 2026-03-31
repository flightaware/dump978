// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef FAUP978_REPORTER_H
#define FAUP978_REPORTER_H

#include <chrono>
#include <memory>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include "track.h"
#include "uat_message.h"

namespace flightaware {
  namespace faup978 {
    struct ReportState {
        std::uint64_t slow_report_time = 0;
        std::uint64_t report_time = 0;
        flightaware::uat::AircraftState report_state;
    };

    class Reporter : public std::enable_shared_from_this<Reporter> {
      public:
        typedef std::shared_ptr<Reporter> Pointer;

        static constexpr const char *TSV_VERSION_8U = "8U";
        static constexpr const char *TSV_VERSION_8U_FIX = "8UF";

        static Pointer Create(boost::asio::io_context &service, std::chrono::milliseconds interval = std::chrono::milliseconds(500), std::chrono::milliseconds timeout = std::chrono::seconds(300)) { return Pointer(new Reporter(service, interval, timeout)); }

        void Start();
        void Stop();

        void HandleMessages(flightaware::uat::SharedMessageVector messages);

      private:
        Reporter(boost::asio::io_context &service, std::chrono::milliseconds interval, std::chrono::milliseconds timeout) : strand_(service), report_timer_(service), purge_timer_(service), interval_(interval), timeout_(timeout) { tracker_ = flightaware::uat::Tracker::Create(service, timeout); }

        void PeriodicReport();
        void PurgeOld();
        void ReportOneAircraft(const flightaware::uat::Tracker::AddressKey &key, const flightaware::uat::AircraftState &aircraft, std::uint64_t now);

        const char *TSVVersion() const { return fecfix_ ? TSV_VERSION_8U_FIX : TSV_VERSION_8U; }

        boost::asio::io_context::strand strand_;
        boost::asio::steady_timer report_timer_;
        boost::asio::steady_timer purge_timer_;
        std::chrono::milliseconds interval_;
        std::chrono::milliseconds timeout_;
        flightaware::uat::Tracker::Pointer tracker_;
        std::map<flightaware::uat::Tracker::AddressKey, ReportState> reported_;
        bool fecfix_ = false;
    };
  }; // namespace flightaware::faup978
};   // namespace flightaware

#endif
