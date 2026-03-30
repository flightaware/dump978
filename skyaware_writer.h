// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef SKYAWARE_WRITER_H
#define SKYAWARE_WRITER_H

#include <chrono>
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>
#include <boost/filesystem.hpp>

#include "track.h"
#include "uat_message.h"

namespace flightaware {
  namespace skyaware {
    class SkyAwareWriter : public std::enable_shared_from_this<SkyAwareWriter> {
      public:
        typedef std::shared_ptr<SkyAwareWriter> Pointer;

        static Pointer Create(boost::asio::io_service &service, flightaware::uat::Tracker::Pointer tracker, const boost::filesystem::path &dir, std::chrono::milliseconds interval, unsigned history_count, std::chrono::milliseconds history_interval, boost::optional<std::pair<double, double>> location) { return Pointer(new SkyAwareWriter(service, tracker, dir, interval, history_count, history_interval, location)); }

        void Start();
        void Stop();

      private:
        SkyAwareWriter(boost::asio::io_service &service, flightaware::uat::Tracker::Pointer tracker, const boost::filesystem::path &dir, std::chrono::milliseconds interval, unsigned history_count, std::chrono::milliseconds history_interval, boost::optional<std::pair<double, double>> location) : strand_(service), timer_(service), tracker_(tracker), dir_(dir), interval_(interval), history_count_(history_count), history_interval_(history_interval), location_(location) {}

        void PeriodicWrite();

        boost::asio::io_service::strand strand_;
        boost::asio::steady_timer timer_;
        flightaware::uat::Tracker::Pointer tracker_;
        boost::filesystem::path dir_;
        std::chrono::milliseconds interval_;
        unsigned history_count_;
        std::chrono::milliseconds history_interval_;
        boost::optional<std::pair<double, double>> location_;
        unsigned next_history_index_ = 0;
        std::uint64_t next_history_time_ = 0;
    };
  }; // namespace flightaware::skyaware
};   // namespace flightaware

#endif
