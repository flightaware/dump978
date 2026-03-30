// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_STRATUX_SERIAL_H
#define DUMP978_STRATUX_SERIAL_H

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/optional.hpp>

#include "common.h"
#include "fec.h"
#include "message_source.h"

namespace flightaware {
  namespace uat {
    class StratuxSerial : public MessageSource, public std::enable_shared_from_this<StratuxSerial> {
      public:
        typedef std::shared_ptr<StratuxSerial> Pointer;

        virtual ~StratuxSerial() {}

        virtual void Start();
        virtual void Stop();

        static Pointer Create(boost::asio::io_service &io_service, const std::string &path) { return Pointer(new StratuxSerial(io_service, path)); }

      protected:
        StratuxSerial(boost::asio::io_service &io_service, const std::string &path);

      private:
        void StartReading();
        void ParseInput(const Bytes &buf);
        boost::optional<RawMessage> ParseMessage(const Bytes &message, std::uint64_t sys_timestamp);
        void HandleError(const boost::system::error_code &ec);

        // the size of the read buffer
        const std::size_t read_buffer_size = 1024 * 16;
        // how long to wait between scheduling reads (to reduce the spinning on short messages)
        const std::chrono::milliseconds read_interval = std::chrono::milliseconds(50);

        boost::asio::io_service &io_service_;
        std::string path_;
        boost::asio::serial_port port_;
        boost::asio::steady_timer read_timer_;
        std::shared_ptr<Bytes> readbuf_;
        FEC fec_;

        enum class ParserState;
        ParserState parser_state_;
        unsigned preamble_index_;
        std::uint32_t message_length_;
        Bytes message_;
        std::uint64_t message_start_timestamp_;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
