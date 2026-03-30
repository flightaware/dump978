// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "socket_input.h"

using namespace flightaware::uat;
using boost::asio::ip::tcp;

#include <iostream>

RawInput::RawInput(boost::asio::io_service &service, const std::string &host, const std::string &port_or_service, std::chrono::milliseconds reconnect_interval) : host_(host), port_or_service_(port_or_service), reconnect_interval_(reconnect_interval), resolver_(service), socket_(service), reconnect_timer_(service), used_(0) { readbuf_.resize(8192); }

void RawInput::Start() {
    auto self(shared_from_this());

    std::cerr << "Connecting to " << host_ << ":" << port_or_service_ << std::endl;
    tcp::resolver::query query(host_, port_or_service_);
    resolver_.async_resolve(query, [this, self](const boost::system::error_code &ec, tcp::resolver::iterator it) {
        if (!ec) {
            next_endpoint_ = it;
            TryNextEndpoint(boost::asio::error::make_error_code(boost::asio::error::host_not_found));
        } else if (ec == boost::asio::error::operation_aborted) {
            return;
        } else {
            HandleError(ec);
            return;
        }
    });
}

void RawInput::Stop() {
    reconnect_timer_.cancel();
    socket_.close();
}

void RawInput::TryNextEndpoint(const boost::system::error_code &last_error) {
    if (next_endpoint_ == tcp::resolver::iterator()) {
        // No more addresses to try
        HandleError(last_error);
        return;
    }

    tcp::endpoint endpoint = *next_endpoint_++;
    auto self(shared_from_this());
    socket_.async_connect(endpoint, [this, self, endpoint](const boost::system::error_code &ec) {
        if (!ec) {
            std::cerr << "Connected to " << endpoint << std::endl;
            ScheduleRead();
        } else if (ec == boost::asio::error::operation_aborted) {
            return;
        } else {
            std::cerr << "connection to " << endpoint << " failed: " << ec.message() << std::endl;
            socket_.close();
            TryNextEndpoint(ec);
        }
    });
}

void RawInput::ScheduleRead() {
    auto self(shared_from_this());

    if (!socket_.is_open())
        return;

    if (used_ >= readbuf_.size()) {
        HandleError(boost::asio::error::make_error_code(boost::asio::error::no_buffer_space));
        return;
    }

    socket_.async_read_some(boost::asio::buffer(readbuf_.data() + used_, readbuf_.size() - used_), [this, self](const boost::system::error_code &ec, std::size_t len) {
        if (ec) {
            HandleError(ec);
            return;
        }

        used_ += len;
        ParseBuffer();
        ScheduleRead();
    });
}

void RawInput::HandleError(const boost::system::error_code &ec) {
    if (ec == boost::asio::error::operation_aborted) {
        return;
    }
    socket_.close();

    if (reconnect_interval_.count() > 0) {
        // do this before calling the error handler, so the error handler has the option
        // of calling Stop() and cancelling the reconnect timer

        auto self(shared_from_this());

        reconnect_timer_.expires_from_now(reconnect_interval_);
        reconnect_timer_.async_wait([this, self](const boost::system::error_code &ec) {
            if (!ec)
                Start();
        });
    }

    if (error_handler_) {
        error_handler_(ec);
    }
}

void RawInput::ParseBuffer() {
    SharedMessageVector messages;

    auto sol = readbuf_.begin();
    auto end = readbuf_.begin() + used_;
    while (sol < end) {
        auto eol = std::find(sol, end, '\n');
        if (eol == end)
            break;

        std::string line(sol, eol);
        auto result = ParseLine(line);
        if (result) {
            if (!messages) {
                messages = std::make_shared<MessageVector>();
            }
            messages->emplace_back(std::move(*result));
        } else {
            std::cerr << "error: failed to parse input line: " << line << std::endl;
            HandleError(boost::system::errc::make_error_code(boost::system::errc::protocol_error));
            return;
        }
        sol = eol + 1;
    }

    if (sol != readbuf_.begin()) {
        std::copy(sol, end, readbuf_.begin());
        used_ = std::distance(sol, end);
    }
    if (messages) {
        DispatchMessages(messages);
    }
}

static inline int hexvalue(char c) {
    switch (c) {
    case '0':
        return 0;
    case '1':
        return 1;
    case '2':
        return 2;
    case '3':
        return 3;
    case '4':
        return 4;
    case '5':
        return 5;
    case '6':
        return 6;
    case '7':
        return 7;
    case '8':
        return 8;
    case '9':
        return 9;
    case 'a':
    case 'A':
        return 10;
    case 'b':
    case 'B':
        return 11;
    case 'c':
    case 'C':
        return 12;
    case 'd':
    case 'D':
        return 13;
    case 'e':
    case 'E':
        return 14;
    case 'f':
    case 'F':
        return 15;
    default:
        return -1;
    }
}

boost::optional<RawMessage> RawInput::ParseMetadataLine(const std::string &line) {
    // Parse metadata line that starts with '!'
    RawMessage::MetadataMap metadata;

    for (size_t i = 1; i < line.size();) {
        auto equals = line.find('=', i);
        auto semicolon = line.find(';', i);
        if (equals == std::string::npos || semicolon == std::string::npos || semicolon < equals) {
            // no more valid data
            break;
        }

        auto key = line.substr(i, equals - i);
        auto value = line.substr(equals + 1, semicolon - equals - 1);
        metadata[key] = value;

        i = semicolon + 1;
    }

    return RawMessage(std::move(metadata));
}

boost::optional<RawMessage> RawInput::ParseLine(const std::string &line) {
    if (line.size() < 2) {
        // too short
        return boost::none;
    }

    if (line[0] == '!') {
        // metadata only
        return ParseMetadataLine(line);
    }

    // message with data payload
    if (line[0] != '-' && line[0] != '+') {
        // badly formatted
        return boost::none;
    }

    auto eod = line.find(';', 1);
    if (eod == std::string::npos) {
        // missing semicolon
        return boost::none;
    }

    auto hexlength = eod - 1;
    if (hexlength % 2 != 0) {
        // wrong number of data characters
        return boost::none;
    }

    // parse hex digits
    Bytes payload;
    payload.reserve(hexlength / 2);

    for (decltype(eod) i = 1; i < eod; i += 2) {
        auto h1 = hexvalue(line[i]);
        auto h2 = hexvalue(line[i + 1]);
        if (h1 < 0 || h2 < 0) {
            // bad hex value
            return boost::none;
        }
        payload.push_back((std::uint8_t)((h1 << 4) | h2));
    }

    // parse key-value pairs

    unsigned rs = 0;
    double rssi = 0;
    std::uint64_t t = 0;
    std::uint64_t rt = 0;

    for (auto i = eod + 1; i < line.size();) {
        auto equals = line.find('=', i);
        auto semicolon = line.find(';', i);
        if (equals == std::string::npos || semicolon == std::string::npos || semicolon < equals) {
            // no more valid data
            break;
        }

        auto key = line.substr(i, equals - i);
        auto value = line.substr(equals + 1, semicolon - equals - 1);

        if (key == "rs") {
            try {
                rs = std::stoi(value, nullptr, 10);
            } catch (const std::exception &e) {
                rs = 0;
            }
        } else if (key == "rssi") {
            try {
                rssi = std::stod(value);
            } catch (const std::exception &e) {
                rssi = 0;
            }
        } else if (key == "t") {
            try {
                t = (std::uint64_t)(std::stod(value) * 1000);
            } catch (const std::exception &e) {
                t = 0;
            }
        } else if (key == "rt") {
            try {
                rt = (std::uint64_t)std::stoll(value, nullptr, 10);
            } catch (const std::exception &e) {
                rt = 0;
            }
        }

        i = semicolon + 1;
    }

    return RawMessage(std::move(payload), t, rs, rssi, rt);
}
