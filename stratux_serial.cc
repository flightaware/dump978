// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

// Try to convince asio to support serial hardware flowcontrol
#define _DEFAULT_SOURCE
#define _BSD_SOURCE

#include "stratux_serial.h"

#include <iostream>
#include <array>

using namespace flightaware::uat;

//
// Support for the Stratux v3 UAT dongle.
// This is based on the TI CC1310 and produces demodulated
// messages including FEC bytes over USB serial.
//

// Input message format is:
//
// 0A B0 CD E0   - preamble
// xx yy         - payload size in bytes, 16 bits, big-endian
// ss            - RSSI, 8 bits
// tt tt tt tt   - timestamp, 32 bits, big-endian
// pp pp pp ...  - payload, size as above, includes FEC data

enum class StratuxSerial::ParserState {
    PREAMBLE, // scanning for preamble sequence
    LENGTH_1, // reading first length byte
    LENGTH_2, // reading second length byte
    MESSAGE   // reading rssi / timestamp / payload
};

StratuxSerial::StratuxSerial(boost::asio::io_service &io_service, const std::string &path) : path_(path), port_(io_service), read_timer_(io_service), parser_state_(ParserState::PREAMBLE), preamble_index_(0) {}

void StratuxSerial::Start() {
    try {
        port_.open(path_);

        // configure for 2Mbps, 8N1, hardware flow control
        port_.set_option(boost::asio::serial_port_base::character_size(8));
        port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        port_.set_option(boost::asio::serial_port_base::baud_rate(2000000));

        // not_supported usually means a library build problem, ignore that
        boost::system::error_code ec;
        port_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware), ec);
        if (ec && ec != boost::asio::error::operation_not_supported) {
            throw boost::system::system_error(ec);
        }
    } catch (const boost::system::system_error &e) {
        HandleError(e.code());
        return;
    }

    StartReading();
}

void StratuxSerial::Stop() {
    read_timer_.cancel();
    if (port_.is_open()) {
        boost::system::error_code ignored;
        port_.close(ignored);
    }
}

void StratuxSerial::StartReading() {
    auto self(shared_from_this());
    std::shared_ptr<Bytes> buf;

    buf.swap(readbuf_);
    if (buf) {
        buf->resize(read_buffer_size);
    } else {
        buf = std::make_shared<Bytes>(read_buffer_size);
    }

    port_.async_read_some(boost::asio::buffer(*buf), [this, self, buf](const boost::system::error_code &ec, std::size_t len) {
        if (ec) {
            readbuf_ = buf;
            HandleError(ec);
        } else {
            buf->resize(len);
            ParseInput(*buf);
            readbuf_ = buf;

            // If we didn't get a full-ish buffer, then wait a bit before the next read so we don't
            // spin reading only a few bytes each time.

            // (unfortunately, boost::asio's edge-triggered epoll still gets woken repeatedly each time a
            // little more data arrives, but at least we don't have to do a bunch of work on every one of
            // those)
            if (len < read_buffer_size * 3 / 4) {
                read_timer_.expires_from_now(read_interval);
                read_timer_.async_wait([this, self](const boost::system::error_code &ec) {
                    if (!ec) {
                        StartReading();
                    }
                });
            } else {
                StartReading();
            }
        }
    });
}

void StratuxSerial::ParseInput(const Bytes &buf) {
    SharedMessageVector messages;
    static const std::array<std::uint8_t, 4> preamble = {0x0A, 0xB0, 0xCD, 0xE0};

    // 2000000Mbps, 8N1 = 200,000 bytes/s = 200 bytes/ms
    static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
    auto start_of_read = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - unix_epoch).count() - buf.size() / 200;
    std::uint64_t previous_sys_timestamp = 0;
    std::uint32_t previous_raw_timestamp = 0;

    for (auto i = buf.begin(); i != buf.end();) {
        switch (parser_state_) {
        case ParserState::PREAMBLE:
            if (*i == preamble[preamble_index_]) {
                // remember the (system) time of the preamble start
                if (preamble_index_ == 0) {
                    message_start_timestamp_ = start_of_read + (std::distance(buf.begin(), i) / 200);
                }
                ++i;
                if (++preamble_index_ >= preamble.size()) {
                    parser_state_ = ParserState::LENGTH_1;
                }
            } else {
                if (preamble_index_ > 0) {
                    preamble_index_ = 0;
                } else {
                    ++i;
                }
            }
            break;

        case ParserState::LENGTH_1:
            message_length_ = *i++ + 5;
            parser_state_ = ParserState::LENGTH_2;
            break;

        case ParserState::LENGTH_2:
            message_length_ += (*i++ << 8);
            message_.clear();
            parser_state_ = ParserState::MESSAGE;
            break;

        case ParserState::MESSAGE: {
            auto bytes_to_copy = std::min<std::size_t>(message_length_ - message_.size(), std::distance(i, buf.end()));
            std::copy(i, i + bytes_to_copy, std::back_inserter(message_));
            i += bytes_to_copy;

            if (message_.size() == message_length_) {
                // work out a suitable timestamp
                std::uint32_t raw_timestamp = message_[1] | (message_[2] << 8) | (message_[3] << 16) | (message_[4] << 24);
                std::uint64_t sys_timestamp;
                if (previous_sys_timestamp != 0 && raw_timestamp > previous_raw_timestamp) {
                    sys_timestamp = previous_sys_timestamp + (raw_timestamp - previous_raw_timestamp) / 4000;
                } else {
                    sys_timestamp = previous_sys_timestamp = message_start_timestamp_;
                    previous_raw_timestamp = raw_timestamp;
                }

                auto parsed = ParseMessage(message_, sys_timestamp);
                if (parsed) {
                    if (!messages) {
                        messages = std::make_shared<MessageVector>();
                    }
                    messages->push_back(std::move(*parsed));
                }
                message_.clear();
                parser_state_ = ParserState::PREAMBLE;
                preamble_index_ = 0;
            }
            break;
        }

        default:
            assert("impossible state" && false);
        }
    }

    if (messages) {
        DispatchMessages(messages);
    }
}

boost::optional<RawMessage> StratuxSerial::ParseMessage(const Bytes &message, std::uint64_t sys_timestamp) {
    assert(message.size() >= 5);

    // not entirely clear what the RSSI format is; here we assume it's
    // the format returned by the CC1310 (signed dBm)
    std::int8_t raw_rssi = message[0];
    float rssi = 1.0 * raw_rssi;

    std::uint32_t raw_timestamp = message[1] | (message[2] << 8) | (message[3] << 16) | (message[4] << 24);

    Bytes payload;
    std::copy(message.begin() + 5, message.end(), std::back_inserter(payload));

    bool success;
    Bytes corrected;
    unsigned errors;

    switch (payload.size()) {
    case UPLINK_BYTES:
        std::tie(success, corrected, errors) = fec_.CorrectUplink(payload);
        break;

    case DOWNLINK_LONG_BYTES:
        std::tie(success, corrected, errors) = fec_.CorrectDownlink(payload);
        break;

    default:
        // unexpected length
        return boost::none;
    }

    if (!success) {
        // FEC failed
        return boost::none;
    }

    return RawMessage{std::move(corrected), sys_timestamp, errors, rssi, raw_timestamp};
}

void StratuxSerial::HandleError(const boost::system::error_code &ec) { DispatchError(ec); }
