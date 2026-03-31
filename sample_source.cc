// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "sample_source.h"

#include <boost/asio/post.hpp>

#include <chrono>
#include <iostream>

using namespace flightaware::uat;

void FileSampleSource::Start() {
    stream_.open(path_.native());
    if (!stream_.good()) {
        auto ec = boost::system::error_code(errno, boost::system::system_category());
        stream_.close();
        DispatchError(ec);
        return;
    }

    next_block_ = std::chrono::steady_clock::now();
    timestamp_ = 1; // always use synthetic timestamps for file sources

    auto self = std::static_pointer_cast<FileSampleSource>(shared_from_this());
    boost::asio::post(service_,
                      std::bind(&FileSampleSource::ReadBlock, self, boost::system::error_code()));
}

void FileSampleSource::Stop() {
    timer_.cancel();
    if (stream_.is_open()) {
        stream_.close();
        DispatchError(boost::asio::error::eof);
    }
}

void FileSampleSource::ReadBlock(const boost::system::error_code &ec) {
    if (ec) {
        if (ec == boost::asio::error::operation_aborted) {
            return;
        }

        stream_.close();
        DispatchError(ec);
        return;
    }

    if (!stream_.is_open()) {
        return;
    }

    block_.resize(block_.capacity());
    stream_.read(reinterpret_cast<char *>(block_.data()), block_.size());

    if (stream_.bad()) {
        auto ec = boost::system::error_code(errno, boost::system::system_category());
        stream_.close();
        DispatchError(ec);
        return;
    }

    block_.resize(stream_.gcount() - (stream_.gcount() % alignment_));
    if (!block_.empty()) {
        DispatchBuffer(timestamp_, block_);
        timestamp_ += (block_.size() * 1000ULL / bytes_per_second_);
    }

    if (stream_.eof()) {
        stream_.close();
        DispatchError(boost::asio::error::eof);
        return;
    }

    auto self = std::static_pointer_cast<FileSampleSource>(shared_from_this());
    if (throttle_) {
        auto delay = std::chrono::nanoseconds(1000000000ULL * block_.size() / bytes_per_second_);
        next_block_ += delay;
        timer_.expires_at(next_block_);
        timer_.async_wait(std::bind(&FileSampleSource::ReadBlock, self, std::placeholders::_1));
    } else {
        boost::asio::post(service_,
                          std::bind(&FileSampleSource::ReadBlock, self, boost::system::error_code()));
    }
}

//
//
//

void StdinSampleSource::Start() {
    stream_.assign(::dup(STDIN_FILENO));
    ScheduleRead();
}

void StdinSampleSource::Stop() {
    if (stream_.is_open()) {
        stream_.close();
        DispatchError(boost::asio::error::eof);
    }
}

void StdinSampleSource::ScheduleRead() {
    if (!stream_.is_open()) {
        return;
    }

    block_.resize(block_.capacity());

    auto self = shared_from_this();
    stream_.async_read_some(boost::asio::buffer(block_.data() + used_, block_.size() - used_), [this, self](const boost::system::error_code &ec, std::size_t bytes_transferred) {
        if (ec) {
            if (ec == boost::asio::error::operation_aborted) {
                return;
            }

            stream_.close();
            DispatchError(ec);
            return;
        }

        used_ += bytes_transferred;

        // work out a starting timestamp
        static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
        auto end_of_block = std::chrono::system_clock::now();
        auto start_of_block = end_of_block - (std::chrono::milliseconds(1000) * bytes_transferred / samples_per_second_ / alignment_);
        std::uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(start_of_block - unix_epoch).count();

        // fixme, don't copy!
        auto trailing_bytes = used_ % alignment_;
        auto leading_bytes = used_ - trailing_bytes;

        Bytes buffer;
        buffer.resize(leading_bytes);
        std::copy(block_.begin(), block_.begin() + leading_bytes, buffer.begin());
        std::copy(block_.begin() + leading_bytes, block_.end(), block_.begin());
        used_ = trailing_bytes;
        DispatchBuffer(timestamp, buffer);
        ScheduleRead();
    });
}
