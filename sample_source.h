// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_SAMPLE_SOURCE_H
#define DUMP978_SAMPLE_SOURCE_H

#include <chrono>
#include <fstream>
#include <functional>
#include <memory>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "common.h"
#include "convert.h"

namespace flightaware {
  namespace uat {
    class SampleSource : public std::enable_shared_from_this<SampleSource> {
      public:
        typedef std::shared_ptr<SampleSource> Pointer;
        typedef std::function<void(std::uint64_t, const Bytes &)> Consumer;
        typedef std::function<void(const boost::system::error_code &ec)> ErrorHandler;

        virtual ~SampleSource() {}

        virtual void Init() = 0;
        virtual void Start() = 0;
        virtual void Stop() = 0;
        virtual SampleFormat Format() = 0;

        void SetConsumer(Consumer consumer) { consumer_ = consumer; }
        void SetErrorHandler(ErrorHandler handler) { error_handler_ = handler; }

      protected:
        SampleSource() {}

        void DispatchBuffer(std::uint64_t timestamp, const Bytes &buffer) {
            if (consumer_) {
                consumer_(timestamp, buffer);
            }
        }

        void DispatchError(const boost::system::error_code &ec) {
            if (error_handler_) {
                error_handler_(ec);
            }
        }

      private:
        Consumer consumer_;
        ErrorHandler error_handler_;
    };

    class FileSampleSource : public SampleSource {
      public:
        static SampleSource::Pointer Create(boost::asio::io_service &service, const boost::filesystem::path &path, const boost::program_options::variables_map &options = boost::program_options::variables_map(), std::size_t samples_per_second = 2083333, std::size_t samples_per_block = 524288) { return Pointer(new FileSampleSource(service, path, options, samples_per_second, samples_per_block)); }

        void Init() override {}
        void Start() override;
        void Stop() override;
        SampleFormat Format() override { return format_; }

      private:
        FileSampleSource(boost::asio::io_service &service, const boost::filesystem::path &path, const boost::program_options::variables_map &options, std::size_t samples_per_second, std::size_t samples_per_block) : service_(service), path_(path), timer_(service) {
            if (!options.count("format")) {
                throw std::runtime_error("--format must be specified when using a file input");
            }

            throttle_ = (options.count("file-throttle") > 0);

            format_ = options["format"].as<SampleFormat>();
            alignment_ = BytesPerSample(format_);
            bytes_per_second_ = samples_per_second * alignment_;
            block_.reserve(samples_per_block * alignment_);
        }

        void ReadBlock(const boost::system::error_code &ec);

        boost::asio::io_service &service_;
        boost::filesystem::path path_;
        SampleFormat format_;
        unsigned alignment_;
        bool throttle_;
        std::size_t bytes_per_second_;

        std::ifstream stream_;
        boost::asio::steady_timer timer_;
        std::chrono::steady_clock::time_point next_block_;
        Bytes block_;
        std::uint64_t timestamp_;
    };

    class StdinSampleSource : public SampleSource {
      public:
        static SampleSource::Pointer Create(boost::asio::io_service &service, const boost::program_options::variables_map &options, std::size_t samples_per_second = 2083333, std::size_t samples_per_block = 524288) { return Pointer(new StdinSampleSource(service, options, samples_per_second, samples_per_block)); }

        void Init() override {}
        void Start() override;
        void Stop() override;
        SampleFormat Format() override { return format_; }

      private:
        StdinSampleSource(boost::asio::io_service &service, const boost::program_options::variables_map &options, std::size_t samples_per_second, std::size_t samples_per_block) : service_(service), samples_per_second_(samples_per_second), stream_(service), used_(0) {
            if (!options.count("format")) {
                throw std::runtime_error("--format must be specified when using a file input");
            }

            format_ = options["format"].as<SampleFormat>();
            alignment_ = BytesPerSample(format_);
            block_.reserve(samples_per_block * alignment_);
        }

        void ScheduleRead();

        boost::asio::io_service &service_;
        SampleFormat format_;
        unsigned alignment_;
        std::size_t samples_per_second_;
        boost::asio::posix::stream_descriptor stream_;
        Bytes block_;
        std::size_t used_;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
