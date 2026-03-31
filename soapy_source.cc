// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "soapy_source.h"
#include "exception.h"

#include <iomanip>
#include <iostream>

#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Version.hpp>

using namespace flightaware::uat;

std::atomic_bool SoapySampleSource::log_handler_registered_(false);

static void SoapyLogger(const SoapySDRLogLevel logLevel, const char *message) {
    // clang-format off
    static std::map<SoapySDRLogLevel, std::string> levels = {
        {SOAPY_SDR_FATAL, "FATAL"},
        {SOAPY_SDR_CRITICAL, "CRITICAL"},
        {SOAPY_SDR_ERROR, "ERROR"},
        {SOAPY_SDR_WARNING, "WARNING"},
        {SOAPY_SDR_NOTICE, "NOTICE"},
        {SOAPY_SDR_INFO, "INFO"},
        {SOAPY_SDR_DEBUG, "DEBUG"},
        {SOAPY_SDR_TRACE, "TRACE"},
        {SOAPY_SDR_SSI, "SSI"}
    };
    // clang-format on

    if (logLevel == SOAPY_SDR_SSI) {
        // we don't care about this, and it's not masked by log level
        return;
    }

    std::string level;
    auto i = levels.find(logLevel);
    if (i == levels.end())
        level = "UNKNOWN";
    else
        level = i->second;

    std::cerr << "SoapySDR: " << level << ": " << message << std::endl;
}

static std::string FormatToSoapy(SampleFormat format) {
    // clang-format off
    static const std::map<SampleFormat,std::string> lookup = {
        { SampleFormat::CU8, SOAPY_SDR_CU8 },
        { SampleFormat::CS8_, SOAPY_SDR_CS8 },
        { SampleFormat::CS16H, SOAPY_SDR_CS16 },
        { SampleFormat::CF32H, SOAPY_SDR_CF32 }
    };
    // clang-format on

    auto i = lookup.find(format);
    if (i != lookup.end()) {
        return i->second;
    } else {
        return "";
    }
}

static SampleFormat SoapyToFormat(std::string format) {
    // clang-format off
    static const std::map<std::string,SampleFormat> lookup = {
        { SOAPY_SDR_CU8, SampleFormat::CU8 },
        { SOAPY_SDR_CS8, SampleFormat::CS8_ },
        { SOAPY_SDR_CS16, SampleFormat::CS16H },
        { SOAPY_SDR_CF32, SampleFormat::CF32H }
    };
    // clang-format on

    auto i = lookup.find(format);
    if (i != lookup.end()) {
        return i->second;
    } else {
        return SampleFormat::UNKNOWN;
    }
}

#if defined(SOAPY_SDR_API_VERSION) && (SOAPY_SDR_API_VERSION >= 0x00060000)
static SoapySDR::Kwargs KwargsFromString(const std::string &markup) { return SoapySDR::KwargsFromString(markup); }
#else
// Compatibility shim
static std::string trim(std::string::const_iterator i1, std::string::const_iterator i2) {
    auto begin = i1;
    while (begin < i2 && std::isspace(*begin))
        ++begin;

    auto end = i2;
    while (begin < end && std::isspace(end[-1]))
        --end;

    return std::string(begin, end);
}

static SoapySDR::Kwargs KwargsFromString(const std::string &markup) {
    SoapySDR::Kwargs kwargs;

    auto scan = markup.begin();
    while (scan < markup.end()) {
        auto key_start = scan;
        while (scan < markup.end() && *scan != '=' && *scan != ',') {
            ++scan;
        }

        std::string key = trim(key_start, scan);
        if (scan == markup.end() || *scan == ',') {
            ++scan;
            kwargs[key] = "";
        } else {
            ++scan;
            auto value_start = scan;
            while (scan < markup.end() && *scan != ',') {
                ++scan;
            }

            std::string value = trim(value_start, scan);
            ++scan;

            if (!key.empty()) {
                kwargs[key] = value;
            }
        }
    }

    return kwargs;
}
#endif

class SoapySDRCategory : public boost::system::error_category {
  public:
    const char *name() const noexcept override { return "soapysdr"; }
    std::string message(int ev) const override { return SoapySDR::errToStr(ev); }
};

static SoapySDRCategory soapysdr_category;

SoapySampleSource::SoapySampleSource(boost::asio::io_context &service, const std::string &device_name, const boost::program_options::variables_map &options) : timer_(service), device_name_(device_name), options_(options) {
    if (!log_handler_registered_.exchange(true)) {
        SoapySDR::registerLogHandler(SoapyLogger);
        SoapySDR::setLogLevel(SOAPY_SDR_NOTICE);
    }
}

SoapySampleSource::~SoapySampleSource() { Stop(); }

void SoapySampleSource::Init() {
    try {
        void (*unmake)(SoapySDR::Device *) = &SoapySDR::Device::unmake; // select the right overload
        device_ = {SoapySDR::Device::make(device_name_), unmake};
    } catch (const std::runtime_error &err) {
        throw config_error(std::string("No matching SoapySDR device found (cause: ") + err.what() + ")");
    }

    if (!device_) {
        throw config_error("No matching SoapySDR device found");
    }

    // hacky mchackerson
    device_->setSampleRate(SOAPY_SDR_RX, 0, 2083333.0);
    device_->setFrequency(SOAPY_SDR_RX, 0, 978000000);
    device_->setBandwidth(SOAPY_SDR_RX, 0, 3.0e6);

    if (options_.count("sdr-auto-gain")) {
        if (!device_->hasGainMode(SOAPY_SDR_RX, 0)) {
            throw config_error("Device does not support automatic gain mode");
        }
        std::cerr << "SoapySDR: using automatic gain" << std::endl;
        device_->setGainMode(SOAPY_SDR_RX, 0, true);
    } else if (options_.count("sdr-gain")) {
        auto gain = options_["sdr-gain"].as<double>();
        std::cerr << "SoapySDR: using manual gain " << std::fixed << std::setprecision(1) << gain << " dB" << std::endl;
        device_->setGainMode(SOAPY_SDR_RX, 0, false);
        device_->setGain(SOAPY_SDR_RX, 0, gain);
    } else {
        auto range = device_->getGainRange(SOAPY_SDR_RX, 0);
        std::cerr << "SoapySDR: using maximum manual gain " << std::fixed << std::setprecision(1) << range.maximum() << " dB" << std::endl;
        device_->setGainMode(SOAPY_SDR_RX, 0, false);
        device_->setGain(SOAPY_SDR_RX, 0, range.maximum());
    }

    if (options_.count("sdr-ppm")) {
        auto ppm = options_["sdr-ppm"].as<double>();
        if (ppm != 0) {
#ifdef SOAPY_SDR_API_HAS_FREQUENCY_CORRECTION_API
            if (device_->hasFrequencyCorrection(SOAPY_SDR_RX, 0) && ppm != 0) {
                std::cerr << "SoapySDR: using frequency correction " << std::fixed << std::setprecision(1) << ppm << " ppm" << std::endl;
                device_->setFrequencyCorrection(SOAPY_SDR_RX, 0, ppm);
            } else {
                std::cerr << "SoapySDR: device does not support frequency correction, --sdr-ppm option ignored" << std::endl;
            }
#else
            std::cerr << "SoapySDR: library version does not support frequency correction, --sdr-ppm option ignored" << std::endl;
#endif
        }
    }

    if (options_.count("sdr-antenna")) {
        auto antenna = options_["sdr-antenna"].as<std::string>();
        std::cerr << "SoapySDR: using antenna " << antenna << std::endl;
        device_->setAntenna(SOAPY_SDR_RX, 0, antenna);
    }

    if (options_.count("sdr-device-settings")) {
        for (auto kv : KwargsFromString(options_["sdr-device-settings"].as<std::string>())) {
            std::cerr << "SoapySDR: using device setting " << kv.first << "=" << kv.second << std::endl;
            device_->writeSetting(kv.first, kv.second);
        }
    }

    if (options_.count("format")) {
        format_ = options_["format"].as<SampleFormat>();
    }

    std::string soapy_format;
    if (format_ == SampleFormat::UNKNOWN) {
        double fullScale;
        soapy_format = device_->getNativeStreamFormat(SOAPY_SDR_RX, 0, fullScale);
        format_ = SoapyToFormat(soapy_format);
        if (format_ == SampleFormat::UNKNOWN) {
            throw config_error("Unsupported native SDR format: " + soapy_format + "; try specifying --format");
        }
    } else {
        soapy_format = FormatToSoapy(format_);
        if (soapy_format.empty()) {
            throw config_error("Unsupported sample format");
        }
    }

    std::vector<size_t> channels = {0};

    SoapySDR::Kwargs stream_settings;
    if (device_->getDriverKey() == "RTLSDR") {
        // some soapysdr builds have a very low default here
        stream_settings["buffsize"] = "262144";
    }

    if (options_.count("sdr-stream-settings")) {
        for (auto kv : KwargsFromString(options_["sdr-stream-settings"].as<std::string>())) {
            stream_settings[kv.first] = kv.second;
        }
    }

    for (auto &kv : stream_settings) {
        std::cerr << "SoapySDR: using stream setting " << kv.first << "=" << kv.second << std::endl;
    }

    try {
        stream_ = {device_->setupStream(SOAPY_SDR_RX, soapy_format, channels, stream_settings), std::bind(&SoapySDR::Device::closeStream, device_, std::placeholders::_1)};
    } catch (const std::runtime_error &err) {
        throw config_error(std::string("Failed to construct soapysdr stream (cause: ") + err.what() + ")");
    }

    if (!stream_) {
        throw config_error("Failed to construct soapysdr stream");
    }
}

void SoapySampleSource::Start() {
    if (!device_ || !stream_) {
        Init();
    }

    device_->activateStream(stream_.get());

    halt_ = false;
    rx_thread_.reset(new std::thread(&SoapySampleSource::Run, this));

    Keepalive();
}

void SoapySampleSource::Keepalive() {
    if (rx_thread_ && rx_thread_->joinable()) {
        // Keep the io_context alive while the rx_thread is active
        auto self(shared_from_this());
        timer_.expires_after(std::chrono::milliseconds(1000));
        timer_.async_wait([self, this](const boost::system::error_code &ec) {
            if (!ec) {
                Keepalive();
            }
        });
    }
}

void SoapySampleSource::Stop() {
    if (stream_) {
        // rtlsdr needs the rx thread to drain data before this returns..
        device_->deactivateStream(stream_.get());
    }
    if (rx_thread_) {
        halt_ = true;
        rx_thread_->join();
        rx_thread_.reset();
    }
    if (stream_) {
        stream_.reset();
    }
    if (device_) {
        device_.reset();
    }
}

void SoapySampleSource::Run() {
    const auto bytes_per_element = BytesPerSample(format_);
    const auto elements = std::max<size_t>(65536, device_->getStreamMTU(stream_.get()));

    Bytes block;
    block.reserve(elements * bytes_per_element);

    const auto overflow_report_interval = std::chrono::milliseconds(15000);
    auto last_overflow_report = std::chrono::steady_clock::now();
    unsigned overflow_count = 0;

    while (!halt_) {
        void *buffs[1] = {block.data()};
        int flags = 0;
        long long time_ns;

        block.resize(elements * bytes_per_element);
        auto elements_read = device_->readStream(stream_.get(), buffs, elements, flags, time_ns,
                                                 /* timeout, microseconds */ 5000000);
        if (halt_) {
            break;
        }

        if (elements_read < 0) {
            if (elements_read == SOAPY_SDR_OVERFLOW) {
                ++overflow_count;
            } else {
                DispatchError(boost::system::error_code{elements_read, soapysdr_category});
                break;
            }
        }

        if (overflow_count > 0) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_overflow_report > overflow_report_interval) {
                std::cerr << "SoapySDR: " << overflow_count << " recent input overruns (sample data dropped)" << std::endl;
                last_overflow_report = now;
                overflow_count = 0;
            }
        }

        if (elements_read <= 0) {
            continue;
        }

        block.resize(elements_read * bytes_per_element);

        // work out a starting timestamp
        static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
        auto end_of_block = std::chrono::system_clock::now();
        auto start_of_block = end_of_block - (std::chrono::milliseconds(1000) * elements / 2083333);
        std::uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(start_of_block - unix_epoch).count();

        DispatchBuffer(timestamp, block);
    }
}
