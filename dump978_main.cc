// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include <boost/asio/ip/address_v4.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <memory>

#include "adaptive.h"
#include "convert.h"
#include "demodulator.h"
#include "exception.h"
#include "message_dispatch.h"
#include "sample_source.h"
#include "soapy_source.h"
#include "socket_output.h"
#include "stratux_serial.h"

using namespace flightaware::uat;

namespace po = boost::program_options;
using boost::asio::ip::tcp;

struct listen_option {
    std::string host;
    std::string port;
};

SampleFormat format;

// Specializations of validate for --xxx-port
void validate(boost::any &v, const std::vector<std::string> &values, listen_option *target_type, int) {
    po::validators::check_first_occurrence(v);
    const std::string &s = po::validators::get_single_string(values);

    static const boost::regex r("(?:([^:]+):)?(\\d+)");
    boost::smatch match;
    if (boost::regex_match(s, match, r)) {
        listen_option o;
        o.host = match[1];
        o.port = match[2];
        v = boost::any(o);
    } else {
        throw po::validation_error(po::validation_error::invalid_option_value);
    }
}

// Specializations of validate for --format
namespace flightaware {
  namespace uat {
    void validate(boost::any &v, const std::vector<std::string> &values, SampleFormat *target_type, int) {
        po::validators::check_first_occurrence(v);
        const std::string &s = po::validators::get_single_string(values);

        // clang-format off
        static std::map<std::string, SampleFormat> formats = {
            {"CU8", SampleFormat::CU8},
            {"CS8", SampleFormat::CS8_},
            {"CS16H", SampleFormat::CS16H},
            {"CF32H", SampleFormat::CF32H}
        };
        // clang-format on

        auto entry = formats.find(s);
        if (entry == formats.end())
            throw po::validation_error(po::validation_error::invalid_option_value);

        v = boost::any(entry->second);
    }
  } // namespace flightaware::uat
} // namespace flightaware

#define EXIT_NO_RESTART (64)

static int realmain(int argc, char **argv) {
    boost::asio::io_context io_context;

    // clang-format off
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("version", "show version")
        ("raw-stdout", "write raw messages to stdout")
        ("json-stdout", "write decoded json to stdout")
        ("format", po::value<SampleFormat>(), "set sample format")
        ("stdin", "read sample data from stdin")
        ("file", po::value<std::string>(), "read sample data from a file")
        ("file-throttle", "throttle file input to realtime")
        ("sdr", po::value<std::string>(), "read sample data from named SDR device")
        ("sdr-auto-gain", "enable SDR AGC")
        ("sdr-gain", po::value<double>(), "set SDR gain in dB")
        ("sdr-ppm", po::value<double>(), "set SDR frequency correction in PPM")
        ("sdr-antenna", po::value<std::string>(), "set SDR antenna name")
        ("sdr-stream-settings", po::value<std::string>(), "set SDR stream key-value settings")
        ("sdr-device-settings", po::value<std::string>(), "set SDR device key-value settings")
        ("adaptive-range", "enable adaptive gain management targeting a dynamic range")
        ("adaptive-burst", "enable adaptive gain management based on detecting loud undecoded bursts")
        ("adaptive-min-gain", po::value<double>(), "adaptive gain: never use gains below this, in dB")
        ("adaptive-max-gain", po::value<double>(), "adaptive gain: never use gains above this, in dB")
        ("adaptive-duty-cycle", po::value<double>(), "adaptive gain: fraction of samples to inspect, in percent (default: 50)")
        ("adaptive-range-target", po::value<double>(), "adaptive gain: target dynamic range, in dB (default: auto)")
        ("adaptive-range-alpha", po::value<double>(), "adaptive gain: noise floor smoothing factor, 0..1")
        ("adaptive-range-percentile", po::value<unsigned>(), "adaptive gain: percentile of sample magnitude to use as the noise floor")
        ("adaptive-range-change-delay", po::value<unsigned>(), "adaptive gain: seconds to wait after a gain change before acting on noise measurements")
        ("adaptive-range-scan-delay", po::value<unsigned>(), "adaptive gain: seconds to wait before rescanning after an abortive dynamic range scan")
        ("adaptive-range-rescan-delay", po::value<unsigned>(), "adaptive gain: seconds to wait between periodic scans for increased gain")
        ("adaptive-burst-alpha", po::value<double>(), "adaptive gain: burst rate smoothing factor, 0..1")
        ("adaptive-burst-change-delay", po::value<unsigned>(), "adaptive gain: seconds to wait after a gain change before acting on burst measurements")
        ("adaptive-burst-loud-rate", po::value<double>(), "adaptive gain: decrease gain if the loud undecoded burst rate exceeds this rate (per second)")
        ("adaptive-burst-loud-runlength", po::value<unsigned>(), "adaptive gain: decrease gain if the loud undecoded burst rate is exceeded for this many consecutive seconds")
        ("adaptive-burst-quiet-rate", po::value<double>(), "adaptive gain: allow gain increases only if the loud decoded message rate is below this rate (per second)")
        ("adaptive-burst-quiet-runlength", po::value<unsigned>(), "adaptive gain: allow gain increases only after the loud decoded message rate has been low for this many consecutive seconds")
        ("stratuxv3", po::value<std::string>(), "read messages from Stratux v3 UAT dongle on given serial port")
        ("raw-port", po::value<std::vector<listen_option>>(), "listen for connections on [host:]port and provide raw messages")
        ("raw-legacy-port", po::value<std::vector<listen_option>>(), "listen for connections on [host:]port and provide raw messages, with no initial metadata header")
        ("json-port", po::value<std::vector<listen_option>>(), "listen for connections on [host:]port and provide decoded json");
    // clang-format on

    po::variables_map opts;

    try {
        po::store(po::parse_command_line(argc, argv, desc), opts);
        po::notify(opts);
    } catch (boost::program_options::error &err) {
        std::cerr << err.what() << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("help")) {
        std::cerr << "dump978-fa " << VERSION << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("version")) {
        std::cerr << "dump978-fa " << VERSION << std::endl;
        return EXIT_NO_RESTART;
    }

    MessageDispatch dispatch;
    SampleSource::Pointer sample_source;
    std::shared_ptr<SoapySampleSource> soapy_source;
    MessageSource::Pointer message_source;

    tcp::resolver resolver(io_context);

    if (opts.count("stdin") + opts.count("file") + opts.count("sdr") + opts.count("stratuxv3") != 1) {
        std::cerr << "Exactly one of --stdin, --file, --sdr, or --stratuxv3 must be used" << std::endl;
        return EXIT_NO_RESTART;
    }

    // clang-format off
    static const char *adaptive_option_names[] = {
        "adaptive-range", "adaptive-burst", "adaptive-min-gain", "adaptive-max-gain", "adaptive-duty-cycle",
        "adaptive-range-target", "adaptive-range-alpha", "adaptive-range-percentile",
        "adaptive-range-change-delay", "adaptive-range-scan-delay", "adaptive-range-rescan-delay",
        "adaptive-burst-alpha", "adaptive-burst-change-delay", "adaptive-burst-loud-rate",
        "adaptive-burst-loud-runlength", "adaptive-burst-quiet-rate", "adaptive-burst-quiet-runlength"
    };
    // clang-format on

    for (auto option : adaptive_option_names) {
        if (opts.count(option) && !opts.count("sdr")) {
            std::cerr << "--" << option << " can only be used with --sdr" << std::endl;
            return EXIT_NO_RESTART;
        }
    }

    AdaptiveParams adaptive_params;
    adaptive_params.range_control = (opts.count("adaptive-range") > 0);
    adaptive_params.burst_control = (opts.count("adaptive-burst") > 0);

    if ((adaptive_params.range_control || adaptive_params.burst_control) && opts.count("sdr-auto-gain")) {
        std::cerr << "--sdr-auto-gain cannot be combined with adaptive gain control" << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("adaptive-min-gain"))
        adaptive_params.min_gain_db = opts["adaptive-min-gain"].as<double>();
    if (opts.count("adaptive-max-gain"))
        adaptive_params.max_gain_db = opts["adaptive-max-gain"].as<double>();
    if (opts.count("adaptive-duty-cycle"))
        adaptive_params.duty_cycle = opts["adaptive-duty-cycle"].as<double>() / 100.0;
    if (opts.count("adaptive-range-target"))
        adaptive_params.range_target = opts["adaptive-range-target"].as<double>();
    if (opts.count("adaptive-range-alpha"))
        adaptive_params.range_alpha = opts["adaptive-range-alpha"].as<double>();
    if (opts.count("adaptive-range-percentile"))
        adaptive_params.range_percentile = opts["adaptive-range-percentile"].as<unsigned>();
    if (opts.count("adaptive-range-change-delay"))
        adaptive_params.range_change_delay = opts["adaptive-range-change-delay"].as<unsigned>();
    if (opts.count("adaptive-range-scan-delay"))
        adaptive_params.range_scan_delay = opts["adaptive-range-scan-delay"].as<unsigned>();
    if (opts.count("adaptive-range-rescan-delay"))
        adaptive_params.range_rescan_delay = opts["adaptive-range-rescan-delay"].as<unsigned>();
    if (opts.count("adaptive-burst-alpha"))
        adaptive_params.burst_alpha = opts["adaptive-burst-alpha"].as<double>();
    if (opts.count("adaptive-burst-change-delay"))
        adaptive_params.burst_change_delay = opts["adaptive-burst-change-delay"].as<unsigned>();
    if (opts.count("adaptive-burst-loud-rate"))
        adaptive_params.burst_loud_rate = opts["adaptive-burst-loud-rate"].as<double>();
    if (opts.count("adaptive-burst-loud-runlength"))
        adaptive_params.burst_loud_runlength = opts["adaptive-burst-loud-runlength"].as<unsigned>();
    if (opts.count("adaptive-burst-quiet-rate"))
        adaptive_params.burst_quiet_rate = opts["adaptive-burst-quiet-rate"].as<double>();
    if (opts.count("adaptive-burst-quiet-runlength"))
        adaptive_params.burst_quiet_runlength = opts["adaptive-burst-quiet-runlength"].as<unsigned>();

    if (opts.count("stdin")) {
        sample_source = StdinSampleSource::Create(io_context, opts);
    } else if (opts.count("file")) {
        boost::filesystem::path path(opts["file"].as<std::string>());
        sample_source = FileSampleSource::Create(io_context, path, opts);
    } else if (opts.count("sdr")) {
        auto device = opts["sdr"].as<std::string>();
        soapy_source = SoapySampleSource::Create(io_context, device, opts);
        sample_source = soapy_source;
    } else if (opts.count("stratuxv3")) {
        auto path = opts["stratuxv3"].as<std::string>();
        message_source = StratuxSerial::Create(io_context, path);
    } else {
        assert("impossible case" && false);
    }

    auto create_output_port = [&](std::string option, SocketListener::ConnectionFactory factory) -> bool {
        if (!opts.count(option)) {
            return true;
        }

        bool ok = true;
        for (auto l : opts[option].as<std::vector<listen_option>>()) {
            boost::system::error_code ec;
            auto range = resolver.resolve(tcp::v4(), l.host, l.port, tcp::resolver::passive);
            if (!ec) {
                bool success = false;
                for (auto i = range.begin(); i != range.end(); ++i) {
                    const auto &endpoint = i->endpoint();

                    try {
                        auto listener = SocketListener::Create(io_context, endpoint, dispatch, factory);
                        listener->Start();
                        std::cerr << option << ": listening for connections on " << endpoint << std::endl;
                        success = true;
                    } catch (boost::system::system_error &err) {
                        std::cerr << option << ": could not listen on " << endpoint << ": " << err.what() << std::endl;
                        ec = err.code();
                    }
                }
                if (!success) {
                    std::cerr << option << ": no available listening addresses" << std::endl;
                    ok = false;
                }
            } else {
                std::cerr << option << ": failed to resolve " << l.host << ":" << l.port << ": " << ec.message() << std::endl;
                ok = false;
            }
        }

        return ok;
    };

    // Emit initial metadata-only message advertising our version etc
    SharedMessageVector header;
    if (!opts.count("raw-disable-header")) {
        header = std::make_shared<MessageVector>();

        // clang-format off
        RawMessage::MetadataMap metadata = {
            {"program", "dump978-fa"},
            {"version", VERSION},
            {"fecfix", "1"} // we have a fixed version of the FEC library
        };
        // clang-format on
        header->push_back(RawMessage(std::move(metadata)));
    }

    auto raw_factory = std::bind(&RawOutput::Create, std::placeholders::_1, std::placeholders::_2, header);
    auto raw_ok = create_output_port("raw-port", raw_factory);

    auto raw_legacy_factory = std::bind(&RawOutput::Create, std::placeholders::_1, std::placeholders::_2, SharedMessageVector());
    auto raw_legacy_ok = create_output_port("raw-legacy-port", raw_legacy_factory);

    auto json_ok = create_output_port("json-port", &JsonOutput::Create);
    if (!raw_ok || !raw_legacy_ok || !json_ok) {
        return 1;
    }

    if (opts.count("raw-stdout")) {
        dispatch.AddClient([](SharedMessageVector messages) {
            for (const auto &message : *messages) {
                std::cout << message << std::endl;
            }
        });
    }

    if (opts.count("json-stdout")) {
        dispatch.AddClient([](SharedMessageVector messages) {
            for (const auto &message : *messages) {
                if (message.Type() == MessageType::DOWNLINK_SHORT || message.Type() == MessageType::DOWNLINK_LONG) {
                    std::cout << AdsbMessage(message).ToJson() << std::endl;
                }
            }
        });
    }

    bool saw_error = false;

    if (sample_source) {
        sample_source->Init();
        auto format = sample_source->Format();

        AdaptiveGainControl::Pointer adaptive;
        if (soapy_source && (adaptive_params.range_control || adaptive_params.burst_control)) {
            adaptive = AdaptiveGainControl::Create(adaptive_params, soapy_source, 2083333);
            if (!adaptive->Enabled()) {
                adaptive.reset();
            }
        }

        auto receiver = std::make_shared<SingleThreadReceiver>(format, adaptive);

        sample_source->SetConsumer([receiver](std::uint64_t timestamp, const Bytes &buffer) { receiver->HandleSamples(timestamp, buffer.begin(), buffer.end()); });

        sample_source->SetErrorHandler(std::bind(&SingleThreadReceiver::HandleError, receiver, std::placeholders::_1));

        message_source = std::static_pointer_cast<MessageSource>(receiver);
    }

    message_source->SetConsumer(std::bind(&MessageDispatch::Dispatch, &dispatch, std::placeholders::_1));
    message_source->SetErrorHandler([&io_context, &saw_error](const boost::system::error_code &ec) {
        if (ec == boost::asio::error::eof) {
            std::cerr << "Message source reports EOF" << std::endl;
        } else {
            std::cerr << "Message source reports error: " << ec.message() << std::endl;
            saw_error = true;
        }
        io_context.stop();
    });

    boost::asio::signal_set signals(io_context, SIGINT, SIGTERM);
    signals.async_wait([&io_context, &saw_error](const boost::system::error_code &ec, int signum) {
        std::cerr << "Caught signal " << signum << ", exiting" << std::endl;
        saw_error = true;
        io_context.stop();
    });

    message_source->Start();
    if (sample_source) {
        sample_source->Start();
    }

    io_context.run();

    if (sample_source) {
        sample_source->Stop();
    }
    message_source->Stop();

    if (saw_error) {
        std::cerr << "Abnormal exit" << std::endl;
        return 1;
    } else {
        std::cerr << "Ran out of things to do, exiting" << std::endl;
        return 0;
    }
}

int main(int argc, char **argv) {
    try {
        return realmain(argc, argv);
    } catch (const config_error &e) {
        std::cerr << "Configuration error: " << e.what() << std::endl;
        return 2;
#ifndef DEBUG_EXCEPTIONS
    } catch (...) {
        std::cerr << "Uncaught exception: " << boost::current_exception_diagnostic_information() << std::endl;
        return 2;
#endif
    }
}
