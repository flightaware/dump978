// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include <boost/exception/diagnostic_information.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <memory>

#include "faup978_reporter.h"
#include "message_source.h"
#include "socket_input.h"
#include "uat_message.h"

using namespace flightaware::uat;
using namespace flightaware::faup978;

namespace po = boost::program_options;
using boost::asio::ip::tcp;

struct connect_option {
    std::string host;
    std::string port;
};

// Specializations of validate for --connect
void validate(boost::any &v, const std::vector<std::string> &values, connect_option *target_type, int) {
    po::validators::check_first_occurrence(v);
    const std::string &s = po::validators::get_single_string(values);

    static const boost::regex r("(?:([^:]+):)?(\\d+)");
    boost::smatch match;
    if (boost::regex_match(s, match, r)) {
        v = boost::any(connect_option{match[1], match[2]});
    } else {
        throw po::validation_error(po::validation_error::invalid_option_value);
    }
}

#define EXIT_NO_RESTART (64)

static int realmain(int argc, char **argv) {
    boost::asio::io_context io_context;

    // clang-format off
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("version", "show version")
        ("connect", po::value<connect_option>(), "connect to host:port for raw UAT data");
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
        std::cerr << "faup978 " << VERSION << std::endl;
        std::cerr << desc << std::endl;
        return EXIT_NO_RESTART;
    }

    if (opts.count("version")) {
        std::cerr << "faup978 " << VERSION << std::endl;
        return EXIT_NO_RESTART;
    }

    if (!opts.count("connect")) {
        std::cerr << "--connect option is required" << std::endl;
        return EXIT_NO_RESTART;
    }

    auto connect = opts["connect"].as<connect_option>();
    auto input = RawInput::Create(io_context, connect.host, connect.port);
    auto reporter = Reporter::Create(io_context);

    input->SetConsumer(std::bind(&Reporter::HandleMessages, reporter, std::placeholders::_1));
    input->SetErrorHandler([&io_context](const boost::system::error_code &ec) {
        std::cerr << "Connection failed: " << ec.message() << std::endl;
        io_context.stop();
    });

    reporter->Start();
    input->Start();

    io_context.run();

    input->Stop();
    reporter->Stop();
    return 0;
}

int main(int argc, char **argv) {
#ifndef DEBUG_EXCEPTIONS
    try {
        return realmain(argc, argv);
    } catch (...) {
        std::cerr << "Uncaught exception: " << boost::current_exception_diagnostic_information() << std::endl;
        return 2;
    }
#else
    return realmain(argc, argv);
#endif
}
