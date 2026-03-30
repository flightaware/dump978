// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include <iomanip>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/asio/ip/v6_only.hpp>

#include "message_dispatch.h"
#include "socket_output.h"

namespace asio = boost::asio;
using boost::asio::ip::tcp;

using namespace flightaware::uat;

SocketOutput::SocketOutput(asio::io_service &service, tcp::socket &&socket) : strand_(service), socket_(std::move(socket)), peer_(socket_.remote_endpoint()), flush_pending_(false) {}

void SocketOutput::Start() { ReadAndDiscard(); }

void SocketOutput::ReadAndDiscard() {
    auto self(shared_from_this());
    auto buf = std::make_shared<Bytes>(512);
    socket_.async_read_some(asio::buffer(*buf), strand_.wrap([this, self, buf](const boost::system::error_code &ec, std::size_t len) {
        if (ec) {
            HandleError(ec);
        } else {
            ReadAndDiscard();
        }
    }));
}

void SocketOutput::Write(SharedMessageVector messages) {
    auto self(shared_from_this());
    strand_.dispatch([this, self, messages]() {
        if (IsOpen()) {
            InternalWrite(messages);
            Flush();
        }
    });
}

void SocketOutput::Flush() {
    if (flush_pending_)
        return;

    auto writebuf = std::make_shared<std::string>(outbuf_.str());
    if (writebuf->empty())
        return;

    flush_pending_ = true;
    outbuf_.str(std::string());

    auto self(shared_from_this());
    async_write(socket_, boost::asio::buffer(*writebuf), strand_.wrap([this, self, writebuf](const boost::system::error_code &ec, size_t len) {
        flush_pending_ = false;
        if (ec) {
            HandleError(ec);
            return;
        }

        Flush(); // maybe some more data arrived
    }));
}

void SocketOutput::HandleError(const boost::system::error_code &ec) {
    if (ec == boost::asio::error::eof) {
        std::cerr << peer_ << ": connection closed" << std::endl;
    } else if (ec != boost::asio::error::operation_aborted) {
        std::cerr << peer_ << ": connection error: " << ec.message() << std::endl;
    }

    Close();
}

void SocketOutput::Close() {
    socket_.close();
    if (close_notifier_) {
        close_notifier_();
    }
}

//////////////

void RawOutput::Start() {
    SocketOutput::Start();
    if (header_)
        Write(header_);
}

void RawOutput::InternalWrite(SharedMessageVector messages) {
    for (const auto &message : *messages) {
        Buf() << message << '\n';
    }
}

//////////////

void JsonOutput::InternalWrite(SharedMessageVector messages) {
    for (const auto &message : *messages) {
        if (message.Type() == MessageType::DOWNLINK_SHORT || message.Type() == MessageType::DOWNLINK_LONG) {
            Buf() << AdsbMessage(message).ToJson() << '\n';
        }
    }
}

//////////////

SocketListener::SocketListener(asio::io_service &service, const tcp::endpoint &endpoint, MessageDispatch &dispatch, ConnectionFactory factory) : service_(service), acceptor_(service), endpoint_(endpoint), socket_(service), dispatch_(dispatch), factory_(factory) {}

void SocketListener::Start() {
    acceptor_.open(endpoint_.protocol());
    acceptor_.set_option(asio::socket_base::reuse_address(true));
    acceptor_.set_option(tcp::acceptor::reuse_address(true));

    // We are v6 aware and bind separately to v4 and v6 addresses
    if (endpoint_.protocol() == tcp::v6())
        acceptor_.set_option(asio::ip::v6_only(true));

    acceptor_.bind(endpoint_);
    acceptor_.listen();
    Accept();
}

void SocketListener::Close() {
    acceptor_.cancel();
    socket_.close();
}

void SocketListener::Accept() {
    auto self(shared_from_this());

    acceptor_.async_accept(socket_, peer_, [this, self](const boost::system::error_code &ec) {
        if (!ec) {
            std::cerr << endpoint_ << ": accepted a connection from " << peer_ << std::endl;
            auto new_output = factory_(service_, std::move(socket_));
            if (new_output) {
                auto handle = dispatch_.AddClient(std::bind(&SocketOutput::Write, new_output, std::placeholders::_1));
                new_output->SetCloseNotifier([this, self, handle] { dispatch_.RemoveClient(handle); });
                new_output->Start();
            }
        } else {
            if (ec == boost::system::errc::operation_canceled)
                return;
            std::cerr << endpoint_ << ": accept error: " << ec.message() << std::endl;
        }

        Accept();
    });
}
