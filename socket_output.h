// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef SOCKET_OUTPUT_H
#define SOCKET_OUTPUT_H

#include <memory>
#include <sstream>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>

#include "message_dispatch.h"
#include "uat_message.h"

namespace flightaware {
  namespace uat {
    class SocketOutput : public std::enable_shared_from_this<SocketOutput> {
      public:
        typedef std::shared_ptr<SocketOutput> Pointer;

        virtual void Start();
        void Write(SharedMessageVector messages);
        virtual void Close();

        void SetCloseNotifier(std::function<void()> notifier) { close_notifier_ = notifier; }

        bool IsOpen() const { return socket_.is_open(); }

        virtual ~SocketOutput() {};

      protected:
        SocketOutput(boost::asio::io_context &service_, boost::asio::ip::tcp::socket &&socket_);
        std::ostringstream &Buf() { return outbuf_; }

        virtual void InternalWrite(SharedMessageVector messages) = 0;

      private:
        void HandleError(const boost::system::error_code &ec);
        void Flush();
        void ReadAndDiscard();

        boost::asio::io_context::strand strand_;
        boost::asio::ip::tcp::socket socket_;
        boost::asio::ip::tcp::endpoint peer_;

        std::ostringstream outbuf_;
        bool flush_pending_;

        std::function<void()> close_notifier_;
    };

    class RawOutput : public SocketOutput {
      public:
        // factory method, this class must always be constructed via make_shared
        static Pointer Create(boost::asio::io_context &service, boost::asio::ip::tcp::socket &&socket, SharedMessageVector header) { return Pointer(new RawOutput(service, std::move(socket), header)); }

        void Start() override;

      protected:
        void InternalWrite(SharedMessageVector messages) override;

      private:
        RawOutput(boost::asio::io_context &service_, boost::asio::ip::tcp::socket &&socket_, SharedMessageVector header) : SocketOutput(service_, std::move(socket_)) { header_ = header; }

        SharedMessageVector header_;
    };

    class JsonOutput : public SocketOutput {
      public:
        // factory method, this class must always be constructed via make_shared
        static Pointer Create(boost::asio::io_context &service, boost::asio::ip::tcp::socket &&socket) { return Pointer(new JsonOutput(service, std::move(socket))); }

      protected:
        void InternalWrite(SharedMessageVector messages) override;

      private:
        JsonOutput(boost::asio::io_context &service_, boost::asio::ip::tcp::socket &&socket_) : SocketOutput(service_, std::move(socket_)) {}
    };

    class SocketListener : public std::enable_shared_from_this<SocketListener> {
      public:
        typedef std::shared_ptr<SocketListener> Pointer;
        typedef std::function<SocketOutput::Pointer(boost::asio::io_context &, boost::asio::ip::tcp::socket &&)> ConnectionFactory;

        // factory method, this class must always be constructed via make_shared
        static Pointer Create(boost::asio::io_context &service, const boost::asio::ip::tcp::endpoint &endpoint, MessageDispatch &dispatch, ConnectionFactory factory) { return Pointer(new SocketListener(service, endpoint, dispatch, factory)); }

        void Start();
        void Close();

      private:
        SocketListener(boost::asio::io_context &service, const boost::asio::ip::tcp::endpoint &endpoint, MessageDispatch &dispatch, ConnectionFactory factory);

        void Accept();

        boost::asio::io_context &service_;
        boost::asio::ip::tcp::acceptor acceptor_;
        boost::asio::ip::tcp::endpoint endpoint_;
        boost::asio::ip::tcp::socket socket_;
        boost::asio::ip::tcp::endpoint peer_;
        MessageDispatch &dispatch_;
        ConnectionFactory factory_;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
