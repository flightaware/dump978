// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_MESSAGE_SOURCE_H
#define DUMP978_MESSAGE_SOURCE_H

#include <boost/system/error_code.hpp>

#include "uat_message.h"

namespace flightaware {
  namespace uat {
    class MessageSource {
      public:
        typedef std::shared_ptr<MessageSource> Pointer;
        typedef std::function<void(SharedMessageVector)> Consumer;
        typedef std::function<void(const boost::system::error_code &)> ErrorHandler;

        virtual ~MessageSource() {}

        void SetConsumer(Consumer consumer) { consumer_ = consumer; }
        void SetErrorHandler(ErrorHandler handler) { error_handler_ = handler; }

        virtual void Start() {}
        virtual void Stop() {}

      protected:
        void DispatchMessages(SharedMessageVector messages) {
            if (consumer_) {
                consumer_(messages);
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
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
