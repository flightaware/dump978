// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef MESSAGE_DISPATCH_H
#define MESSAGE_DISPATCH_H

#include <atomic>
#include <functional>
#include <map>
#include <mutex>

#include "uat_message.h"

namespace flightaware {
  namespace uat {
    class MessageDispatch {
      public:
        typedef unsigned Handle;
        typedef std::function<void(SharedMessageVector)> MessageHandler;

        MessageDispatch();
        MessageDispatch(const MessageDispatch &) = delete;
        MessageDispatch &operator=(const MessageDispatch &) = delete;

        Handle AddClient(MessageHandler handler);
        void RemoveClient(Handle client);

        void Dispatch(SharedMessageVector messages);

      protected:
        void PurgeDeadClients();

      private:
        std::recursive_mutex mutex_;
        Handle next_handle_;
        unsigned busy_;

        struct Client {
            MessageHandler handler;
            bool deleted;
        };

        std::map<Handle, Client> clients_;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
