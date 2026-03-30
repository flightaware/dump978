// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_DEMODULATOR_H
#define DUMP978_DEMODULATOR_H

#include <functional>
#include <vector>

#include "common.h"
#include "convert.h"
#include "fec.h"
#include "message_source.h"
#include "uat_message.h"

namespace flightaware {
  namespace uat {
    class Demodulator {
      public:
        // Return value of Demodulate
        struct Message {
            Bytes payload;
            unsigned corrected_errors;
            PhaseBuffer::const_iterator begin;
            PhaseBuffer::const_iterator end;
        };

        virtual ~Demodulator() {}
        virtual std::vector<Message> Demodulate(PhaseBuffer::const_iterator begin, PhaseBuffer::const_iterator end) = 0;

        virtual unsigned NumTrailingSamples() = 0;

      protected:
        FEC fec_;
    };

    class TwoMegDemodulator : public Demodulator {
      public:
        std::vector<Message> Demodulate(PhaseBuffer::const_iterator begin, PhaseBuffer::const_iterator end) override;
        unsigned NumTrailingSamples() override;

      private:
        boost::optional<Message> DemodBest(PhaseBuffer::const_iterator begin, bool downlink);
        boost::optional<Message> DemodOneDownlink(PhaseBuffer::const_iterator begin);
        boost::optional<Message> DemodOneUplink(PhaseBuffer::const_iterator begin);
    };

    class Receiver : public MessageSource {
      public:
        virtual void HandleSamples(std::uint64_t timestamp, Bytes::const_iterator begin, Bytes::const_iterator end) = 0;

        virtual void HandleError(const boost::system::error_code &ec) { DispatchError(ec); }
    };

    class SingleThreadReceiver : public Receiver {
      public:
        SingleThreadReceiver(SampleFormat format);

        void HandleSamples(std::uint64_t timestamp, Bytes::const_iterator begin, Bytes::const_iterator end) override;

      private:
        SampleConverter::Pointer converter_;
        std::unique_ptr<Demodulator> demodulator_;

        Bytes samples_;
        std::size_t saved_samples_ = 0;

        PhaseBuffer phase_;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
