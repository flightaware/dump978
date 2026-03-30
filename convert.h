// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef DUMP978_CONVERT_H
#define DUMP978_CONVERT_H

#include <memory>
#include <array>

#include "common.h"

namespace flightaware {
  namespace uat {
    // Describes a sample data layout:
    //   CU8   - interleaved I/Q data, 8 bit unsigned integers
    //   CS8_  - interleaved I/Q data, 8 bit signed integers
    //   CS16H - interleaved I/Q data, 16 bit signed integers, host byte order
    //   CF32H - interleaved I/Q data, 32 bit signed floats, host byte order
    enum class SampleFormat { CU8, CS8_, CS16H, CF32H, UNKNOWN };

    // Return the number of bytes for 1 sample in the given format
    inline static unsigned BytesPerSample(SampleFormat f) {
        switch (f) {
        case SampleFormat::CU8:
            return 2;
        case SampleFormat::CS8_:
            return 2;
        case SampleFormat::CS16H:
            return 4;
        case SampleFormat::CF32H:
            return 8;
        default:
            return 0;
        }
    }

    // Base class for all sample converters.
    // Use SampleConverter::Create to build converters.
    class SampleConverter {
      public:
        typedef std::shared_ptr<SampleConverter> Pointer;

        SampleConverter(SampleFormat format) : format_(format), bytes_per_sample_(flightaware::uat::BytesPerSample(format)) {}

        virtual ~SampleConverter() {}

        // Read samples from `begin` .. `end` and write one phase value per sample to
        // `out`. The input buffer should contain an integral number of samples
        // (trailing partial samples are ignored, not buffered).
        virtual void ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) = 0;

        // Read samples from `begin` .. `end` and write one magnitude-squared value
        // per sample to `out`. The input buffer should contain an integral number of
        // samples (trailing partial samples are ignored, not buffered).
        virtual void ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) = 0;

        SampleFormat Format() const { return format_; }
        unsigned BytesPerSample() const { return bytes_per_sample_; }

        // Return a new SampleConverter that converts from the given format
        static Pointer Create(SampleFormat format);

      private:
        SampleFormat format_;
        unsigned bytes_per_sample_;
    };

    class CU8Converter : public SampleConverter {
      public:
        CU8Converter();

        void ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) override;
        void ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) override;

      private:
        union cu8_alias {
            std::uint8_t iq[2];
            std::uint16_t iq16;
        };

        std::array<std::uint16_t, 65536> lookup_phase_;
        std::array<double, 65536> lookup_magsq_;
    };

    class CS8Converter : public SampleConverter {
      public:
        CS8Converter();

        void ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) override;
        void ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) override;

      private:
        union cs8_alias {
            std::int8_t iq[2];
            std::uint16_t iq16;
        };

        std::array<std::uint16_t, 65536> lookup_phase_;
        std::array<double, 65536> lookup_magsq_;
    };

    class CS16HConverter : public SampleConverter {
      public:
        CS16HConverter();
        void ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) override;
        void ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) override;

      private:
        std::uint16_t TableAtan(std::uint32_t r);
        std::uint16_t TableAtan2(std::int16_t y, std::int16_t x);
        std::array<std::uint16_t, 65536> lookup_atan_;
    };

    class CF32HConverter : public SampleConverter {
      public:
        CF32HConverter() : SampleConverter(SampleFormat::CF32H) {}
        void ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) override;
        void ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) override;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
