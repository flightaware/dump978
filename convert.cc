// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "convert.h"

#include <assert.h>
#include <cmath>
#include <stdexcept>

using namespace flightaware::uat;

static inline std::uint16_t scaled_atan2(double y, double x) {
    double ang = std::atan2(y, x);
    if (ang < 0) {
        // atan2 returns [-pi..pi], normalize to [0..2*pi]
        ang += 2 * M_PI;
    }
    double scaled_ang = std::round(32768 * ang / M_PI);
    return scaled_ang < 0 ? 0 : scaled_ang > 65535 ? 65535 : (std::uint16_t)scaled_ang;
}

static inline std::uint16_t scaled_atan(double x) {
    double ang = std::atan(x);
    if (ang < 0) {
        // atan returns [-pi/2..pi/2], normalize to [0..2*pi]
        ang += 2 * M_PI;
    }
    double scaled_ang = std::round(32768 * ang / M_PI);
    return scaled_ang < 0 ? 0 : scaled_ang > 65535 ? 65535 : (std::uint16_t)scaled_ang;
}

static inline double magsq(double i, double q) { return i * i + q * q; }

SampleConverter::Pointer SampleConverter::Create(SampleFormat format) {
    switch (format) {
    case SampleFormat::CU8:
        return Pointer(new CU8Converter());
    case SampleFormat::CS8_:
        return Pointer(new CS8Converter());
    case SampleFormat::CS16H:
        return Pointer(new CS16HConverter());
    case SampleFormat::CF32H:
        return Pointer(new CF32HConverter());
    default:
        throw std::runtime_error("format not implemented yet");
    }
}

CU8Converter::CU8Converter() : SampleConverter(SampleFormat::CU8) {

    cu8_alias u;

    unsigned i, q;
    for (i = 0; i < 256; ++i) {
        double d_i = (i - 127.5) / 128.0;
        for (q = 0; q < 256; ++q) {
            double d_q = (q - 127.5) / 128.0;
            u.iq[0] = i;
            u.iq[1] = q;
            lookup_phase_[u.iq16] = scaled_atan2(d_q, d_i);
            lookup_magsq_[u.iq16] = magsq(d_i, d_q);
        }
    }
}

void CU8Converter::ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) {
    const cu8_alias *in_iq = reinterpret_cast<const cu8_alias *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 2;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 8) {
        *out++ = lookup_phase_[in_iq[0].iq16];
        *out++ = lookup_phase_[in_iq[1].iq16];
        *out++ = lookup_phase_[in_iq[2].iq16];
        *out++ = lookup_phase_[in_iq[3].iq16];
        *out++ = lookup_phase_[in_iq[4].iq16];
        *out++ = lookup_phase_[in_iq[5].iq16];
        *out++ = lookup_phase_[in_iq[6].iq16];
        *out++ = lookup_phase_[in_iq[7].iq16];
    }
    for (auto i = 0; i < n7; ++i, ++in_iq) {
        *out++ = lookup_phase_[in_iq[0].iq16];
    }
}

void CU8Converter::ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) {
    const cu8_alias *in_iq = reinterpret_cast<const cu8_alias *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 2;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 8) {
        *out++ = lookup_magsq_[in_iq[0].iq16];
        *out++ = lookup_magsq_[in_iq[1].iq16];
        *out++ = lookup_magsq_[in_iq[2].iq16];
        *out++ = lookup_magsq_[in_iq[3].iq16];
        *out++ = lookup_magsq_[in_iq[4].iq16];
        *out++ = lookup_magsq_[in_iq[5].iq16];
        *out++ = lookup_magsq_[in_iq[6].iq16];
        *out++ = lookup_magsq_[in_iq[7].iq16];
    }
    for (auto i = 0; i < n7; ++i, ++in_iq) {
        *out++ = lookup_magsq_[in_iq[0].iq16];
    }
}

CS8Converter::CS8Converter() : SampleConverter(SampleFormat::CS8_) {
    cs8_alias u;

    int i, q;
    for (i = -128; i <= 127; ++i) {
        double d_i = i / 128.0;
        for (q = -128; q <= 127; ++q) {
            double d_q = q / 128.0;
            u.iq[0] = i;
            u.iq[1] = q;
            lookup_phase_[u.iq16] = scaled_atan2(d_q, d_i);
            lookup_magsq_[u.iq16] = magsq(d_i, d_q);
        }
    }
}

void CS8Converter::ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) {
    auto in_iq = reinterpret_cast<const cs8_alias *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 2;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 8) {
        *out++ = lookup_phase_[in_iq[0].iq16];
        *out++ = lookup_phase_[in_iq[1].iq16];
        *out++ = lookup_phase_[in_iq[2].iq16];
        *out++ = lookup_phase_[in_iq[3].iq16];
        *out++ = lookup_phase_[in_iq[4].iq16];
        *out++ = lookup_phase_[in_iq[5].iq16];
        *out++ = lookup_phase_[in_iq[6].iq16];
        *out++ = lookup_phase_[in_iq[7].iq16];
    }
    for (auto i = 0; i < n7; ++i, ++in_iq) {
        *out++ = lookup_phase_[in_iq[0].iq16];
    }
}

void CS8Converter::ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) {
    auto in_iq = reinterpret_cast<const cs8_alias *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 2;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 8) {
        *out++ = lookup_magsq_[in_iq[0].iq16];
        *out++ = lookup_magsq_[in_iq[1].iq16];
        *out++ = lookup_magsq_[in_iq[2].iq16];
        *out++ = lookup_magsq_[in_iq[3].iq16];
        *out++ = lookup_magsq_[in_iq[4].iq16];
        *out++ = lookup_magsq_[in_iq[5].iq16];
        *out++ = lookup_magsq_[in_iq[6].iq16];
        *out++ = lookup_magsq_[in_iq[7].iq16];
    }
    for (auto i = 0; i < n7; ++i, ++in_iq) {
        *out++ = lookup_magsq_[in_iq[0].iq16];
    }
}

CS16HConverter::CS16HConverter() : SampleConverter(SampleFormat::CS16H) {
    // atan lookup, positive values only, 8-bit fixed point covering 0.0 .. 256.0
    for (std::size_t i = 0; i < lookup_atan_.size(); ++i) {
        lookup_atan_[i] = scaled_atan(i / 256.0);
    }
}

// caution, expects unsigned (positive) input only
inline std::uint16_t CS16HConverter::TableAtan(std::uint32_t r) {
    if (r > lookup_atan_.size())
        return 16384; // pi/2
    else
        return lookup_atan_[r];
}

inline std::uint16_t CS16HConverter::TableAtan2(std::int16_t y, std::int16_t x) {
    // atan2 using the atan lookup table
    // we rely on unsigned 16-bit integer overflow/wrap semantics
    // max error is about 0.2 degrees
    if (x == 0) {
        if (y >= 0) {
            return 16384; // pi/2
        } else {
            return 49152; // 3/2 pi
        }
    }

    const std::int32_t r = (std::int32_t)(256 * y) / x;
    if (x < 0) {
        if (y < 0) {
            // x < 0, y < 0   => y/x > 0
            // atan2(y,x) = pi + atan(y/x)
            return (std::uint16_t)32768 + TableAtan((std::uint32_t)r);
        } else {
            // x < 0, y >= 0  => y/x <= 0
            // atan2(y,x) = -pi + atan(y/x) = -pi - atan(-y/x)
            return (std::uint16_t)32768 - TableAtan((std::uint32_t)-r);
        }
    } else {
        if (y < 0) {
            // x > 0, y < 0   => y/x < 0
            // atan2(y,x) = atan(y/x) = -atan(-y/x)
            return (std::uint16_t)0 - TableAtan((std::uint32_t)-r);
        } else {
            // x > 0, y >= 0  => y/x >= 0
            // atan2(y,x) = atan(y/x)
            return TableAtan((std::uint32_t)r);
        }
    }
}

void CS16HConverter::ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) {
    auto in_iq = reinterpret_cast<const std::int16_t *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 4;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 16) {
        *out++ = TableAtan2(in_iq[1], in_iq[0]);
        *out++ = TableAtan2(in_iq[3], in_iq[2]);
        *out++ = TableAtan2(in_iq[5], in_iq[4]);
        *out++ = TableAtan2(in_iq[7], in_iq[6]);
        *out++ = TableAtan2(in_iq[9], in_iq[8]);
        *out++ = TableAtan2(in_iq[11], in_iq[10]);
        *out++ = TableAtan2(in_iq[13], in_iq[12]);
        *out++ = TableAtan2(in_iq[15], in_iq[14]);
    }
    for (auto i = 0; i < n7; ++i, in_iq += 2) {
        *out++ = TableAtan2(in_iq[1], in_iq[0]);
    }
}

void CS16HConverter::ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) {
    auto in_iq = reinterpret_cast<const std::int16_t *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 4;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 16) {
        *out++ = magsq(in_iq[1], in_iq[0]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[3], in_iq[2]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[5], in_iq[4]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[7], in_iq[6]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[9], in_iq[8]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[11], in_iq[10]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[13], in_iq[12]) / 32768.0 / 32768.0;
        *out++ = magsq(in_iq[15], in_iq[14]) / 32768.0 / 32768.0;
    }
    for (auto i = 0; i < n7; ++i, in_iq += 2) {
        *out++ = magsq(in_iq[1], in_iq[0]) / 32768.0 / 32768.0;
    }
}

void CF32HConverter::ConvertPhase(Bytes::const_iterator begin, Bytes::const_iterator end, PhaseBuffer::iterator out) {
    auto in_iq = reinterpret_cast<const float *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 8;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 16) {
        *out++ = scaled_atan2(in_iq[1], in_iq[0]);
        *out++ = scaled_atan2(in_iq[3], in_iq[2]);
        *out++ = scaled_atan2(in_iq[5], in_iq[4]);
        *out++ = scaled_atan2(in_iq[7], in_iq[6]);
        *out++ = scaled_atan2(in_iq[9], in_iq[8]);
        *out++ = scaled_atan2(in_iq[11], in_iq[10]);
        *out++ = scaled_atan2(in_iq[13], in_iq[12]);
        *out++ = scaled_atan2(in_iq[15], in_iq[14]);
    }
    for (auto i = 0; i < n7; ++i, in_iq += 2) {
        *out++ = scaled_atan2(in_iq[1], in_iq[0]);
    }
}

void CF32HConverter::ConvertMagSq(Bytes::const_iterator begin, Bytes::const_iterator end, std::vector<double>::iterator out) {
    auto in_iq = reinterpret_cast<const float *>(&*begin);

    // unroll the loop
    const auto n = std::distance(begin, end) / 8;
    const auto n8 = n / 8;
    const auto n7 = n & 7;

    for (auto i = 0; i < n8; ++i, in_iq += 16) {
        *out++ = magsq(in_iq[1], in_iq[0]);
        *out++ = magsq(in_iq[3], in_iq[2]);
        *out++ = magsq(in_iq[5], in_iq[4]);
        *out++ = magsq(in_iq[7], in_iq[6]);
        *out++ = magsq(in_iq[9], in_iq[8]);
        *out++ = magsq(in_iq[11], in_iq[10]);
        *out++ = magsq(in_iq[13], in_iq[12]);
        *out++ = magsq(in_iq[15], in_iq[14]);
    }
    for (auto i = 0; i < n7; ++i, in_iq += 2) {
        *out++ = magsq(in_iq[1], in_iq[0]);
    }
}
