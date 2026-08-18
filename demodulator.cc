// Copyright 2015, Oliver Jowett <oliver@mutability.co.uk>
// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "demodulator.h"

#include <assert.h>
#include <iomanip>
#include <iostream>

using namespace flightaware::uat;

SingleThreadReceiver::SingleThreadReceiver(SampleFormat format, AdaptiveGainControl::Pointer adaptive) : converter_(SampleConverter::Create(format)), demodulator_(new TwoMegDemodulator()), adaptive_(adaptive) {}

// Handle samples in 'buffer' by:
//   converting them to a phase buffer
//   demodulating the phase buffer
//   dispatching any demodulated messages
//   preserving the end of the phase buffer for reuse in the next call
void SingleThreadReceiver::HandleSamples(std::uint64_t timestamp, Bytes::const_iterator begin, Bytes::const_iterator end) {
    assert(converter_);

    const auto buffer_bytes = std::distance(begin, end);
    const auto buffer_samples = buffer_bytes / converter_->BytesPerSample();

    const auto previous_samples = saved_samples_;
    const auto previous_bytes = previous_samples * converter_->BytesPerSample();

    const auto total_samples = buffer_samples + previous_samples;
    const auto total_bytes = total_samples * converter_->BytesPerSample();

    if (samples_.size() < total_bytes) {
        samples_.resize(total_bytes);
    }

    // TODO: rearrange things to avoid this copy
    std::copy(begin, end, samples_.begin() + previous_bytes);

    if (phase_.size() < total_samples) {
        phase_.resize(total_samples);
    }

    converter_->ConvertPhase(samples_.begin(), samples_.begin() + total_bytes, phase_.begin());
    auto messages = demodulator_->Demodulate(phase_.begin(), phase_.begin() + total_samples);

    // decoded message extents & mean power, for the adaptive gain controller
    struct DecodedSpan {
        std::size_t begin;
        std::size_t end;
        double power;
    };
    std::vector<DecodedSpan> decoded_spans;

    if (!messages.empty()) {
        SharedMessageVector dispatch = std::make_shared<MessageVector>();
        dispatch->reserve(messages.size());
        for (auto &message : messages) {
            std::vector<double> magsq;
            magsq.resize(std::distance(message.begin, message.end));

            auto begin_sample = samples_.begin() + std::distance(phase_.cbegin(), message.begin) * converter_->BytesPerSample();
            auto end_sample = samples_.begin() + std::distance(phase_.cbegin(), message.end) * converter_->BytesPerSample();

            converter_->ConvertMagSq(begin_sample, end_sample, magsq.begin());

            auto total_power = 0.0;
            for (auto m : magsq) {
                total_power += m;
            }

            auto mean_power = total_power / magsq.size();
            auto rssi = (total_power == 0 ? -1000 : 10 * std::log10(mean_power));
            std::uint64_t message_timestamp = timestamp - (1000 * previous_samples / 2083333) + (1000 * std::distance(phase_.cbegin(), message.begin) / 2083333);

            dispatch->emplace_back(std::move(message.payload), message_timestamp, message.corrected_errors, rssi);

            if (adaptive_) {
                decoded_spans.push_back(DecodedSpan{(std::size_t)std::distance(phase_.cbegin(), message.begin), (std::size_t)std::distance(phase_.cbegin(), message.end), mean_power});
            }
        }

        DispatchMessages(dispatch);
    }

    // Feed sample magnitudes to the adaptive gain controller. The last tail_size
    // samples are re-presented on the next call, so to feed every sample exactly
    // once we stop short of them here and pick them up next time; by then, any
    // message starting in that region has been demodulated, so decoded spans are
    // known when their samples are fed. Decoded message samples are fed with the
    // message's mean power; everything else is fed as undecoded samples. A
    // decoded message clipped at the feed boundary has its remaining samples
    // carried over and fed as decoded samples on the next call, so that burst
    // detection never sees part of a decoded message as an undecoded burst.
    if (adaptive_ && adaptive_->Enabled()) {
        const std::size_t tail = demodulator_->NumTrailingSamples();
        const std::size_t feed_end = ((std::size_t)total_samples > tail) ? (std::size_t)total_samples - tail : 0;
        if (feed_end > 0) {
            if (mag16_.size() < feed_end) {
                mag16_.resize(feed_end);
            }
            converter_->ConvertMag16(samples_.begin(), samples_.begin() + feed_end * converter_->BytesPerSample(), mag16_.begin());

            std::size_t cursor = 0;
            if (adaptive_carry_samples_ > 0) {
                // remainder of a decoded message clipped at the previous feed
                // boundary; it now sits at the start of the buffer
                auto carry_end = std::min(adaptive_carry_samples_, feed_end);
                adaptive_->Update(&mag16_[0], carry_end, &adaptive_carry_power_);
                cursor = carry_end;
                adaptive_carry_samples_ -= carry_end;
            }
            for (const auto &span : decoded_spans) {
                if (span.begin >= feed_end) {
                    break;
                }
                auto span_begin = std::max(span.begin, cursor);
                if (span_begin > cursor) {
                    adaptive_->Update(&mag16_[cursor], span_begin - cursor, nullptr);
                }
                auto span_end = std::min(span.end, feed_end);
                if (span_end > span_begin) {
                    adaptive_->Update(&mag16_[span_begin], span_end - span_begin, &span.power);
                    cursor = span_end;
                }
                if (span.end > feed_end) {
                    adaptive_carry_samples_ = span.end - feed_end;
                    adaptive_carry_power_ = span.power;
                }
            }
            if (cursor < feed_end) {
                adaptive_->Update(&mag16_[cursor], feed_end - cursor, nullptr);
            }
        }
    }

    // preserve the tail of the sample buffer for next time
    const auto tail_size = demodulator_->NumTrailingSamples();
    if (total_samples > tail_size) {
        std::copy(samples_.end() - tail_size * converter_->BytesPerSample(), samples_.end(), samples_.begin());
        saved_samples_ = tail_size;
    } else {
        saved_samples_ = total_samples;
    }
}

static inline std::int16_t PhaseDifference(std::uint16_t from, std::uint16_t to) {
    int32_t difference = to - from; // lies in the range -65535 .. +65535
    if (difference >= 32768)        //   +32768..+65535
        return difference - 65536;  //   -> -32768..-1: always in range
    else if (difference < -32768)   //   -65535..-32769
        return difference + 65536;  //   -> +1..32767: always in range
    else
        return difference;
}

static inline bool SyncWordMatch(std::uint64_t word, std::uint64_t expected) {
    std::uint64_t diff;

    if (word == expected)
        return 1;

    diff = word ^ expected; // guaranteed nonzero

    // This is a bit-twiddling popcount
    // hack, tweaked as we only care about
    // "<N" or ">=N" set bits for fixed N -
    // so we can bail out early after seeing N
    // set bits.
    //
    // It relies on starting with a nonzero value
    // with zero or more trailing clear bits
    // after the last set bit:
    //
    //    010101010101010000
    //                 ^
    // Subtracting one, will flip the
    // bits starting at the last set bit:
    //
    //    010101010101001111
    //                 ^
    // then we can use that as a bitwise-and
    // mask to clear the lowest set bit:
    //
    //    010101010101000000
    //                 ^
    // And repeat until the value is zero
    // or we have seen too many set bits.

    // >= 1 bit
    diff &= (diff - 1); // clear lowest set bit
    if (!diff)
        return 1; // 1 bit error

    // >= 2 bits
    diff &= (diff - 1); // clear lowest set bit
    if (!diff)
        return 1; // 2 bits error

    // >= 3 bits
    diff &= (diff - 1); // clear lowest set bit
    if (!diff)
        return 1; // 3 bits error

    // >= 4 bits
    diff &= (diff - 1); // clear lowest set bit
    if (!diff)
        return 1; // 4 bits error

    // > 4 bits in error, give up
    return 0;
}

#ifdef AUTO_CENTER
// check that there is a valid sync word starting at 'phase'
// that matches the sync word 'pattern'. Return a pair:
// first element is true if the sync word looks OK; second
// element has the dphi threshold to use for bit slicing
static inline std::pair<bool, std::int16_t> CheckSyncWord(PhaseBuffer::const_iterator phase, std::uint64_t pattern) {
    const unsigned MAX_SYNC_ERRORS = 4;

    std::int32_t dphi_zero_total = 0;
    int zero_bits = 0;
    std::int32_t dphi_one_total = 0;
    int one_bits = 0;

    // find mean dphi for zero and one bits;
    // take the mean of the two as our central value

    for (unsigned i = 0; i < SYNC_BITS; ++i) {
        auto dphi = PhaseDifference(phase[i * 2], phase[i * 2 + 1]);
        if (pattern & (1UL << (35 - i))) {
            ++one_bits;
            dphi_one_total += dphi;
        } else {
            ++zero_bits;
            dphi_zero_total += dphi;
        }
    }

    dphi_zero_total /= zero_bits;
    dphi_one_total /= one_bits;

    std::int16_t center = (dphi_one_total + dphi_zero_total) / 2;

    // recheck sync word using our center value
    unsigned error_bits = 0;
    for (unsigned i = 0; i < SYNC_BITS; ++i) {
        auto dphi = PhaseDifference(phase[i * 2], phase[i * 2 + 1]);

        if (pattern & (1UL << (35 - i))) {
            if (dphi < center)
                ++error_bits;
        } else {
            if (dphi > center)
                ++error_bits;
        }
    }

    return {(error_bits <= MAX_SYNC_ERRORS), center};
}
#endif

// demodulate 'bytes' bytes from samples at 'phase' using 'center' as the bit
// slicing threshold
static inline std::pair<Bytes, std::vector<std::size_t>> DemodBits(PhaseBuffer::const_iterator phase, unsigned bytes, std::int16_t zero_slice, std::int16_t one_slice) {
    std::pair<Bytes, std::vector<std::size_t>> result_pair;
    auto &result = result_pair.first;
    auto &erasures = result_pair.second;

    result.reserve(bytes);

    for (unsigned i = 0; i < bytes; ++i) {
        std::uint8_t b = 0;
        bool erasure = false;
        if (PhaseDifference(phase[0], phase[1]) > one_slice)
            b |= 0x80;
        else if (PhaseDifference(phase[0], phase[1]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[2], phase[3]) > one_slice)
            b |= 0x40;
        else if (PhaseDifference(phase[2], phase[3]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[4], phase[5]) > one_slice)
            b |= 0x20;
        else if (PhaseDifference(phase[4], phase[5]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[6], phase[7]) > one_slice)
            b |= 0x10;
        else if (PhaseDifference(phase[6], phase[7]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[8], phase[9]) > one_slice)
            b |= 0x08;
        else if (PhaseDifference(phase[8], phase[9]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[10], phase[11]) > one_slice)
            b |= 0x04;
        else if (PhaseDifference(phase[10], phase[11]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[12], phase[13]) > one_slice)
            b |= 0x02;
        else if (PhaseDifference(phase[12], phase[13]) > zero_slice)
            erasure = true;
        if (PhaseDifference(phase[14], phase[15]) > one_slice)
            b |= 0x01;
        else if (PhaseDifference(phase[14], phase[15]) > zero_slice)
            erasure = true;
        result.push_back(b);
        if (erasure)
            erasures.push_back(i);
        phase += 16;
    }

    return result_pair;
}

unsigned TwoMegDemodulator::NumTrailingSamples() { return (SYNC_BITS + UPLINK_BITS) * 2; }

// Try to demodulate messages from `begin` .. `end` and return a list of
// messages. Messages that start near the end of the range may not be
// demodulated (less than (SYNC_BITS + UPLINK_BITS)*2 before the end of the
// buffer)
std::vector<Demodulator::Message> TwoMegDemodulator::Demodulate(PhaseBuffer::const_iterator begin, PhaseBuffer::const_iterator end) {
    // We expect samples at twice the UAT bitrate.
    // We look at phase difference between pairs of adjacent samples, i.e.
    //  sample 1 - sample 0   -> sync0
    //  sample 2 - sample 1   -> sync1
    //  sample 3 - sample 2   -> sync0
    //  sample 4 - sample 3   -> sync1
    // ...
    //
    // We accumulate bits into two buffers, sync0 and sync1.
    // Then we compare those buffers to the expected 36-bit sync word that
    // should be at the start of each UAT frame. When (if) we find it,
    // that tells us which sample to start decoding from.

    // Stop when we run out of remaining samples for a max-sized frame.
    // Arrange for our caller to pass the trailing data back to us next time;
    // ensure we don't consume any partial sync word we might be part-way
    // through. This means we don't need to maintain state between calls.

    std::vector<Demodulator::Message> messages;

    const int trailing_samples = (SYNC_BITS + UPLINK_BITS) * 2;
    if (std::distance(begin, end) < trailing_samples) {
        return messages;
    }

    const auto limit = end - trailing_samples;

    unsigned sync_bits = 0;
    std::uint64_t sync0 = 0, sync1 = 0;
    const std::uint64_t SYNC_MASK = ((((std::uint64_t)1) << SYNC_BITS) - 1);

    for (auto probe = begin; probe < limit; probe += 2) {
        auto d0 = PhaseDifference(probe[0], probe[1]);
        auto d1 = PhaseDifference(probe[1], probe[2]);

        sync0 = ((sync0 << 1) | (d0 > 0 ? 1 : 0)) & SYNC_MASK;
        sync1 = ((sync1 << 1) | (d1 > 0 ? 1 : 0)) & SYNC_MASK;

        if (++sync_bits < SYNC_BITS)
            continue; // haven't fully populated sync0/1 yet

        // see if we have (the start of) a valid sync word
        // when we find a match, try to demodulate both with that match
        // and with the next position, and pick the one with fewer
        // errors.
        if (SyncWordMatch(sync0, DOWNLINK_SYNC_WORD)) {
            auto start = probe - SYNC_BITS * 2 + 2;
            auto message = DemodBest(start, true /* downlink */);
            if (message) {
                probe = message->end - 2;
                sync_bits = 0;
                messages.emplace_back(std::move(message.value()));
                continue;
            }
        }

        if (SyncWordMatch(sync1, DOWNLINK_SYNC_WORD)) {
            auto start = probe - SYNC_BITS * 2 + 3;
            auto message = DemodBest(start, true /* downlink */);
            if (message) {
                probe = message->end - 2;
                sync_bits = 0;
                messages.emplace_back(std::move(message.value()));
                continue;
            }
        }

        if (SyncWordMatch(sync0, UPLINK_SYNC_WORD)) {
            auto start = probe - SYNC_BITS * 2 + 2;
            auto message = DemodBest(start, false /* !downlink */);
            if (message) {
                probe = message->end - 2;
                sync_bits = 0;
                messages.emplace_back(std::move(message.value()));
                continue;
            }
        }

        if (SyncWordMatch(sync1, UPLINK_SYNC_WORD)) {
            auto start = probe - SYNC_BITS * 2 + 3;
            auto message = DemodBest(start, false /* !downlink */);
            if (message) {
                probe = message->end - 2;
                sync_bits = 0;
                messages.emplace_back(std::move(message.value()));
                continue;
            }
        }
    }

    return messages;
}

boost::optional<Demodulator::Message> TwoMegDemodulator::DemodBest(PhaseBuffer::const_iterator start, bool downlink) {
    auto message0 = downlink ? DemodOneDownlink(start) : DemodOneUplink(start);
    auto message1 = downlink ? DemodOneDownlink(start + 1) : DemodOneUplink(start + 1);

    if (!message0 && !message1)
        return boost::none;

    unsigned errors0 = (message0 ? message0->corrected_errors : 9999);
    unsigned errors1 = (message1 ? message1->corrected_errors : 9999);

    if (errors0 <= errors1)
        return message0; // should be move-eligible
    else
        return message1; // should be move-eligible
}

boost::optional<Demodulator::Message> TwoMegDemodulator::DemodOneDownlink(PhaseBuffer::const_iterator start) {
#ifdef AUTO_CENTER
    auto sync = CheckSyncWord(start, DOWNLINK_SYNC_WORD);
    if (!sync.first) {
        // Sync word had errors
        return boost::none;
    }

    auto result = DemodBits(start + SYNC_BITS * 2, DOWNLINK_LONG_BYTES, sync.second, sync.second);
#else
    auto result = DemodBits(start + SYNC_BITS * 2, DOWNLINK_LONG_BYTES, 0, 0);
#endif
    auto &raw = result.first;
    auto &erasures = result.second;

    bool success;
    Bytes corrected;
    unsigned errors;
    std::tie(success, corrected, errors) = fec_.CorrectDownlink(raw, erasures);
    if (!success) {
        // Error correction failed
        return boost::none;
    }

    auto bits = (corrected.size() == DOWNLINK_LONG_DATA_BYTES ? DOWNLINK_LONG_BITS : DOWNLINK_SHORT_BITS);
    return Demodulator::Message{std::move(corrected), errors, start, start + (SYNC_BITS + bits) * 2};
}

boost::optional<Demodulator::Message> TwoMegDemodulator::DemodOneUplink(PhaseBuffer::const_iterator start) {
#ifdef AUTO_CENTER
    auto sync = CheckSyncWord(start, UPLINK_SYNC_WORD);
    if (!sync.first) {
        // Sync word had errors
        return boost::none;
    }

    auto result = DemodBits(start + SYNC_BITS * 2, UPLINK_BYTES, sync.second, sync.second);
#else
    auto result = DemodBits(start + SYNC_BITS * 2, UPLINK_BYTES, 0, 0);
#endif
    auto &raw = result.first;
    auto &erasures = result.second;

    bool success;
    Bytes corrected;
    unsigned errors;
    std::tie(success, corrected, errors) = fec_.CorrectUplink(raw, erasures);

    if (!success) {
        // Error correction failed
        return boost::none;
    }

    return Demodulator::Message{std::move(corrected), errors, start, start + (SYNC_BITS + UPLINK_BITS) * 2};
}
