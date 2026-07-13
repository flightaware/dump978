// adaptive.cc: adaptive gain control
//
// Ported from dump1090's adaptive.c
// (https://github.com/flightaware/dump1090)
//
// Copyright (c) 2021 FlightAware, LLC
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "adaptive.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace flightaware::uat;

//
// block handling
//
// 1 block = approx 1 second of samples. Control updates are done at the end of each block only.
// Each block is made up of an integer number of subblocks (currently 20)
//
// 1 subblock = approx 50ms of samples. Duty cycle decisions are made at the subblock level;
// either the whole subblock is processed, or the whole subblock is skipped.
// Each subblock is made up of an integer number of windows (currently 1250)
//
// 1 window = approx 40us of samples. Burst measurements are made by counting samples within each window.
//
// All three levels are aligned, i.e. every block boundary is also a subblock boundary;
// every subblock boundary is also a window boundary.
//
// Duty cycle is expressed as N/D where D = kSubblocksPerBlock and
// N = round(kSubblocksPerBlock * duty_cycle); i.e. within each block, there are exactly
// N active subblocks out of D total subblocks. The active subblocks are distributed
// evenly across the block by increasing a counter by N on each subblock, modulo D, and
// marking the subblock as active each time the counter rolls over.

static const unsigned kSubblocksPerBlock = 20; // subblocks per block
static const unsigned kReportInterval = 60;    // blocks (~seconds) between periodic reports

// Return the dB value of gain step 'step', clamping out-of-range steps to the table bounds
double AdaptiveGainControl::StepDb(int step) const {
    if (gain_steps_.empty())
        return 0;
    if (step < 0)
        step = 0;
    if (step >= GainStepCount())
        step = GainStepCount() - 1;
    return gain_steps_[step];
}

// Return the gain step closest to 'db'
int AdaptiveGainControl::NearestStep(double db) const {
    int best = 0;
    for (int i = 1; i < GainStepCount(); ++i) {
        if (std::fabs(gain_steps_[i] - db) < std::fabs(gain_steps_[best] - db))
            best = i;
    }
    return best;
}

// Try to change the SDR gain to 'step' and tell the user about it,
// with 'why' as the reason to show. Return true if the gain actually changed.
bool AdaptiveGainControl::SetGainStep(int step, const char *why) {
    if (step < gain_min_)
        step = gain_min_;
    if (step > gain_max_)
        step = gain_max_;

    if (current_step_ == step)
        return false;

    fprintf(stderr, "adaptive: changing gain from %.1fdB (step %d) to %.1fdB (step %d) because: %s\n", StepDb(current_step_), current_step_, StepDb(step), step, why);

    double reported = sdr_->SetGainDb(StepDb(step));
    if (std::fabs(reported - StepDb(step)) >= 0.5)
        fprintf(stderr, "adaptive: device reports gain %.1fdB after requesting %.1fdB\n", reported, StepDb(step));

    current_step_ = step;
    ++report_gain_changes_;
    return true;
}

// Update internal state to reflect a gain change
// (usually after SetGainStep returns true, but also called during init)
void AdaptiveGainControl::GainChanged() {
    gain_up_db_ = StepDb(current_step_ + 1) - StepDb(current_step_);
    gain_down_db_ = StepDb(current_step_) - StepDb(current_step_ - 1);

    double loud_threshold_dbfs = 0 - gain_up_db_ - 3.0;
    burst_loud_threshold_ = std::pow(10, loud_threshold_dbfs / 10.0);

    range_change_timer_ = params_.range_change_delay;
    burst_change_timer_ = params_.burst_change_delay;
    burst_loud_blocks_ = 0;
    burst_quiet_blocks_ = 0;
}

void AdaptiveGainControl::IncreaseGain(const char *why) {
    if (SetGainStep(current_step_ + 1, why))
        GainChanged();
}

void AdaptiveGainControl::DecreaseGain(const char *why) {
    if (SetGainStep(current_step_ - 1, why))
        GainChanged();
}

AdaptiveGainControl::AdaptiveGainControl(const AdaptiveParams &params, SdrGainControl::Pointer sdr, unsigned sample_rate) : params_(params), sdr_(sdr) {
    double range_min = 0, range_max = 0, range_step = 0;
    sdr_->GetGainLimits(range_min, range_max, range_step);

    // If the SDR doesn't support gain control, disable ourselves
    if (!(range_max > range_min)) {
        if (params_.burst_control || params_.range_control) {
            fprintf(stderr, "warning: adaptive gain control requested, but SDR gain control not available, ignored.\n");
        }
        params_.burst_control = false;
        params_.range_control = false;
    }

    // If we're disabled, do nothing
    if (!params_.burst_control && !params_.range_control)
        return;

    // Build the gain step table. Unlike dump1090, which sees the tuner's discrete
    // gain table directly, SoapySDR reports gain as a min/max/step range; if the
    // device does not report a usable step size, probe in 1dB steps.
    double step_db = range_step;
    if (step_db <= 0 || (range_max - range_min) / step_db > 512)
        step_db = 1.0;
    int step_count = (int)((range_max - range_min) / step_db + 1e-6) + 1;
    for (int i = 0; i < step_count; ++i)
        gain_steps_.push_back(range_min + i * step_db);

    // Autoselect a target dynamic range if none was given
    // (matches dump1090's default for SoapySDR devices; approximately 30dB for an RTLSDR)
    if (params_.range_target <= 0)
        params_.range_target = (range_max - range_min) * 0.6;

    // Set up window, subblock, and block sizes
    // Look for 40us bursts
    samples_per_window_ = sample_rate / 25000;

    // Use ~50ms subblocks; ensure it's an exact multiple of window size
    samples_per_subblock_ = samples_per_window_ * 1250;

    subblocks_remaining_ = kSubblocksPerBlock;
    subblock_samples_remaining_ = samples_per_subblock_;
    subblock_active_ = false;

    double N = std::round(kSubblocksPerBlock * params_.duty_cycle);
    if (N <= 0)
        N = 1;
    if (N > kSubblocksPerBlock)
        N = kSubblocksPerBlock;
    fprintf(stderr, "adaptive: using %.0f%% duty cycle\n", 100.0 * N / kSubblocksPerBlock);
    dutycycle_N_ = (unsigned)N;

    burst_window_remaining_ = samples_per_window_;
    burst_window_counter_ = 0;

    range_radix_.assign(65536, 0);
    range_state_ = RangeState::RESCAN_UP;

    // select and enforce gain limits
    int maxgain = GainStepCount() - 1;
    for (gain_min_ = 0; gain_min_ < maxgain; ++gain_min_) {
        if (StepDb(gain_min_) >= params_.min_gain_db)
            break;
    }

    for (gain_max_ = maxgain; gain_max_ > gain_min_; --gain_max_) {
        if (StepDb(gain_max_) <= params_.max_gain_db)
            break;
    }

    current_step_ = NearestStep(sdr_->GetGainDb());

    fprintf(stderr, "adaptive: enabled adaptive gain control with gain limits %.1fdB (step %d) .. %.1fdB (step %d)\n", StepDb(gain_min_), gain_min_, StepDb(gain_max_), gain_max_);
    if (params_.range_control)
        fprintf(stderr, "adaptive: enabled dynamic range control, target dynamic range %.1fdB\n", params_.range_target);
    if (params_.burst_control)
        fprintf(stderr, "adaptive: enabled burst control\n");
    SetGainStep(current_step_, "constraining gain to adaptive gain limits");
    GainChanged();

    range_gain_limit_ = current_step_;
    report_timer_ = kReportInterval;
}

// Feed some samples into the adaptive system. Any number of samples might be passed in.
void AdaptiveGainControl::Update(const std::uint16_t *buf, unsigned length, const double *decoded_power) {
    if (!params_.burst_control && !params_.range_control)
        return;

    // process complete subblocks
    while (length >= subblock_samples_remaining_) {
        if (subblock_active_)
            UpdateSubblock(buf, subblock_samples_remaining_, decoded_power);

        buf += subblock_samples_remaining_;
        length -= subblock_samples_remaining_;
        subblock_samples_remaining_ = samples_per_subblock_;

        dutycycle_counter_ += dutycycle_N_;
        if (dutycycle_counter_ >= kSubblocksPerBlock) {
            dutycycle_counter_ -= kSubblocksPerBlock;
            subblock_active_ = true;
        } else {
            subblock_active_ = false;
            // fake a quiet window to reset any existing run
            BurstEndOfWindow(0);
        }

        if (!--subblocks_remaining_) {
            // Block completed, do a control update
            subblocks_remaining_ = kSubblocksPerBlock;
            EndOfBlock();
        }
    }

    // process final samples that don't complete a subblock
    if (length > 0) {
        if (subblock_active_)
            UpdateSubblock(buf, length, decoded_power);
        subblock_samples_remaining_ -= length;
    }
}

// Feed some samples into the adaptive system. The samples are guaranteed to not cross a subblock boundary.
// The samples should be processsed (i.e. duty cycle is in the active part)
void AdaptiveGainControl::UpdateSubblock(const std::uint16_t *buf, unsigned length, const double *decoded_power) {
    if (decoded_power) {
        if (*decoded_power >= burst_loud_threshold_)
            ++burst_block_loud_decoded_;
        BurstSkip(length);
    } else {
        BurstUpdate(buf, length);
        RangeUpdate(buf, length);
    }
}

// Burst measurement: ignore the next 'length' samples (they are a successfully decoded message)
void AdaptiveGainControl::BurstSkip(unsigned length) {
    if (!params_.burst_control)
        return;

    // first window
    if (length < burst_window_remaining_) {
        // partial fill
        burst_window_remaining_ -= length;
        return;
    }

    // skip remainder of first window, dispatch it
    BurstEndOfWindow(burst_window_counter_);
    length -= burst_window_remaining_;

    // skip remaining windows, dispatch them
    unsigned windows = length / samples_per_window_;
    unsigned samples = windows * samples_per_window_;
    while (windows--)
        BurstEndOfWindow(0);

    length -= samples;

    // final partial window
    burst_window_counter_ = 0;
    burst_window_remaining_ = samples_per_window_ - length;
}

// Burst measurement: process 'length' samples from 'buf', look for loud bursts;
// the samples might cross burst window boundaries;
// the samples will not cross a block boundary.
void AdaptiveGainControl::BurstUpdate(const std::uint16_t *buf, unsigned length) {
    if (!params_.burst_control)
        return;

    // first window
    if (length < burst_window_remaining_) {
        // partial fill
        burst_window_counter_ += BurstCountSamples(buf, length);
        burst_window_remaining_ -= length;
        return;
    }

    // complete fill of first partial window
    unsigned n = burst_window_remaining_;
    unsigned counter = burst_window_counter_ + BurstCountSamples(buf, n);
    BurstEndOfWindow(counter);
    buf += n;
    length -= n;

    // remaining windows
    unsigned windows = length / samples_per_window_;
    unsigned samples = windows * samples_per_window_;
    BurstScanWindows(buf, windows);
    buf += samples;
    length -= samples;

    // final partial window
    burst_window_counter_ = BurstCountSamples(buf, length);
    burst_window_remaining_ = samples_per_window_ - length;
}

// Burst measurement: process 'windows' complete burst windows starting at 'buf';
// 'buf' is aligned to the start of a burst window
void AdaptiveGainControl::BurstScanWindows(const std::uint16_t *buf, unsigned windows) {
    while (windows--) {
        unsigned counter = BurstCountSamples(buf, samples_per_window_);
        buf += samples_per_window_;
        BurstEndOfWindow(counter);
    }
}

// Burst measurement: process 'n' samples from 'buf', look for loud samples;
// the samples are guaranteed not to cross window boundaries;
// return the number of loud samples seen
// (dump1090 uses a SIMD helper here; a scalar loop is fine at 2.08MS/s with duty cycling)
unsigned AdaptiveGainControl::BurstCountSamples(const std::uint16_t *buf, unsigned n) {
    unsigned counter = 0;
    while (n--) {
        if (*buf++ > 46395 /* -3dBFS */)
            ++counter;
    }
    return counter;
}

// Burst measurement: we reached the end of a burst window with 'counter'
// loud samples seen, handle that window.
void AdaptiveGainControl::BurstEndOfWindow(unsigned counter) {
    if (counter > samples_per_window_ / 4) {
        // This window is loud, extend any existing run of loud windows
        ++burst_runlength_;
    } else {
        // Quiet window. If we saw a run of loud windows >= 80us long, count
        // that as a candidate for an over-amplified message that was
        // not decoded.
        if (burst_runlength_ >= 2 && burst_runlength_ <= 5)
            ++burst_block_loud_undecoded_;
        burst_runlength_ = 0;
    }
}

// Noise measurement: process 'length' samples from 'buf'.
// The samples will not cross a block boundary.
void AdaptiveGainControl::RangeUpdate(const std::uint16_t *buf, unsigned length) {
    if (!params_.range_control)
        return;

    range_radix_counter_ += length;
    while (length--) {
        // do a very simple radix sort of sample magnitudes
        // so we can later find the Nth percentile value
        ++range_radix_[buf[0]];
        ++buf;
    }
}

// Noise measurement: we reached the end of a block, update
// our noise estimate
void AdaptiveGainControl::RangeEndOfBlock() {
    if (!params_.range_control)
        return;

    unsigned n = 0, i = 0;

    // measure Nth percentile magnitude
    unsigned count_n = range_radix_counter_ * params_.range_percentile / 100;
    while (i < 65536 && n <= count_n)
        n += range_radix_[i++];
    std::uint16_t percentile_n = i - 1;

    // maintain an EMA of the Nth percentile
    range_smoothed_ = range_smoothed_ * (1 - params_.range_alpha) + percentile_n * params_.range_alpha;

    // reset radix sort for the next block
    std::fill(range_radix_.begin(), range_radix_.end(), 0);
    range_radix_counter_ = 0;
}

// Burst measurement: we reached the end of a block, update our burst rate estimate
void AdaptiveGainControl::BurstEndOfBlock() {
    if (!params_.burst_control)
        return;

    // scale rates based on the actual duty cycle fraction
    //  (e.g. if we are only inspecting 2/5 of samples, then scale the rate by 5/2)
    double scale = (double)kSubblocksPerBlock / dutycycle_N_;

    // maintain an EMA of the number of undecoded loud bursts seen per block
    report_loud_undecoded_ += burst_block_loud_undecoded_;
    burst_loud_undecoded_smoothed_ = burst_loud_undecoded_smoothed_ * (1 - params_.burst_alpha) + scale * burst_block_loud_undecoded_ * params_.burst_alpha;
    burst_block_loud_undecoded_ = 0;

    // maintain an EMA of the number of decoded, but loud, messages seen per block
    report_loud_decoded_ += burst_block_loud_decoded_;
    burst_loud_decoded_smoothed_ = burst_loud_decoded_smoothed_ * (1 - params_.burst_alpha) + scale * burst_block_loud_decoded_ * params_.burst_alpha;
    burst_block_loud_decoded_ = 0;
}

// Adaptive gain: we reached a block boundary. Update measurements and act on them.
void AdaptiveGainControl::EndOfBlock() {
    RangeEndOfBlock();
    BurstEndOfBlock();

    ControlUpdate();

    if (report_timer_ > 0 && --report_timer_ == 0) {
        PeriodicReport();
        report_timer_ = kReportInterval;
    }
}

// Periodic status line so that convergence is observable in logs
// (replaces dump1090's stats subsystem)
void AdaptiveGainControl::PeriodicReport() {
    fprintf(stderr, "adaptive: gain %.1fdB (step %d)", StepDb(current_step_), current_step_);
    if (params_.range_control) {
        if (range_smoothed_ > 0)
            fprintf(stderr, ", noise floor %.1fdBFS, dynamic range %.1fdB", 20 * std::log10(range_smoothed_ / 65536.0), -20 * std::log10(range_smoothed_ / 65536.0));
        else
            fprintf(stderr, ", noise floor not yet measured");
    }
    if (params_.burst_control)
        fprintf(stderr, ", loud undecoded %u, loud decoded %u", report_loud_undecoded_, report_loud_decoded_);
    fprintf(stderr, ", gain changes %u (last %us)\n", report_gain_changes_, kReportInterval);

    report_gain_changes_ = 0;
    report_loud_undecoded_ = 0;
    report_loud_decoded_ = 0;
}

void AdaptiveGainControl::ControlUpdate() {
    // votes for what to do with the gain
    // "gain_not_up" overlaps somewhat with "gain_down", but they are not identical;
    // burst control may want to prevent gain from increasing, but not necessarily
    // decrease gain.

    bool gain_up = false;
    const char *gain_up_reason = nullptr;
    bool gain_down = false;
    const char *gain_down_reason = nullptr;
    bool gain_not_up = false;

    int current_gain = current_step_;

    if (burst_change_timer_)
        --burst_change_timer_;
    if (range_change_timer_ > 0)
        --range_change_timer_;
    if (range_rescan_timer_ > 0)
        --range_rescan_timer_;

    if (params_.burst_control && !burst_change_timer_) {
        if (burst_loud_undecoded_smoothed_ > params_.burst_loud_rate) {
            burst_quiet_blocks_ = 0;
            ++burst_loud_blocks_;
        } else if (burst_loud_decoded_smoothed_ < params_.burst_quiet_rate) {
            burst_loud_blocks_ = 0;
            ++burst_quiet_blocks_;
        } else {
            burst_loud_blocks_ = 0;
            burst_quiet_blocks_ = 0;
        }

        if (burst_loud_blocks_ >= params_.burst_loud_runlength) {
            // we need to reduce gain (further)
            gain_down = gain_not_up = true;
            gain_down_reason = "high rate of loud undecoded messages";

            // if we're currently doing a downward scan, reducing gain further may confuse it;
            // stop that scan and restart it once we are no longer in a reduced-gain state
            if (range_state_ == RangeState::SCAN_DOWN || range_state_ == RangeState::RESCAN_DOWN) {
                range_state_ = RangeState::SCAN_IDLE;
                range_rescan_timer_ = 0;
            }
        } else if (burst_quiet_blocks_ < params_.burst_quiet_runlength) {
            // we're OK at the current gain, but should not increase it
            gain_not_up = true;
        } else if (current_gain < range_gain_limit_) {
            // we're OK at the current gain, and can increase gain to the previously discovered
            // dynamic range limit
            gain_up = true;
            gain_up_reason = "low loud message rate and gain below dynamic range limit";
        }
    }

    if (params_.range_control && !range_change_timer_) {
        double available_range = -20 * std::log10(range_smoothed_ / 65536.0);
        // allow the gain limit to increase if this gain setting is acceptable
        // (decreasing the limit is done separately depending on the current state as we make slightly different decisions in IDLE
        // to provide hysteresis)
        if (available_range >= params_.range_target && current_gain > range_gain_limit_) {
            range_gain_limit_ = current_gain;
        }
        switch (range_state_) {
        case RangeState::SCAN_UP:
        case RangeState::RESCAN_UP:
            if (available_range < params_.range_target) {
                // Current gain fails to meet our target. Switch to downward scanning.
                fprintf(stderr, "adaptive: available dynamic range (%.1fdB) < required dynamic range (%.1fdB), switching to downward scan\n", available_range, params_.range_target);
                gain_down = gain_not_up = true;
                gain_down_reason = "probing dynamic range gain lower bound";
                range_state_ = (range_state_ == RangeState::RESCAN_UP ? RangeState::RESCAN_DOWN : RangeState::SCAN_DOWN);
                if (range_gain_limit_ >= current_gain) {
                    range_gain_limit_ = current_gain - 1;
                }
                break;
            }

            if (current_step_ >= gain_max_) {
                // We have reached our upper gain limit
                fprintf(stderr, "adaptive: reached upper gain limit, halting dynamic range scan here\n");
                range_state_ = RangeState::SCAN_IDLE;
                range_rescan_timer_ = params_.range_rescan_delay;
                break;
            }

            // This gain step is OK and we have more to try, try the next gain step up.
            // (But if burst detection has inhibited increasing gain, don't do anything yet, just try again next block)
            if (!gain_not_up) {
                fprintf(stderr, "adaptive: available dynamic range (%.1fdB) >= required dynamic range (%.1fdB), continuing upward scan\n", available_range, params_.range_target);
                gain_up = true;
                gain_up_reason = "probing dynamic range gain upper bound";
            }
            break;

        case RangeState::SCAN_DOWN:
        case RangeState::RESCAN_DOWN:
            if (available_range >= params_.range_target) {
                // Current gain meets our target; we are done with the scan.
                fprintf(stderr, "adaptive: available dynamic range (%.1fdB) >= required dynamic range (%.1fdB), stopping downwards scan here\n", available_range, params_.range_target);
                range_state_ = RangeState::SCAN_IDLE;
                // Note: quirk ported as-is from dump1090's adaptive.c for behavior parity:
                // the state is updated before this comparison, so the rescan delay is
                // always selected here regardless of whether this was a SCAN or RESCAN.
                range_rescan_timer_ = (range_state_ == RangeState::SCAN_DOWN ? params_.range_scan_delay : params_.range_rescan_delay);
                break;
            }

            if (range_gain_limit_ >= current_gain) {
                range_gain_limit_ = current_gain - 1;
            }

            if (current_step_ <= gain_min_) {
                fprintf(stderr, "adaptive: reached lower gain limit, halting dynamic range scan here\n");
                range_state_ = RangeState::SCAN_IDLE;
                range_rescan_timer_ = params_.range_rescan_delay;
                break;
            }

            // This gain step is too loud and we have more to try, try the next gain step down
            fprintf(stderr, "adaptive: available dynamic range (%.1fdB) < required dynamic range (%.1fdB), continuing downwards scan\n", available_range, params_.range_target);
            gain_down = gain_not_up = true;
            gain_down_reason = "probing dynamic range gain lower bound";
            break;

        case RangeState::SCAN_IDLE:
            // Look for increased noise that could be compensated for by decreasing gain.
            // Do this even if we're waiting to rescan or if burst control is also active
            if (available_range + gain_down_db_ / 2 < params_.range_target && current_step_ > gain_min_) {
                fprintf(stderr, "adaptive: available dynamic range (%.1fdB) + half gain step down (%.1fdB) < required dynamic range (%.1fdB), starting downward scan\n", available_range, gain_down_db_ / 2, params_.range_target);
                if (range_gain_limit_ >= current_gain) {
                    range_gain_limit_ = current_gain - 1;
                }
                range_state_ = RangeState::SCAN_DOWN;
                gain_down = gain_not_up = true;
                gain_down_reason = "dynamic range fell below target value";
                break;
            }

            // Infrequently consider increasing gain to handle the case where we've selected a too-low gain where the noise floor is dominated by noise unrelated to the gain setting.
            // But don't do this while burst control is preventing gain increases.
            if (!range_rescan_timer_ && !gain_not_up) {
                if (available_range >= params_.range_target && current_step_ < gain_max_) {
                    fprintf(stderr, "adaptive: start periodic scan for acceptable dynamic range at increased gain\n");
                    gain_up = true;
                    gain_up_reason = "periodic re-probing of dynamic range gain upper bound";
                    range_state_ = RangeState::RESCAN_UP;
                    break;
                }

                // Nothing to do for a while.
                range_rescan_timer_ = params_.range_rescan_delay;
            }

            break;

        default:
            fprintf(stderr, "adaptive: in a weird state (%u), trying to fix it\n", (unsigned)range_state_);
            range_state_ = RangeState::SCAN_IDLE;
            range_rescan_timer_ = params_.range_rescan_delay;
            break;
        }
    }

    // now actually perform any gain changes

    if (gain_down)
        DecreaseGain(gain_down_reason);
    else if (gain_up && !gain_not_up)
        IncreaseGain(gain_up_reason);
}
