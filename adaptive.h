// -*- c++ -*-

// adaptive.h: adaptive gain control
//
// Ported from dump1090's adaptive.c / adaptive.h
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

#ifndef DUMP978_ADAPTIVE_H
#define DUMP978_ADAPTIVE_H

#include <cstdint>
#include <memory>
#include <vector>

namespace flightaware {
  namespace uat {
    // Tunables for adaptive gain control; defaults match dump1090.
    struct AdaptiveParams {
        bool burst_control = false;         // enable burst / overload control?
        bool range_control = false;         // enable dynamic range control?
        double min_gain_db = 0;             // never step gain below this
        double max_gain_db = 99999;         // never step gain above this
        double duty_cycle = 0.5;            // fraction of samples to inspect
        double burst_alpha = 2.0 / (5 + 1); // EMA factor for burst rates
        unsigned burst_change_delay = 5;    // blocks to wait after a gain change before acting on burst measurements
        double burst_loud_rate = 5.0;       // undecoded loud burst rate that provokes a gain decrease
        unsigned burst_loud_runlength = 10; // consecutive loud blocks needed before decreasing gain
        double burst_quiet_rate = 5.0;      // loud decoded message rate that must not be exceeded before increasing gain
        unsigned burst_quiet_runlength = 10; // consecutive quiet blocks needed before increasing gain
        double range_alpha = 2.0 / (5 + 1);  // EMA factor for the noise floor estimate
        unsigned range_percentile = 40;      // percentile of sample magnitude to use as the noise floor
        double range_target = 0;             // target dynamic range in dB; <= 0 means autoselect
        unsigned range_change_delay = 10;    // blocks to wait after a gain change before acting on noise measurements
        unsigned range_scan_delay = 300;     // blocks to wait before rescanning after an abortive scan
        unsigned range_rescan_delay = 3600;  // blocks to wait between periodic rescans for increased gain
    };

    // Abstract interface for changing SDR gain; implemented by SoapySampleSource.
    // Kept free of SoapySDR types so that adaptive gain control is unit-testable.
    class SdrGainControl {
      public:
        typedef std::shared_ptr<SdrGainControl> Pointer;

        virtual ~SdrGainControl() {}

        // Return the device's current gain in dB
        virtual double GetGainDb() = 0;

        // Try to set the device gain to `db` dB; return the gain the device reports afterwards
        virtual double SetGainDb(double db) = 0;

        // Return the device's overall gain range; `step` may be zero if the device
        // does not report a step size
        virtual void GetGainLimits(double &min, double &max, double &step) = 0;
    };

    // Software adaptive gain control, a port of dump1090's adaptive.c.
    //
    // Feed magnitude samples via Update(); the controller measures the noise
    // floor and loud undecoded bursts, and steps the SDR gain up or down at
    // ~1 second intervals to hold a target dynamic range and back off from
    // overload. Not thread-safe: all calls must come from the same thread
    // (the SDR rx/demodulator thread).
    class AdaptiveGainControl {
      public:
        typedef std::shared_ptr<AdaptiveGainControl> Pointer;

        static Pointer Create(const AdaptiveParams &params, SdrGainControl::Pointer sdr, unsigned sample_rate) { return Pointer(new AdaptiveGainControl(params, sdr, sample_rate)); }

        // false if adaptive control was requested but the SDR's gain is not controllable
        bool Enabled() const { return params_.burst_control || params_.range_control; }

        // Feed samples into the adaptive system. Any number of samples might be passed in.
        // `buf` holds one uint16 magnitude per sample (65536 = full scale).
        // `decoded_power` is nullptr for samples that were not part of a successfully
        // decoded message; for decoded message samples, it points to the mean power of
        // the message (linear, 1.0 = full scale, i.e. mean magnitude-squared).
        void Update(const std::uint16_t *buf, unsigned length, const double *decoded_power);

      private:
        AdaptiveGainControl(const AdaptiveParams &params, SdrGainControl::Pointer sdr, unsigned sample_rate);

        int GainStepCount() const { return (int)gain_steps_.size(); }
        double StepDb(int step) const;
        int NearestStep(double db) const;
        bool SetGainStep(int step, const char *why);
        void GainChanged();
        void IncreaseGain(const char *why);
        void DecreaseGain(const char *why);

        void UpdateSubblock(const std::uint16_t *buf, unsigned length, const double *decoded_power);
        void EndOfBlock();
        void ControlUpdate();
        void PeriodicReport();

        void BurstUpdate(const std::uint16_t *buf, unsigned length);
        void BurstSkip(unsigned length);
        unsigned BurstCountSamples(const std::uint16_t *buf, unsigned n);
        void BurstScanWindows(const std::uint16_t *buf, unsigned windows);
        void BurstEndOfWindow(unsigned counter);
        void BurstEndOfBlock();

        void RangeUpdate(const std::uint16_t *buf, unsigned length);
        void RangeEndOfBlock();

        AdaptiveParams params_;
        SdrGainControl::Pointer sdr_;

        // gain step table and limits
        std::vector<double> gain_steps_; // dB value for each gain step
        int current_step_ = 0;           // gain step we last set (logical state; the hardware may quantize)
        int gain_min_ = 0;               // lowest gain step we will use
        int gain_max_ = 0;               // highest gain step we will use

        // gain steps relative to current gain
        double gain_up_db_ = 0;
        double gain_down_db_ = 0;

        // block handling
        unsigned samples_per_window_ = 0;   // samples per burst window (~40us)
        unsigned samples_per_subblock_ = 0; // samples per subblock (~50ms)
        unsigned subblocks_remaining_ = 0;  // subblocks remaining in the current block

        unsigned dutycycle_N_ = 0;        // subblock duty cycle numerator N (out of D = kSubblocksPerBlock)
        unsigned dutycycle_counter_ = 0;  // subblock duty cycle counter (modulo D)
        bool subblock_active_ = false;    // is the current subblock active i.e. samples should be processed, not skipped?
        unsigned subblock_samples_remaining_ = 0; // samples remaining in the current subblock

        // burst handling
        unsigned burst_window_remaining_ = 0;     // samples remaining in the current burst window
        unsigned burst_window_counter_ = 0;       // loud samples seen in current burst window
        unsigned burst_runlength_ = 0;            // consecutive loud burst windows seen
        unsigned burst_block_loud_undecoded_ = 0; // loud undecoded bursts seen in this block so far
        unsigned burst_block_loud_decoded_ = 0;   // loud decoded messages seen in this block so far
        double burst_loud_undecoded_smoothed_ = 0; // smoothed rate of loud misdecodes per block
        double burst_loud_decoded_smoothed_ = 0;   // smoothed rate of loud successful decodes per block
        unsigned burst_change_timer_ = 0;          // countdown inhibiting control after changing gain
        double burst_loud_threshold_ = 0;          // current signal level threshold for a "loud decode"
        unsigned burst_loud_blocks_ = 0;           // consecutive blocks with loud rate
        unsigned burst_quiet_blocks_ = 0;          // consecutive blocks with quiet rate

        // noise floor measurement (adaptive dynamic range)
        std::vector<unsigned> range_radix_;    // radix-sort buckets for current block
        unsigned range_radix_counter_ = 0;     // sum of all radix-sort buckets (= number of samples sorted)
        double range_smoothed_ = 0;            // smoothed noise floor estimate (u16 magnitude units)
        enum class RangeState { SCAN_IDLE, SCAN_UP, SCAN_DOWN, RESCAN_UP, RESCAN_DOWN };
        RangeState range_state_ = RangeState::SCAN_UP;
        unsigned range_change_timer_ = 0; // countdown inhibiting control after changing gain
        unsigned range_rescan_timer_ = 0; // countdown to next upwards gain reprobe
        int range_gain_limit_ = 0;        // probed maximum gain step with acceptable dynamic range

        // periodic reporting (replaces dump1090's stats subsystem)
        unsigned report_timer_ = 0;         // blocks until the next periodic report
        unsigned report_gain_changes_ = 0;  // gain changes since the last report
        unsigned report_loud_undecoded_ = 0; // loud undecoded bursts since the last report
        unsigned report_loud_decoded_ = 0;   // loud decoded messages since the last report
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
