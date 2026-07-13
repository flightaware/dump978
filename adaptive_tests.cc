// adaptive_tests.cc: tests for adaptive gain control
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

#include <cmath>
#include <cstdio>
#include <vector>

using namespace flightaware::uat;

static int failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::fprintf(stderr, "FAIL at line %d: %s\n", __LINE__, #cond); \
            ++failures;                                                \
        }                                                              \
    } while (0)

// A fake SDR that behaves like an RTLSDR seen through SoapySDR:
// gain range 0 .. 49.6 dB, no step size reported.
class FakeGainControl : public SdrGainControl {
  public:
    FakeGainControl() : gain_(49.6) {}

    double GetGainDb() override { return gain_; }

    double SetGainDb(double db) override {
        gain_ = db;
        history.push_back(db);
        return db;
    }

    void GetGainLimits(double &min, double &max, double &step) override {
        min = 0;
        max = 49.6;
        step = 0;
    }

    std::vector<double> history;

  private:
    double gain_;
};

// Use a low sample rate so the tests can feed many ~1 second blocks quickly.
// The adaptive block structure scales with the sample rate (window = rate/25000).
static const unsigned kTestRate = 250000;
static const unsigned kSamplesPerWindow = kTestRate / 25000;
static const unsigned kSamplesPerSubblock = kSamplesPerWindow * 1250;
static const unsigned kSubblocksPerBlock = 20;

// Feed `blocks` whole blocks built from copies of `subblock`
static void FeedBlocks(AdaptiveGainControl::Pointer adaptive, const std::vector<std::uint16_t> &subblock, unsigned blocks, const double *decoded_power = nullptr) {
    for (unsigned b = 0; b < blocks; ++b) {
        for (unsigned s = 0; s < kSubblocksPerBlock; ++s) {
            adaptive->Update(subblock.data(), subblock.size(), decoded_power);
        }
    }
}

static std::vector<std::uint16_t> ConstantSubblock(std::uint16_t magnitude) { return std::vector<std::uint16_t>(kSamplesPerSubblock, magnitude); }

// A subblock containing repeated 3-window-long full-scale bursts separated by silence,
// i.e. what an overloaded receiver failing to decode messages looks like
static std::vector<std::uint16_t> BurstySubblock() {
    std::vector<std::uint16_t> subblock(kSamplesPerSubblock, 0);
    const unsigned period = kSamplesPerWindow * 40;
    for (unsigned i = 0; i < subblock.size(); ++i) {
        if (i % period < kSamplesPerWindow * 3)
            subblock[i] = 65535;
    }
    return subblock;
}

// uint16 magnitude for a given dBFS level
static std::uint16_t MagnitudeAtDbfs(double dbfs) { return (std::uint16_t)std::round(std::pow(10.0, dbfs / 20.0) * 65536.0); }

// The gain limits from --adaptive-min-gain/--adaptive-max-gain must be honored,
// including the initial constraining of the starting gain.
static void TestGainLimits() {
    std::fprintf(stderr, "--- TestGainLimits\n");
    auto fake = std::make_shared<FakeGainControl>();
    AdaptiveParams params;
    params.range_control = true;
    params.min_gain_db = 20;
    params.max_gain_db = 40;
    auto adaptive = AdaptiveGainControl::Create(params, fake, kTestRate);

    CHECK(adaptive->Enabled());
    // the fake device starts at 49.6dB; init must pull it down to <= 40dB
    CHECK(!fake->history.empty());
    CHECK(fake->GetGainDb() <= 40.0);
    CHECK(fake->GetGainDb() >= 20.0);
}

// With plenty of dynamic range available, the range scan should hold the gain
// at the upper limit and never step down.
static void TestRangeQuiet() {
    std::fprintf(stderr, "--- TestRangeQuiet\n");
    auto fake = std::make_shared<FakeGainControl>();
    AdaptiveParams params;
    params.range_control = true;
    params.duty_cycle = 1.0;
    auto adaptive = AdaptiveGainControl::Create(params, fake, kTestRate);

    // constant -40dBFS noise floor; default target is 0.6 * 49.6 = 29.8dB
    FeedBlocks(adaptive, ConstantSubblock(MagnitudeAtDbfs(-40.0)), 30);

    CHECK(fake->GetGainDb() > 49.0);
}

// With too little dynamic range, the range scan should walk the gain down
// until it hits the configured lower limit.
static void TestRangeScanDown() {
    std::fprintf(stderr, "--- TestRangeScanDown\n");
    auto fake = std::make_shared<FakeGainControl>();
    AdaptiveParams params;
    params.range_control = true;
    params.duty_cycle = 1.0;
    params.min_gain_db = 40;
    auto adaptive = AdaptiveGainControl::Create(params, fake, kTestRate);

    // constant -20dBFS noise floor: only 20dB of dynamic range available,
    // below the ~29.8dB target no matter the gain, so the scan runs to the lower limit
    FeedBlocks(adaptive, ConstantSubblock(MagnitudeAtDbfs(-20.0)), 150);

    CHECK(!fake->history.empty());
    CHECK(std::fabs(fake->GetGainDb() - 40.0) < 0.5);
    // the scan must step down one step at a time
    for (std::size_t i = 1; i < fake->history.size(); ++i) {
        CHECK(fake->history[i] < fake->history[i - 1]);
        CHECK(fake->history[i - 1] - fake->history[i] < 1.5);
    }
}

// Loud, message-length, undecoded bursts should push the gain down; after the
// bursts stop, the gain should recover towards the previously known-good limit.
static void TestBurstBackoffAndRecovery() {
    std::fprintf(stderr, "--- TestBurstBackoffAndRecovery\n");
    auto fake = std::make_shared<FakeGainControl>();
    AdaptiveParams params;
    params.burst_control = true;
    auto adaptive = AdaptiveGainControl::Create(params, fake, kTestRate);

    FeedBlocks(adaptive, BurstySubblock(), 40);

    CHECK(!fake->history.empty());
    double backed_off = fake->GetGainDb();
    CHECK(backed_off < 49.0);

    // quiet input: burst control should raise the gain back towards the limit
    FeedBlocks(adaptive, ConstantSubblock(0), 40);
    CHECK(fake->GetGainDb() > backed_off);
}

// The same bursts fed as successfully decoded messages must not trigger a backoff.
static void TestBurstDecodedSkip() {
    std::fprintf(stderr, "--- TestBurstDecodedSkip\n");
    auto fake = std::make_shared<FakeGainControl>();
    AdaptiveParams params;
    params.burst_control = true;
    auto adaptive = AdaptiveGainControl::Create(params, fake, kTestRate);

    // decoded messages with a quiet mean power (below the loud-decode threshold)
    double decoded_power = 1e-4;
    FeedBlocks(adaptive, BurstySubblock(), 40, &decoded_power);

    CHECK(fake->history.empty());
    CHECK(fake->GetGainDb() > 49.0);
}

// An uncontrollable SDR (no gain range) must disable adaptive control cleanly.
class FixedGainControl : public SdrGainControl {
  public:
    double GetGainDb() override { return 0; }
    double SetGainDb(double db) override { return 0; }
    void GetGainLimits(double &min, double &max, double &step) override { min = max = step = 0; }
};

static void TestUncontrollableSdr() {
    std::fprintf(stderr, "--- TestUncontrollableSdr\n");
    AdaptiveParams params;
    params.range_control = true;
    params.burst_control = true;
    auto adaptive = AdaptiveGainControl::Create(params, std::make_shared<FixedGainControl>(), kTestRate);
    CHECK(!adaptive->Enabled());
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;

    TestGainLimits();
    TestRangeQuiet();
    TestRangeScanDown();
    TestBurstBackoffAndRecovery();
    TestBurstDecodedSkip();
    TestUncontrollableSdr();

    if (failures) {
        std::fprintf(stderr, "%d test(s) FAILED\n", failures);
        return 1;
    }

    std::fprintf(stderr, "all tests passed\n");
    return 0;
}
