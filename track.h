// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef FAUP978_TRACK_H
#define FAUP978_TRACK_H

#include <chrono>
#include <memory>
#include <numeric>
#include <array>

#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <boost/asio/strand.hpp>

#include "uat_message.h"

namespace flightaware {
  namespace uat {
    class AgedFieldBase {
      public:
        operator bool() const { return Valid(); }

        bool Valid() const { return (updated_ != 0); }

        std::uint64_t Changed() const { return changed_; }

        std::uint64_t Updated() const { return updated_; }

        std::uint64_t ChangeAge(std::uint64_t at) const {
            if (at < changed_) {
                return 0;
            } else {
                return at - changed_;
            }
        }

        std::uint64_t UpdateAge(std::uint64_t at) const {
            if (at < updated_) {
                return 0;
            } else {
                return at - updated_;
            }
        }

      protected:
        std::uint64_t updated_ = 0;
        std::uint64_t changed_ = 0;
    };

    template <class T> class AgedField : public AgedFieldBase {
      public:
        AgedField() : v_() {}
        AgedField(const T &v) : v_(v) {}

        bool MaybeUpdate(std::uint64_t at, const T &v) {
            if (at > updated_) {
                updated_ = at;
                if (v != v_) {
                    changed_ = at;
                }
                v_ = v;
                return true;
            } else {
                return false;
            }
        }

        const T &Value() const { return v_; }

      private:
        T v_;
    };

    struct AircraftState {
        AircraftState(AddressQualifier aq = AddressQualifier::INVALID, AdsbAddress ad = 0) : address_qualifier(aq), address(ad) { rssi.fill(0.0); }

        AddressQualifier address_qualifier;
        AdsbAddress address;

        std::uint64_t last_message_time = 0;
        std::uint32_t messages = 0;
        std::array<double, 16> rssi;

        AgedField<std::pair<double, double>> position; // latitude, longitude
        AgedField<int> pressure_altitude;
        AgedField<int> geometric_altitude;
        AgedField<unsigned> nic;
        AgedField<AirGroundState> airground_state;
        AgedField<int> north_velocity;
        AgedField<int> east_velocity;
        AgedField<int> vertical_velocity_barometric;
        AgedField<int> vertical_velocity_geometric;
        AgedField<int> ground_speed;
        AgedField<double> magnetic_heading;
        AgedField<double> true_heading;
        AgedField<double> true_track;
        AgedField<std::pair<double, double>> aircraft_size; // length, width
        AgedField<double> gps_lateral_offset;
        AgedField<double> gps_longitudinal_offset;
        AgedField<bool> gps_position_offset_applied;
        AgedField<bool> utc_coupled;

        AgedField<unsigned> emitter_category;
        AgedField<std::string> callsign;
        AgedField<std::string> flightplan_id; // aka Mode 3/A squawk
        AgedField<EmergencyPriorityStatus> emergency;
        AgedField<unsigned> mops_version;
        AgedField<unsigned> sil;
        AgedField<unsigned> transmit_mso;
        AgedField<unsigned> sda;
        AgedField<unsigned> nac_p;
        AgedField<unsigned> nac_v;
        AgedField<unsigned> nic_baro;
        AgedField<CapabilityCodes> capability_codes;
        AgedField<OperationalModes> operational_modes;
        AgedField<SILSupplement> sil_supplement;
        AgedField<unsigned> gva;
        AgedField<bool> single_antenna;
        AgedField<bool> nic_supplement;

        // derived from nic, nic_supplement
        AgedField<double> horizontal_containment; // upper bound, meters

        AgedField<int> selected_altitude_mcp;
        AgedField<int> selected_altitude_fms;
        AgedField<double> barometric_pressure_setting;
        AgedField<double> selected_heading;
        AgedField<ModeIndicators> mode_indicators;

        double AverageRssi() const {
            if (!messages)
                return 0.0;

            return std::accumulate(rssi.begin(), rssi.end(), 0.0) / std::min<double>(messages, rssi.size());
        }

        void UpdateFromMessage(const AdsbMessage &message);
    };

    class Tracker : public std::enable_shared_from_this<Tracker> {
      public:
        typedef std::pair<AddressQualifier, AdsbAddress> AddressKey;
        typedef std::shared_ptr<Tracker> Pointer;
        typedef std::map<AddressKey, AircraftState> MapType;

        static Pointer Create(boost::asio::io_context &service, std::chrono::milliseconds timeout = std::chrono::seconds(300)) { return Pointer(new Tracker(service, timeout)); }

        void Start();
        void Stop();
        void HandleMessages(SharedMessageVector messages);

        const MapType &Aircraft() const { return aircraft_; }
        std::uint32_t TotalMessages() const { return total_messages_; }

        void PurgeOld();

      private:
        Tracker(boost::asio::io_context &service, std::chrono::milliseconds timeout) : strand_(service), timer_(service), timeout_(timeout) {}

        void HandleMessage(const AdsbMessage &message);

        boost::asio::io_context::strand strand_;
        boost::asio::steady_timer timer_;
        std::chrono::milliseconds timeout_;
        MapType aircraft_;
        std::uint32_t total_messages_ = 0;
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
