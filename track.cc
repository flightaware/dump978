// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "track.h"

#include <iomanip>
#include <iostream>

#include <boost/asio/bind_executor.hpp>

using namespace flightaware::uat;

void AircraftState::UpdateFromMessage(const AdsbMessage &message) {
    if (message.received_at < last_message_time) {
        // Out of order message
        return;
    }

#define UPDATE(x)                                           \
    do {                                                    \
        if (message.x) {                                    \
            x.MaybeUpdate(message.received_at, *message.x); \
        }                                                   \
    } while (0)

    UPDATE(position); // latitude, longitude
    UPDATE(pressure_altitude);
    UPDATE(geometric_altitude);
    UPDATE(nic);
    UPDATE(airground_state);
    UPDATE(north_velocity);
    UPDATE(east_velocity);
    UPDATE(vertical_velocity_barometric);
    UPDATE(vertical_velocity_geometric);
    UPDATE(ground_speed);
    UPDATE(magnetic_heading);
    UPDATE(true_heading);
    UPDATE(true_track);
    UPDATE(aircraft_size); // length, width
    UPDATE(gps_lateral_offset);
    UPDATE(gps_longitudinal_offset);
    UPDATE(gps_position_offset_applied);
    UPDATE(utc_coupled);

    UPDATE(emitter_category);
    UPDATE(callsign);
    UPDATE(flightplan_id); // aka Mode 3/A squawk
    UPDATE(emergency);
    UPDATE(mops_version);
    UPDATE(sil);
    UPDATE(transmit_mso);
    UPDATE(sda);
    UPDATE(nac_p);
    UPDATE(nac_v);
    UPDATE(nic_baro);
    UPDATE(capability_codes);
    UPDATE(operational_modes);
    UPDATE(sil_supplement);
    UPDATE(gva);
    UPDATE(single_antenna);
    UPDATE(nic_supplement);

    UPDATE(selected_altitude_mcp);
    UPDATE(selected_altitude_fms);
    UPDATE(barometric_pressure_setting);
    UPDATE(selected_heading);
    UPDATE(mode_indicators);

    // derive horizontal containment radius
    if (message.nic) {
        static std::map<unsigned, double> rc_lookup = {
            /* 0 - unknown */
            {1, 37040},
            {2, 14816},
            {3, 7408},
            {4, 3704},
            {5, 1852},
            /* 6 - special case */
            {7, 370.4},
            {8, 185.2},
            {9, 75},
            {10, 25},
            {11, 7.5}
            /* 12..15 - reserved */
        };

        double rc = 0;
        if (*message.nic == 6) {
            if (nic_supplement.Valid() && nic_supplement.Value()) {
                rc = 555.6;
            } else {
                rc = 1111.2;
            }
        } else {
            auto i = rc_lookup.find(*message.nic);
            if (i != rc_lookup.end())
                rc = i->second;
        }

        horizontal_containment.MaybeUpdate(message.received_at, rc);
    }

    rssi[messages % rssi.size()] = message.rssi;
    last_message_time = message.received_at;
    ++messages;

#undef UPDATE
}

void Tracker::Start() { PurgeOld(); }

void Tracker::Stop() { timer_.cancel(); }

void Tracker::PurgeOld() {
    std::uint64_t expires_timestamp = now_millis() - timeout_.count();

    for (auto i = aircraft_.begin(); i != aircraft_.end();) {
        if (i->second.last_message_time < expires_timestamp) {
            i = aircraft_.erase(i);
        } else {
            ++i;
        }
    }
    auto self(shared_from_this());
    timer_.expires_after(timeout_ / 4);
    timer_.async_wait(boost::asio::bind_executor(strand_, [this, self](const boost::system::error_code &ec) {
        if (!ec) {
            PurgeOld();
        }
    }));
}

void Tracker::HandleMessages(SharedMessageVector messages) {
    static auto unix_epoch = std::chrono::system_clock::from_time_t(0);
    const std::uint64_t now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - unix_epoch).count();

    auto self(shared_from_this());
    boost::asio::dispatch(strand_, [this, self, now, messages]() {
        const std::uint64_t PAST_FUZZ = 15000;
        const std::uint64_t FUTURE_FUZZ = 1000;

        for (const auto &message : *messages) {
            // Handle only downlink messages
            if (message.Type() != MessageType::DOWNLINK_SHORT && message.Type() != MessageType::DOWNLINK_LONG) {
                continue;
            }

            // validate message time vs system clock so we are only processing
            // contemporaneous messages
            if (message.ReceivedAt() == 0 || message.ReceivedAt() < (now - PAST_FUZZ) || message.ReceivedAt() > (now + FUTURE_FUZZ)) {
                std::cerr << "DISCARD " << message.ReceivedAt() << std::endl;
                continue;
            }

            HandleMessage(AdsbMessage(message));
        }
    });
}

void Tracker::HandleMessage(const AdsbMessage &message) {
    AddressKey key{message.address_qualifier, message.address};
    auto i = aircraft_.find(key);
    if (i == aircraft_.end()) {
        aircraft_[key] = {message.address_qualifier, message.address};
    }

    aircraft_[key].UpdateFromMessage(message);
    ++total_messages_;
}
