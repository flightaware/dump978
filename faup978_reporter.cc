// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "faup978_reporter.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <boost/asio/bind_executor.hpp>

using namespace flightaware::uat;
using namespace flightaware::faup978;

void Reporter::Start() {
    PeriodicReport();
    PurgeOld();
}

void Reporter::Stop() {
    report_timer_.cancel();
    purge_timer_.cancel();
}

void Reporter::HandleMessages(flightaware::uat::SharedMessageVector messages) {
    // Process metadata messages, note fecfix and report versions
    for (const auto &i : *messages) {
        if (i.Type() != MessageType::METADATA)
            continue;

        auto &metadata = i.Metadata();
        if (metadata.count("fecfix"))
            fecfix_ = true;

        if (metadata.count("program") || metadata.count("version")) {
            const std::uint64_t now = now_millis();
            std::cout << "_v" << '\t' << TSVVersion() << '\t' << "clock" << '\t' << (now / 1000);
            if (metadata.count("program"))
                std::cout << '\t' << "uat_program" << '\t' << metadata.at("program");
            if (metadata.count("version"))
                std::cout << '\t' << "uat_program_version" << '\t' << metadata.at("version");
            std::cout << std::endl;
        }
    }

    tracker_->HandleMessages(messages);
}

void Reporter::PurgeOld() {
    tracker_->PurgeOld();

    auto &aircraft = tracker_->Aircraft();
    for (auto i = reported_.begin(); i != reported_.end();) {
        if (aircraft.count(i->first) == 0) {
            i = reported_.erase(i);
        } else {
            ++i;
        }
    }

    auto self(shared_from_this());
    purge_timer_.expires_after(timeout_ / 4);
    purge_timer_.async_wait(boost::asio::bind_executor(strand_, [this, self](const boost::system::error_code &ec) {
        if (!ec) {
            PurgeOld();
        }
    }));
}

template <typename T> static std::string value_map(T value, const std::map<T, std::string> &mappings, const std::string &default_value) {
    auto i = mappings.find(value);
    if (i == mappings.end()) {
        return default_value;
    } else {
        return i->second;
    }
}

template <typename T> static std::function<void(std::ostream &)> simple_emit(const T &field, int precision = 0) {
    return [&field, precision](std::ostream &os) { os << std::fixed << std::setprecision(precision) << field.Value(); };
}

void Reporter::PeriodicReport() {
    const std::uint64_t now = now_millis();

    for (const auto &entry : tracker_->Aircraft()) {
        ReportOneAircraft(entry.first, entry.second, now);
    }

    auto self(shared_from_this());
    report_timer_.expires_after(interval_);
    report_timer_.async_wait(boost::asio::bind_executor(strand_, [this, self](const boost::system::error_code &ec) {
        if (!ec) {
            PeriodicReport();
        }
    }));
}

void Reporter::ReportOneAircraft(const Tracker::AddressKey &key, const AircraftState &aircraft, std::uint64_t now) {
    // don't report TIS-B at all
    if (aircraft.address_qualifier == AddressQualifier::TISB_ICAO || aircraft.address_qualifier == AddressQualifier::TISB_TRACKFILE) {
        return;
    }

    auto &last = reported_[key];
    auto &last_state = last.report_state;

    if (aircraft.messages < 2) {
        // possibly noise
    }

    if (aircraft.last_message_time <= last.report_time) {
        // no data received since last report
        return;
    }

    bool changed = false;
    changed |= (last_state.pressure_altitude && aircraft.pressure_altitude && std::abs(last_state.pressure_altitude.Value() - aircraft.pressure_altitude.Value()) >= 50);
    changed |= (last_state.geometric_altitude && aircraft.geometric_altitude && std::abs(last_state.pressure_altitude.Value() - aircraft.pressure_altitude.Value()) >= 50);
    changed |= (last_state.vertical_velocity_barometric && aircraft.vertical_velocity_barometric && std::abs(last_state.vertical_velocity_barometric.Value() - aircraft.vertical_velocity_barometric.Value()) >= 500);
    changed |= (last_state.vertical_velocity_geometric && aircraft.vertical_velocity_geometric && std::abs(last_state.vertical_velocity_geometric.Value() - aircraft.vertical_velocity_geometric.Value()) >= 500);
    changed |= (last_state.true_track && aircraft.true_track && std::abs(last_state.true_track.Value() - aircraft.true_track.Value()) >= 2);
    changed |= (last_state.true_heading && aircraft.true_heading && std::abs(last_state.true_heading.Value() - aircraft.true_heading.Value()) >= 2);
    changed |= (last_state.magnetic_heading && aircraft.magnetic_heading && std::abs(last_state.magnetic_heading.Value() - aircraft.magnetic_heading.Value()) >= 2);
    changed |= (last_state.ground_speed && aircraft.ground_speed && std::abs(last_state.ground_speed.Value() - aircraft.ground_speed.Value()) >= 25);

    bool immediate = false;
    immediate |= (aircraft.selected_altitude_mcp.Changed() > last.report_time);
    immediate |= (aircraft.selected_altitude_fms.Changed() > last.report_time);
    immediate |= (aircraft.selected_heading.Changed() > last.report_time);
    immediate |= (aircraft.mode_indicators.Changed() > last.report_time);
    immediate |= (aircraft.barometric_pressure_setting.Changed() > last.report_time);
    immediate |= (aircraft.callsign.Changed() > last.report_time);
    immediate |= (aircraft.flightplan_id.Changed() > last.report_time);
    immediate |= (aircraft.airground_state.Changed() > last.report_time);
    immediate |= (aircraft.emergency.Changed() > last.report_time);

    boost::optional<int> altitude = -10000; // keep the compiler happier
    if (aircraft.pressure_altitude.UpdateAge(now) < 30000)
        altitude = aircraft.pressure_altitude.Value();
    else if (aircraft.geometric_altitude.UpdateAge(now) < 30000)
        altitude = aircraft.geometric_altitude.Value();
    else
        altitude = boost::none;

    boost::optional<AirGroundState> airground = AirGroundState::RESERVED;
    if (aircraft.airground_state.UpdateAge(now) < 30000)
        airground = aircraft.airground_state.Value();
    else
        airground = boost::none;

    boost::optional<int> groundspeed = -10000;
    if (aircraft.ground_speed.UpdateAge(now) < 30000)
        groundspeed = aircraft.ground_speed.Value();
    else
        groundspeed = boost::none;

    std::uint64_t minAge;
    if (immediate) {
        // a change we want to emit right away
        minAge = 0;
    } else if (airground && airground.get() == AirGroundState::ON_GROUND) {
        // we are probably on the ground, increase the update rate
        minAge = 1000;
    } else if (altitude && altitude.get() < 500 && (!groundspeed || groundspeed.get() < 200)) {
        // we are probably on the ground, increase the update rate
        minAge = 1000;
    } else if (groundspeed && groundspeed.get() < 100 && (!altitude || altitude.get() < 1000)) {
        // we are probably on the ground, increase the update rate
        minAge = 1000;
    } else if (!altitude || altitude.get() < 10000) {
        // Below 10000 feet, emit up to every 5s when changing, 10s otherwise
        minAge = changed ? 5000 : 10000;
    } else {
        // Above 10000 feet, emit up to every 10s when changing, 30s otherwise
        minAge = changed ? 10000 : 30000;
    }

    bool force_slow = (now - last.slow_report_time) > 300000;

    if ((now - last.report_time) < minAge) {
        // Not this time.
        return;
    }

    std::vector<std::pair<std::string, std::string>> kv;

    // clang-format off
    static std::map<AddressQualifier, std::string> source_map = {
        {AddressQualifier::ADSB_ICAO, "A"},
        {AddressQualifier::ADSB_OTHER, "A"},
        {AddressQualifier::ADSR_OTHER, "A"},
        {AddressQualifier::TISB_ICAO, "T"},
        {AddressQualifier::TISB_TRACKFILE, "T"}
    };
    // clang-format on

    std::string source = value_map(aircraft.address_qualifier, source_map, "?");

    auto add_slow_field = [&kv, &last, force_slow](const std::string &k, const AgedFieldBase &f, std::function<void(std::ostream &)> stringize) {
        if (f.Valid() && (force_slow || f.Changed() > last.report_time)) {
            std::ostringstream os;
            stringize(os);
            kv.emplace_back(k, os.str());
        }
    };

    auto add_slow_aged_field = [&kv, &source, &last, now, force_slow](const std::string &k, const AgedFieldBase &f, std::function<void(std::ostream &)> stringize) {
        if (f.Valid() && (force_slow || f.Changed() > last.report_time)) {
            std::ostringstream os;
            stringize(os);
            os << ' ' << (f.UpdateAge(now) / 1000) << ' ' << source;
            kv.emplace_back(k, os.str());
        }
    };

    auto add_aged_field = [&kv, &source, &last, now](const std::string &k, const AgedFieldBase &f, std::function<void(std::ostream &)> stringize) {
        if (f.Valid() && f.Updated() > last.report_time) {
            std::ostringstream os;
            stringize(os);
            os << ' ' << (f.UpdateAge(now) / 1000) << ' ' << source;
            kv.emplace_back(k, os.str());
        }
    };

    add_slow_field("uat_version", aircraft.mops_version, simple_emit(aircraft.mops_version));
    add_slow_field("category", aircraft.emitter_category, [&aircraft](std::ostream &os) {
        unsigned as_hex = 0xA0 + (aircraft.emitter_category.Value() & 7) + ((aircraft.emitter_category.Value() & 0x18) << 1);
        os << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << as_hex;
    });
    add_slow_aged_field("nac_p", aircraft.nac_p, simple_emit(aircraft.nac_p));
    add_slow_aged_field("nac_v", aircraft.nac_v, simple_emit(aircraft.nac_v));
    add_slow_aged_field("sil", aircraft.sil, simple_emit(aircraft.sil));
    add_slow_aged_field("sil_type", aircraft.sil_supplement, [&aircraft](std::ostream &os) {
        // clang-format off
        static std::map<SILSupplement, std::string> supplement_map = {
            {SILSupplement::PER_HOUR, "perhour"},
            {SILSupplement::PER_SAMPLE, "persample"},
        };
        // clang-format on
        os << value_map(aircraft.sil_supplement.Value(), supplement_map, "unknown");
    });
    add_slow_aged_field("nic_baro", aircraft.nic_baro, simple_emit(aircraft.nic_baro));

    add_aged_field("airGround", aircraft.airground_state, [&aircraft](std::ostream &os) {
        // clang-format off
        static std::map<AirGroundState, std::string> airground_map = {
            {AirGroundState::AIRBORNE_SUBSONIC, "A+"},
            {AirGroundState::AIRBORNE_SUPERSONIC, "A+"},
            {AirGroundState::ON_GROUND, "A+"}
        };
        // clang-format on
        os << value_map(aircraft.airground_state.Value(), airground_map, "?");
    });
    add_aged_field("squawk", aircraft.flightplan_id, [&aircraft](std::ostream &os) { os << '{' << aircraft.flightplan_id.Value() << '}'; });
    add_aged_field("ident", aircraft.callsign, [&aircraft](std::ostream &os) { os << '{' << aircraft.callsign.Value() << '}'; });
    add_aged_field("alt", aircraft.pressure_altitude, simple_emit(aircraft.pressure_altitude));
    add_aged_field("position", aircraft.position, [&aircraft](std::ostream &os) {
        auto &p = aircraft.position.Value();
        unsigned nic = aircraft.nic.Valid() ? aircraft.nic.Value() : 0; // should always be valid if the position is
        double rc = aircraft.horizontal_containment.Valid() ? aircraft.horizontal_containment.Value() : 0;
        os << '{' << std::fixed << std::setprecision(5) << p.first << ' ' << p.second << ' ' << nic << ' ' << std::setprecision(0) << std::ceil(rc) << '}';
    });
    add_aged_field("alt_gnss", aircraft.geometric_altitude, simple_emit(aircraft.geometric_altitude));
    add_aged_field("vrate", aircraft.vertical_velocity_barometric, simple_emit(aircraft.vertical_velocity_barometric));
    add_aged_field("vrate_geom", aircraft.vertical_velocity_geometric, simple_emit(aircraft.vertical_velocity_geometric));
    add_aged_field("speed", aircraft.ground_speed, simple_emit(aircraft.ground_speed));
    add_aged_field("track", aircraft.true_track, simple_emit(aircraft.true_track, 1));
    add_aged_field("heading_magnetic", aircraft.magnetic_heading, simple_emit(aircraft.magnetic_heading, 1));
    add_aged_field("heading_true", aircraft.true_heading, simple_emit(aircraft.true_heading, 1));
    add_aged_field("nav_alt_mcp", aircraft.selected_altitude_mcp, simple_emit(aircraft.selected_altitude_mcp));
    add_aged_field("nav_alt_fms", aircraft.selected_altitude_fms, simple_emit(aircraft.selected_altitude_fms));
    add_aged_field("nav_heading", aircraft.selected_heading, simple_emit(aircraft.selected_heading));
    add_aged_field("nav_modes", aircraft.mode_indicators, [&aircraft](std::ostream &os) {
        bool first = true;
        auto emit = [&os, &first](bool value, const std::string item) {
            if (value) {
                if (!first)
                    os << " ";
                os << item;
                first = false;
            }
        };

        auto &indicators = aircraft.mode_indicators.Value();
        os << "{";
        emit(indicators.autopilot, "autopilot");
        emit(indicators.vnav, "vnav");
        emit(indicators.altitude_hold, "althold");
        emit(indicators.approach, "approach");
        emit(indicators.lnav, "lnav");
        os << "}";
    });
    add_aged_field("nav_qnh", aircraft.barometric_pressure_setting, simple_emit(aircraft.barometric_pressure_setting, 1));
    add_aged_field("emergency", aircraft.emergency, [&aircraft](std::ostream &os) {
        // clang-format off
        static std::map<EmergencyPriorityStatus, std::string> emergency_map = {
            {EmergencyPriorityStatus::NONE, "none"},
            {EmergencyPriorityStatus::GENERAL, "general"},
            {EmergencyPriorityStatus::MEDICAL, "lifeguard"},
            {EmergencyPriorityStatus::MINFUEL, "minfuel"},
            {EmergencyPriorityStatus::NORDO, "nordo"},
            {EmergencyPriorityStatus::UNLAWFUL, "unlawful"},
            {EmergencyPriorityStatus::DOWNED, "downed"}};
        // clang-format on
        os << value_map(aircraft.emergency.Value(), emergency_map, "unknown");
    });

    // did we actually generate anything?
    if (kv.empty()) {
        return;
    }

    // generate the line
    std::cout << "_v" << '\t' << TSVVersion() << '\t' << "clock" << '\t' << (now / 1000) << '\t';

    bool icao = (aircraft.address_qualifier == AddressQualifier::ADSB_ICAO || aircraft.address_qualifier == AddressQualifier::TISB_ICAO);
    std::cout << (icao ? "hexid" : "otherid") << '\t' << std::hex << std::uppercase << std::setfill('0') << std::setw(6) << aircraft.address << std::dec << std::nouppercase << std::setfill(' ');

    if (force_slow || !icao) {
        // clang-format off
        static std::map<AddressQualifier, std::string> qualifier_map = {
            {AddressQualifier::ADSB_ICAO, "adsb_icao"},
            {AddressQualifier::ADSB_OTHER, "adsb_other"},
            {AddressQualifier::TISB_ICAO, "tisb_icao"},
            {AddressQualifier::TISB_TRACKFILE, "tisb_trackfile"},
            {AddressQualifier::VEHICLE, "vehicle"},
            {AddressQualifier::FIXED_BEACON, "fixed_beacon"},
            {AddressQualifier::ADSR_OTHER, "adsr_other"}
        };
        // clang-format on
        std::cout << '\t' << "addrtype" << '\t' << value_map(aircraft.address_qualifier, qualifier_map, "unknown");
    }

    for (const auto &entry : kv) {
        std::cout << '\t' << entry.first << '\t' << entry.second;
    }

    std::cout << std::endl;

    if (force_slow)
        last.slow_report_time = now;
    last.report_time = now;
    last.report_state = aircraft;
}
