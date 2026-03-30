// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "uat_message.h"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <array>

#include <boost/io/ios_state.hpp>

using namespace flightaware::uat;

//
// streaming raw messages
//

std::ostream &flightaware::uat::operator<<(std::ostream &os, const RawMessage &message) {
    boost::io::ios_flags_saver ifs(os);

    switch (message.Type()) {
    case MessageType::DOWNLINK_SHORT:
    case MessageType::DOWNLINK_LONG:
        os << '-';
        break;
    case MessageType::UPLINK:
        os << '+';
        break;
    case MessageType::METADATA:
        os << '!';
        break;
    default:
        throw std::logic_error("unexpected message type");
    }

    if (message.Type() != MessageType::METADATA) {
        os << std::setfill('0');
        for (auto b : message.Payload()) {
            os << std::hex << std::setw(2) << (int)b;
        }

        os << ";";
    }

    if (message.Errors() > 0) {
        os << "rs=" << std::dec << std::setw(0) << message.Errors() << ';';
    }
    if (message.Rssi() != 0) {
        os << "rssi=" << std::dec << std::setprecision(1) << std::fixed << message.Rssi() << ';';
    }
    if (message.ReceivedAt() != 0) {
        os << "t=" << std::dec << std::setw(0) << (message.ReceivedAt() / 1000) << '.' << std::setfill('0') << std::setw(3) << (message.ReceivedAt() % 1000) << ';';
    }
    if (message.RawTimestamp() != 0) {
        os << "rt=" << std::dec << std::setw(0) << message.RawTimestamp() << ';';
    }
    for (auto &i : message.Metadata()) {
        os << i.first << '=' << i.second << ';';
    }

    return os;
}

//
// decoding messages
//

AdsbMessage::AdsbMessage(const RawMessage &raw) {
    if (raw.Type() != MessageType::DOWNLINK_SHORT && raw.Type() != MessageType::DOWNLINK_LONG) {
        throw std::logic_error("can't parse this sort of message as a downlink ADS-B message");
    }

    // Metadata
    received_at = raw.ReceivedAt();
    raw_timestamp = raw.RawTimestamp();
    errors = raw.Errors();
    rssi = raw.Rssi();

    // HDR
    payload_type = raw.Bits(1, 1, 1, 5);
    address_qualifier = static_cast<AddressQualifier>(raw.Bits(1, 6, 1, 8));
    address = raw.Bits(2, 1, 4, 8);

    // Optional parts of the message
    // DO-282B Table 2-10 "Composition of the ADS-B Payload"
    switch (payload_type) {
    case 0:
        DecodeSV(raw);
        break;
    case 1:
        DecodeSV(raw);
        DecodeMS(raw);
        DecodeAUXSV(raw);
        break;
    case 2:
        DecodeSV(raw);
        DecodeAUXSV(raw);
        break;
    case 3:
        DecodeSV(raw);
        DecodeMS(raw);
        DecodeTS(raw, 30);
        break;
    case 4:
        DecodeSV(raw);
        DecodeTS(raw, 30);
        break;
    case 5:
        DecodeSV(raw);
        DecodeAUXSV(raw);
        break;
    case 6:
        DecodeSV(raw);
        DecodeTS(raw, 25);
        DecodeAUXSV(raw);
        break;
    case 7:
    case 8:
    case 9:
    case 10:
        DecodeSV(raw);
        break;

    default:
        // 11..31, HDR only
        break;
    }
}

void AdsbMessage::DecodeSV(const RawMessage &raw) {
    auto raw_lat = raw.Bits(5, 1, 7, 7);
    auto raw_lon = raw.Bits(7, 8, 10, 7);

    auto raw_alt = raw.Bits(11, 1, 12, 4);
    if (raw_alt != 0) {
        auto altitude = (raw_alt - 41) * 25;
        if (raw.Bit(10, 8)) { // 2.2.4.5.2.2 "ALTITUDE TYPE" field
            geometric_altitude = altitude;
        } else {
            pressure_altitude = altitude;
        }
    }

    nic = raw.Bits(12, 5, 12, 8);

    if (raw_lat != 0 || raw_lon != 0 || *nic != 0) {
        // NB: north and south pole encode identically. We return north pole in this
        // case
        auto lat = raw_lat * 360.0 / 16777216.0;
        if (lat > 90)
            lat -= 180;

        auto lon = raw_lon * 360.0 / 16777216.0;
        if (lon > 180)
            lon -= 360;

        position = std::make_pair<>(RoundN(lat, 5), RoundN(lon, 5));
    }

    airground_state = static_cast<AirGroundState>(raw.Bits(13, 1, 13, 2));

    // bit 13,3 reserved

    switch (*airground_state) {
    case AirGroundState::AIRBORNE_SUBSONIC:
    case AirGroundState::AIRBORNE_SUPERSONIC: {
        int supersonic = (*airground_state == AirGroundState::AIRBORNE_SUPERSONIC ? 4 : 1);
        int ns_sign = raw.Bit(13, 4) ? -1 : 1;
        auto raw_ns = raw.Bits(13, 5, 14, 6);
        if (raw_ns != 0)
            north_velocity = supersonic * ns_sign * (raw_ns - 1);

        int ew_sign = raw.Bit(14, 7) ? -1 : 1;
        auto raw_ew = raw.Bits(14, 8, 16, 1);
        if (raw_ew != 0)
            east_velocity = supersonic * ew_sign * (raw_ew - 1);

        // derive groundspeed, true track from north/east velocity for convenience
        if (north_velocity && east_velocity) { // nb: testing for presence, not non-zero value
            ground_speed = RoundN(std::sqrt(1.0 * (*north_velocity) * (*north_velocity) + 1.0 * (*east_velocity) * (*east_velocity)), 1);
            auto angle = std::atan2(*east_velocity, *north_velocity) * 180.0 / M_PI;
            if (angle < 0)
                angle += 360.0;
            true_track = RoundN(angle, 1);
        }

        vv_src = static_cast<VerticalVelocitySource>(raw.Bits(16, 2, 16, 2));
        int vv_sign = raw.Bit(16, 3) ? -1 : 1;
        auto raw_vv = raw.Bits(16, 4, 17, 4);
        if (raw_vv != 0) {
            auto vertical_velocity = vv_sign * (raw_vv - 1) * 64;
            switch (vv_src.get()) {
            case VerticalVelocitySource::BAROMETRIC:
                vertical_velocity_barometric = vertical_velocity;
                break;
            case VerticalVelocitySource::GEOMETRIC:
                vertical_velocity_geometric = vertical_velocity;
                break;
            default:
                break;
            }
        }

        break;
    }

    case AirGroundState::ON_GROUND: {
        // 13,4 reserved
        auto raw_gs = raw.Bits(13, 5, 14, 6);
        if (raw_gs != 0)
            ground_speed = (raw_gs - 1);

        auto tah_type = raw.Bits(14, 7, 14, 8);
        auto angle = RoundN(raw.Bits(15, 1, 16, 1) * 360.0 / 512.0, 1);
        switch (tah_type) { // 2.2.4.5.2.6.4 / Table 2-28 "Track Angle/Heading Type"
        case 0:             // data unavailable
            break;
        case 1: // true track
            true_track = angle;
            break;
        case 2: // magnetic heading
            magnetic_heading = angle;
            break;
        case 3: // true heading
            true_heading = angle;
            break;
        }

        auto raw_av_size = raw.Bits(16, 2, 16, 5);
        if (raw_av_size != 0) {
            // DO-282B Table 2-35
            static std::array<std::pair<double, double>, 16> aircraft_sizes = {{{0, 0}, // no data
                                                                                {15, 23},
                                                                                {25, 28.5},
                                                                                {25, 34},
                                                                                {35, 33},
                                                                                {35, 38},
                                                                                {45, 39.5},
                                                                                {45, 45},
                                                                                {55, 45},
                                                                                {55, 52},
                                                                                {65, 59.5},
                                                                                {65, 67},
                                                                                {75, 72.5},
                                                                                {75, 80},
                                                                                {85, 80},
                                                                                {85, 90}}};

            aircraft_size = aircraft_sizes[raw_av_size];
        }

        if (raw.Bit(16, 7)) {
            // Longitudinal GPS offset
            auto raw_gps_long = raw.Bits(16, 8, 17, 4);
            if (raw_gps_long != 0) {
                if (raw_gps_long == 1) {
                    gps_position_offset_applied = true;
                } else {
                    gps_position_offset_applied = false;
                    gps_longitudinal_offset = (raw_gps_long - 1) * 2;
                }
            }
        } else {
            // Lateral GPS offset
            // We adopt the convention that left is negative
            auto raw_gps_lat = raw.Bits(16, 8, 17, 2);
            if (raw_gps_lat != 0) {
                if (raw_gps_lat <= 3) {
                    gps_lateral_offset = raw_gps_lat * -2;
                } else {
                    gps_lateral_offset = (raw_gps_lat - 4) * 2;
                }
            }
        }

        break;
    }

    default:
        // nothing;
        break;
    }

    switch (address_qualifier) {
    case AddressQualifier::ADSB_ICAO:
    case AddressQualifier::ADSB_OTHER:
    case AddressQualifier::VEHICLE:
    case AddressQualifier::FIXED_BEACON:
        utc_coupled = raw.Bit(17, 5);
        uplink_feedback = raw.Bits(17, 6, 17, 8);
        break;

    case AddressQualifier::TISB_ICAO:
    case AddressQualifier::TISB_TRACKFILE:
    case AddressQualifier::ADSR_OTHER:
        tisb_site_id = raw.Bits(17, 5, 17, 8);
        break;

    default:
        // nothing
        break;
    }
}

void AdsbMessage::DecodeTS(const RawMessage &raw, unsigned startbyte) {
    // TS starts at byte 30 (§2.2.4.5.6) in payload type 3 or 4;
    // or at byte 25 (§2.2.4.5.7) in payload type 6;
    // the starting offset to use is passed by the caller

    auto raw_altitude = raw.Bits(startbyte + 0, 2, startbyte + 1, 4);
    if (raw_altitude != 0) {
        selected_altitude_type = static_cast<SelectedAltitudeType>(raw.Bits(startbyte + 0, 1, startbyte + 0, 1));
        switch (*selected_altitude_type) {
        case SelectedAltitudeType::MCP_FCU:
            selected_altitude_mcp = (raw_altitude - 1) * 32;
            break;
        case SelectedAltitudeType::FMS:
            selected_altitude_fms = (raw_altitude - 1) * 32;
            break;
        default:
            break;
        }
    }

    auto raw_bps = raw.Bits(startbyte + 1, 5, startbyte + 2, 5);
    if (raw_bps != 0)
        barometric_pressure_setting = 800 + (raw_bps - 1) * 0.8;

    if (raw.Bit(startbyte + 2, 6)) {
        int heading_sign = raw.Bit(startbyte + 2, 7) ? -1 : 1;
        auto heading = RoundN(raw.Bits(startbyte + 2, 8, startbyte + 3, 7) * 180.0 / 256.0, 1);
        selected_heading = heading_sign * heading;
    }

    if (raw.Bit(startbyte + 3, 8)) {
        mode_indicators.emplace();
        mode_indicators->autopilot = raw.Bit(startbyte + 4, 1);
        mode_indicators->vnav = raw.Bit(startbyte + 4, 2);
        mode_indicators->altitude_hold = raw.Bit(startbyte + 4, 3);
        mode_indicators->approach = raw.Bit(startbyte + 4, 4);
        mode_indicators->lnav = raw.Bit(startbyte + 4, 5);
    }

    // 34,6 .. 34,8 reserved
}

static inline bool IsOctal(char ch) { return (ch >= '0' && ch <= '7'); }

void AdsbMessage::DecodeMS(const RawMessage &raw) {
    static const char *base40_alphabet = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ *??";
    auto raw1 = raw.Bits(18, 1, 19, 8);
    auto raw2 = raw.Bits(20, 1, 21, 8);
    auto raw3 = raw.Bits(22, 1, 23, 8);

    emitter_category = (raw1 / 1600) % 40;

    std::string raw_callsign;
    raw_callsign.reserve(8);
    raw_callsign.push_back(base40_alphabet[(raw1 / 40) % 40]);
    raw_callsign.push_back(base40_alphabet[raw1 % 40]);
    raw_callsign.push_back(base40_alphabet[(raw2 / 1600) % 40]);
    raw_callsign.push_back(base40_alphabet[(raw2 / 40) % 40]);
    raw_callsign.push_back(base40_alphabet[raw2 % 40]);
    raw_callsign.push_back(base40_alphabet[(raw3 / 1600) % 40]);
    raw_callsign.push_back(base40_alphabet[(raw3 / 40) % 40]);
    raw_callsign.push_back(base40_alphabet[raw3 % 40]);

    // trim trailing spaces and code 37
    while (!raw_callsign.empty() && (raw_callsign.back() == ' ' || raw_callsign.back() == '*')) {
        raw_callsign.pop_back();
    }

    if (!raw_callsign.empty()) {
        if (raw.Bit(27,
                    7)) { // CSID field, 1 = callsign, 0 = flightplan ID (aka squawk)
            callsign.emplace(std::move(raw_callsign));
        } else {
            // Enforce 4 digit octal squawk
            if (raw_callsign.size() == 4 &&
                IsOctal(raw_callsign[0]) &&
                IsOctal(raw_callsign[1]) &&
                IsOctal(raw_callsign[2]) &&
                IsOctal(raw_callsign[3])) {
                    flightplan_id.emplace(std::move(raw_callsign));
            }
        }
    }

    emergency = static_cast<EmergencyPriorityStatus>(raw.Bits(24, 1, 24, 3));
    mops_version = raw.Bits(24, 4, 24, 6);
    sil = raw.Bits(24, 7, 24, 8);
    transmit_mso = raw.Bits(25, 1, 25, 6);
    sda = raw.Bits(25, 7, 25, 8);
    nac_p = raw.Bits(26, 1, 26, 4);
    nac_v = raw.Bits(26, 5, 26, 7);
    nic_baro = raw.Bits(26, 8, 26, 8);

    capability_codes.emplace();
    capability_codes->uat_in = raw.Bit(27, 1);
    capability_codes->es_in = raw.Bit(27, 2);
    capability_codes->tcas_operational = raw.Bit(27, 3);

    operational_modes.emplace();
    operational_modes->tcas_ra_active = raw.Bit(27, 4);
    operational_modes->ident_active = raw.Bit(27, 5);
    operational_modes->atc_services = raw.Bit(27, 6);

    sil_supplement = static_cast<SILSupplement>(raw.Bits(27, 8, 27, 8));
    gva = raw.Bits(28, 1, 28, 2);
    single_antenna = raw.Bit(28, 3);
    nic_supplement = raw.Bit(28, 4);
    // 28,5 .. 29,8 reserved
}

void AdsbMessage::DecodeAUXSV(const RawMessage &raw) {
    auto raw_alt = raw.Bits(30, 1, 31, 4);
    if (raw_alt != 0) {
        auto altitude = (raw_alt - 41) * 25;
        if (raw.Bit(10, 8)) { // 2.2.4.5.2.2 "ALTITUDE TYPE" field (in SV, which is
            // always present when AUXSV is present)
            pressure_altitude = altitude;
        } else {
            geometric_altitude = altitude;
        }
    }
}

//
// converting decoded messages to json
//

namespace flightaware {
  namespace uat {
    // clang-format off

    NLOHMANN_JSON_SERIALIZE_ENUM(AddressQualifier, {
        {AddressQualifier::INVALID, "invalid"},
        {AddressQualifier::ADSB_ICAO, "adsb_icao"},
        {AddressQualifier::ADSB_OTHER, "adsb_other"},
        {AddressQualifier::TISB_ICAO, "tisb_icao"},
        {AddressQualifier::TISB_TRACKFILE, "tisb_trackfile"},
        {AddressQualifier::VEHICLE, "vehicle"},
        {AddressQualifier::FIXED_BEACON, "fixed_beacon"},
        {AddressQualifier::ADSR_OTHER, "adsr_other"},
        {AddressQualifier::RESERVED, "reserved"},
    });

    NLOHMANN_JSON_SERIALIZE_ENUM(AirGroundState, {
        {AirGroundState::INVALID, "invalid"},
        {AirGroundState::AIRBORNE_SUBSONIC, "airborne"},
        {AirGroundState::AIRBORNE_SUPERSONIC, "supersonic"},
        {AirGroundState::ON_GROUND, "ground"},
        {AirGroundState::RESERVED, "reserved"}
    });

    NLOHMANN_JSON_SERIALIZE_ENUM(VerticalVelocitySource, {
        {VerticalVelocitySource::INVALID, "invalid"},
        {VerticalVelocitySource::GEOMETRIC, "geometric"},
        {VerticalVelocitySource::BAROMETRIC, "barometric"}
    });

    NLOHMANN_JSON_SERIALIZE_ENUM(EmergencyPriorityStatus, {
        {EmergencyPriorityStatus::INVALID, "invalid"},
        {EmergencyPriorityStatus::NONE, "none"},
        {EmergencyPriorityStatus::GENERAL, "general"},
        {EmergencyPriorityStatus::MEDICAL, "medical"},
        {EmergencyPriorityStatus::NORDO, "nordo"},
        {EmergencyPriorityStatus::UNLAWFUL, "unlawful"},
        {EmergencyPriorityStatus::DOWNED, "downed"},
        {EmergencyPriorityStatus::RESERVED, "reserved"}
    });

    NLOHMANN_JSON_SERIALIZE_ENUM(SILSupplement, {
        {SILSupplement::INVALID, "invalid"},
        {SILSupplement::PER_HOUR, "per_hour"},
        {SILSupplement::PER_SAMPLE, "per_sample"}
    });

    NLOHMANN_JSON_SERIALIZE_ENUM(SelectedAltitudeType, {
        {SelectedAltitudeType::INVALID, "invalid"},
        {SelectedAltitudeType::MCP_FCU, "mcp_fcu"},
        {SelectedAltitudeType::FMS, "fms"}
    });

    // clang-format on
  }; // namespace flightaware::uat
}; // namespace flightaware

nlohmann::json AdsbMessage::ToJson() const {
    nlohmann::json o;

    o["address_qualifier"] = address_qualifier;

    std::ostringstream os;
    os << std::hex << std::setfill('0') << std::setw(6) << address;
    o["address"] = os.str();

#define EMIT(x)         \
    do {                \
        if (x) {        \
            o[#x] = *x; \
        }               \
    } while (0)

    if (position)
        o["position"] = {{"lat", position->first}, {"lon", position->second}};

    EMIT(pressure_altitude);
    EMIT(geometric_altitude);
    EMIT(nic);
    EMIT(airground_state);
    EMIT(north_velocity);
    EMIT(east_velocity);
    EMIT(vv_src);
    EMIT(vertical_velocity_barometric);
    EMIT(vertical_velocity_geometric);
    EMIT(ground_speed);
    EMIT(magnetic_heading);
    EMIT(true_heading);
    EMIT(true_track);

    if (aircraft_size)
        o["aircraft_size"] = {{"length", aircraft_size->first}, {"width", aircraft_size->second}};

    EMIT(gps_lateral_offset);
    EMIT(gps_longitudinal_offset);
    EMIT(gps_position_offset_applied);
    EMIT(utc_coupled);
    EMIT(uplink_feedback);
    EMIT(tisb_site_id);

    if (emitter_category) {
        o["emitter_category"] = std::string{(char)('A' + (*emitter_category >> 3)), (char)('0' + (*emitter_category & 7))};
    }

    EMIT(callsign);
    EMIT(flightplan_id);
    EMIT(emergency);
    EMIT(mops_version);
    EMIT(sil);
    EMIT(transmit_mso);
    EMIT(sda);
    EMIT(nac_p);
    EMIT(nac_v);
    EMIT(nic_baro);

    if (capability_codes) {
        auto &cc = o["capability_codes"] = nlohmann::json::object();
        cc["uat_in"] = capability_codes->uat_in;
        cc["es_in"] = capability_codes->es_in;
        cc["tcas_operational"] = capability_codes->tcas_operational;
    }

    if (operational_modes) {
        auto &om = o["operational_modes"] = nlohmann::json::object();
        om["tcas_ra_active"] = operational_modes->tcas_ra_active;
        om["ident_active"] = operational_modes->ident_active;
        om["atc_services"] = operational_modes->atc_services;
    }

    EMIT(sil_supplement);
    EMIT(gva);
    EMIT(single_antenna);
    EMIT(nic_supplement);
    EMIT(selected_altitude_type);
    EMIT(selected_altitude_mcp);
    EMIT(selected_altitude_fms);
    EMIT(barometric_pressure_setting);
    EMIT(selected_heading);

    if (mode_indicators) {
        // clang-format off
        o["mode_indicators"] = {
            { "autopilot", mode_indicators->autopilot },
            { "vnav", mode_indicators->vnav },
            { "altitude_hold", mode_indicators->altitude_hold },
            { "approach", mode_indicators->approach },
            { "lnav", mode_indicators->lnav }
        };
        // clang-format on
    }

#undef EMIT

    // clang-format off
    o["metadata"] = {
        { "rssi", RoundN(rssi, 1) },
        { "errors", errors }
    };
    // clang-format on

    if (received_at != 0) {
        o["metadata"]["received_at"] = received_at / 1000.0;
    }
    if (raw_timestamp != 0) {
        o["metadata"]["raw_timestamp"] = raw_timestamp;
    }

    return o;
}
