// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#include "skyaware_writer.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <boost/asio/bind_executor.hpp>

#include "json.hpp"

#include "track.h"

using namespace flightaware::uat;
using namespace flightaware::skyaware;

void SkyAwareWriter::Start() {
    nlohmann::json receiver_json;

    receiver_json["version"] = "dump978 " VERSION;
    receiver_json["refresh"] = interval_.count();
    receiver_json["history"] = history_count_;

    if (location_) {
        receiver_json["lat"] = RoundN(location_->first, 4);
        receiver_json["lon"] = RoundN(location_->second, 4);
    }

    std::ofstream receiver_file((dir_ / "receiver.json").native());
    receiver_file << std::setw(4) << receiver_json << std::endl;

    PeriodicWrite();
}

void SkyAwareWriter::Stop() { timer_.cancel(); }

void SkyAwareWriter::PeriodicWrite() {
    using json = nlohmann::json;
    json aircraft_json;

    auto now = now_millis();

    aircraft_json["now"] = now / 1000.0;
    aircraft_json["messages"] = tracker_->TotalMessages();

    auto &aircraft_list = aircraft_json["aircraft"] = json::array();

    for (const auto &entry : tracker_->Aircraft()) {
        auto &aircraft = entry.second;

        if (aircraft.messages < 2) {
            // possibly noise
            continue;
        }

        aircraft_list.emplace_back(json::object());
        auto &ac_json = aircraft_list.back();

        std::ostringstream os;
        if (aircraft.address_qualifier != AddressQualifier::ADSB_ICAO && aircraft.address_qualifier != AddressQualifier::TISB_ICAO)
            os << '~';
        os << std::hex << std::setfill('0') << std::setw(6) << aircraft.address;
        ac_json["hex"] = os.str();

        // qualifier..
        switch (aircraft.address_qualifier) {
        case AddressQualifier::TISB_ICAO:
            ac_json["type"] = "tisb_icao";
            ac_json["tisb"] = json::array({"lat", "lon"});
            break;
        case AddressQualifier::TISB_TRACKFILE:
            ac_json["type"] = "tisb_trackfile";
            ac_json["tisb"] = json::array({"lat", "lon"});
            break;
        case AddressQualifier::ADSB_ICAO:
            ac_json["type"] = "adsb_icao";
            break;
        default:
            break;
        }

        const std::uint64_t max_age = 60000;

        if (aircraft.position.UpdateAge(now) < max_age) {
            ac_json["lat"] = aircraft.position.Value().first;
            ac_json["lon"] = aircraft.position.Value().second;
            ac_json["seen_pos"] = aircraft.position.UpdateAge(now) / 1000.0;
        }
        if (aircraft.pressure_altitude.UpdateAge(now) < max_age) {
            ac_json["alt_baro"] = aircraft.pressure_altitude.Value();
        }
        if (aircraft.geometric_altitude.UpdateAge(now) < max_age) {
            ac_json["alt_geom"] = aircraft.pressure_altitude.Value();
        }
        if (aircraft.nic.UpdateAge(now) < max_age) {
            ac_json["nic"] = aircraft.nic.Value();
        }
        if (aircraft.airground_state.UpdateAge(now) < max_age && aircraft.airground_state.Value() == AirGroundState::ON_GROUND) {
            ac_json["alt_baro"] = "ground";
        }
        if (aircraft.vertical_velocity_barometric.UpdateAge(now) < max_age) {
            ac_json["baro_rate"] = aircraft.vertical_velocity_barometric.Value();
        }
        if (aircraft.vertical_velocity_geometric.UpdateAge(now) < max_age) {
            ac_json["geom_rate"] = aircraft.vertical_velocity_geometric.Value();
        }
        if (aircraft.ground_speed.UpdateAge(now) < max_age) {
            ac_json["gs"] = aircraft.ground_speed.Value();
        }
        if (aircraft.magnetic_heading.UpdateAge(now) < max_age) {
            ac_json["mag_heading"] = aircraft.magnetic_heading.Value();
        }
        if (aircraft.true_heading.UpdateAge(now) < max_age) {
            ac_json["true_heading"] = aircraft.true_heading.Value();
        }
        if (aircraft.true_track.UpdateAge(now) < max_age) {
            ac_json["track"] = aircraft.true_track.Value();
        }
        if (aircraft.emitter_category.UpdateAge(now) < max_age) {
            ac_json["category"] = std::string{(char)('A' + (aircraft.emitter_category.Value() >> 3)), (char)('0' + (aircraft.emitter_category.Value() & 7))};
        }
        if (aircraft.callsign.UpdateAge(now) < max_age) {
            ac_json["flight"] = aircraft.callsign.Value();
        }
        if (aircraft.flightplan_id.UpdateAge(now) < max_age) {
            ac_json["squawk"] = aircraft.flightplan_id.Value();
        }
        if (aircraft.emergency.UpdateAge(now) < max_age) {
            ac_json["emergency"] = aircraft.emergency.Value();
        }
        if (aircraft.mops_version.UpdateAge(now) < max_age) {
            ac_json["uat_version"] = aircraft.mops_version.Value();
        }
        if (aircraft.sil.UpdateAge(now) < max_age) {
            ac_json["sil"] = aircraft.sil.Value();
        }
        if (aircraft.sda.UpdateAge(now) < max_age) {
            ac_json["sda"] = aircraft.sda.Value();
        }
        if (aircraft.sda.UpdateAge(now) < max_age) {
            ac_json["nac_p"] = aircraft.nac_p.Value();
        }
        if (aircraft.sda.UpdateAge(now) < max_age) {
            ac_json["nac_v"] = aircraft.nac_v.Value();
        }
        if (aircraft.sda.UpdateAge(now) < max_age) {
            ac_json["nic_baro"] = aircraft.nic_baro.Value();
        }
        if (aircraft.sil_supplement.UpdateAge(now) < max_age) {
            ac_json["sil_type"] = aircraft.sil_supplement.Value();
        }
        if (aircraft.gva.UpdateAge(now) < max_age) {
            ac_json["gva"] = aircraft.gva.Value();
        }
        if (aircraft.selected_altitude_mcp.UpdateAge(now) < max_age) {
            ac_json["nav_altitude_mcp"] = aircraft.selected_altitude_mcp.Value();
        }
        if (aircraft.selected_altitude_fms.UpdateAge(now) < max_age) {
            ac_json["nav_altitude_fms"] = aircraft.selected_altitude_fms.Value();
        }
        if (aircraft.barometric_pressure_setting.UpdateAge(now) < max_age) {
            ac_json["nav_qnh"] = aircraft.barometric_pressure_setting.Value();
        }
        if (aircraft.selected_heading.UpdateAge(now) < max_age) {
            ac_json["nav_heading"] = aircraft.selected_heading.Value();
        }
        if (aircraft.mode_indicators.UpdateAge(now) < max_age) {
            auto &modes = aircraft.mode_indicators.Value();
            auto &modes_json = ac_json["nav_modes"] = json::array();
            if (modes.autopilot) {
                modes_json.emplace_back("autopliot");
            }
            if (modes.vnav) {
                modes_json.emplace_back("vnav");
            }
            if (modes.altitude_hold) {
                modes_json.emplace_back("althold");
            }
            if (modes.approach) {
                modes_json.emplace_back("approach");
            }
            if (modes.lnav) {
                modes_json.emplace_back("lnav");
            }
        }
        // FIXME: tcas
        if (aircraft.horizontal_containment.UpdateAge(now) < max_age) {
            ac_json["rc"] = aircraft.horizontal_containment.Value();
        }

        ac_json["messages"] = aircraft.messages;
        ac_json["seen"] = (now - aircraft.last_message_time) / 1000.0;

        ac_json["rssi"] = RoundN(aircraft.AverageRssi(), 1);
    }

    auto temp_path = dir_ / "aircraft.json.new";
    auto target_path = dir_ / "aircraft.json";

    std::ofstream aircraft_file(temp_path.native());
    aircraft_file << aircraft_json << std::endl;
    aircraft_file.close();
    boost::filesystem::rename(temp_path, target_path);

    if (next_history_time_ <= now) {
        auto temp_history_path = dir_ / "history.json.new";
        auto history_path = dir_ / ("history_" + std::to_string(next_history_index_) + ".json");

        std::ofstream history_file(temp_history_path.native());
        history_file << aircraft_json << std::endl;
        history_file.close();
        boost::filesystem::rename(temp_history_path, history_path);

        next_history_index_ = (next_history_index_ + 1) % history_count_;
        next_history_time_ = now + history_interval_.count();
    }

    auto self(shared_from_this());
    timer_.expires_after(interval_);
    timer_.async_wait(boost::asio::bind_executor(strand_, [this, self](const boost::system::error_code &ec) {
        if (!ec) {
            PeriodicWrite();
        }
    }));
}
