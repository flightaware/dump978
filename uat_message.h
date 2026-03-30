// -*- c++ -*-

// Copyright (c) 2019, FlightAware LLC.
// All rights reserved.
// Licensed under the 2-clause BSD license; see the LICENSE file

#ifndef UAT_MESSAGE_H
#define UAT_MESSAGE_H

#include <cstdint>
#include <vector>

#include <boost/optional.hpp>

#include <json.hpp>

#include "common.h"
#include "uat_protocol.h"

namespace flightaware {
  namespace uat {
    class RawMessage {
      public:
        using MetadataMap = std::map<std::string, std::string>;

        RawMessage() : type_(MessageType::INVALID), received_at_(0), errors_(0), rssi_(0), raw_timestamp_(0) {}

        RawMessage(const Bytes &payload, std::uint64_t received_at, unsigned errors, float rssi, std::uint64_t raw_timestamp = 0) : payload_(payload), received_at_(received_at), errors_(errors), rssi_(rssi), raw_timestamp_(raw_timestamp) {
            switch (payload_.size()) {
            case DOWNLINK_SHORT_DATA_BYTES:
                type_ = MessageType::DOWNLINK_SHORT;
                break;
            case DOWNLINK_LONG_DATA_BYTES:
                type_ = MessageType::DOWNLINK_LONG;
                break;
            case UPLINK_DATA_BYTES:
                type_ = MessageType::UPLINK;
                break;
            default:
                type_ = MessageType::INVALID;
                break;
            }
        }

        RawMessage(Bytes &&payload, std::uint64_t received_at, unsigned errors, float rssi, std::uint64_t raw_timestamp = 0) : payload_(std::move(payload)), received_at_(received_at), errors_(errors), rssi_(rssi), raw_timestamp_(raw_timestamp) {
            switch (payload_.size()) {
            case DOWNLINK_SHORT_DATA_BYTES:
                type_ = MessageType::DOWNLINK_SHORT;
                break;
            case DOWNLINK_LONG_DATA_BYTES:
                type_ = MessageType::DOWNLINK_LONG;
                break;
            case UPLINK_DATA_BYTES:
                type_ = MessageType::UPLINK;
                break;
            default:
                type_ = MessageType::INVALID;
                break;
            }
        }

        RawMessage(MetadataMap &&metadata) : type_(MessageType::METADATA), received_at_(0), errors_(0), rssi_(0.0), raw_timestamp_(0), metadata_(std::move(metadata)) {}

        RawMessage(const MetadataMap &metadata) : type_(MessageType::METADATA), received_at_(0), errors_(0), rssi_(0.0), raw_timestamp_(0), metadata_(metadata) {}

        MessageType Type() const { return type_; }

        Bytes &Payload() { return payload_; }

        const Bytes &Payload() const { return payload_; }

        std::uint64_t ReceivedAt() const { return received_at_; }

        unsigned Errors() const { return errors_; }

        float Rssi() const { return rssi_; }

        std::uint64_t RawTimestamp() const { return raw_timestamp_; }

        const MetadataMap &Metadata() const { return metadata_; }

        // Number of raw bits in the message, excluding the sync bits
        unsigned BitLength() const {
            switch (type_) {
            case MessageType::DOWNLINK_SHORT:
                return DOWNLINK_SHORT_BITS;
            case MessageType::DOWNLINK_LONG:
                return DOWNLINK_LONG_BITS;
            case MessageType::UPLINK:
                return UPLINK_BITS;
            default:
                return 0;
            }
        }

        operator bool() const { return (type_ != MessageType::INVALID); }

        __attribute__((always_inline)) bool Bit(unsigned byte, unsigned bit) const {
            assert(byte >= 1);
            assert(bit >= 1);
            assert(bit <= 8);

            const unsigned bi = (byte - 1) * 8 + bit - 1;
            const unsigned by = bi >> 3;
            const unsigned mask = 1 << (7 - (bi & 7));

            return (payload_.at(by) & mask) != 0;
        }

        __attribute__((always_inline)) std::uint32_t Bits(unsigned first_byte, unsigned first_bit, unsigned last_byte, unsigned last_bit) const {
            assert(first_byte >= 1);
            assert(first_bit >= 1);
            assert(first_bit <= 8);
            assert(last_byte >= 1);
            assert(last_bit >= 1);
            assert(last_bit <= 8);

            const unsigned fbi = (first_byte - 1) * 8 + first_bit - 1;
            const unsigned lbi = (last_byte - 1) * 8 + last_bit - 1;
            assert(fbi <= lbi);

            const unsigned nbi = (lbi - fbi + 1);
            assert(nbi > 0);
            assert(nbi <= 32);

            const unsigned fby = fbi >> 3;
            const unsigned lby = lbi >> 3;
            const unsigned nby = (lby - fby) + 1;

            const unsigned shift = 7 - (lbi & 7);
            const unsigned topmask = 0xFF >> (fbi & 7);

            assert(nby > 0);
            assert(nby <= 5);

            if (payload_.size() < fby + nby)
                throw std::out_of_range("bit range exceeds available data");

            if (nby == 5) {
                return ((payload_[fby] & topmask) << (32 - shift)) | (payload_[fby + 1] << (24 - shift)) | (payload_[fby + 2] << (16 - shift)) | (payload_[fby + 3] << (8 - shift)) | (payload_[fby + 4] >> shift);
            } else if (nby == 4) {
                return ((payload_[fby] & topmask) << (24 - shift)) | (payload_[fby + 1] << (16 - shift)) | (payload_[fby + 2] << (8 - shift)) | (payload_[fby + 3] >> shift);
            } else if (nby == 3) {
                return ((payload_[fby] & topmask) << (16 - shift)) | (payload_[fby + 1] << (8 - shift)) | (payload_[fby + 2] >> shift);
            } else if (nby == 2) {
                return ((payload_[fby] & topmask) << (8 - shift)) | (payload_[fby + 1] >> shift);
            } else if (nby == 1) {
                return (payload_[fby] & topmask) >> shift;
            } else {
                return 0;
            }
        }

      private:
        MessageType type_;
        Bytes payload_;
        std::uint64_t received_at_;
        unsigned errors_;
        float rssi_;
        std::uint64_t raw_timestamp_;
        MetadataMap metadata_;
    };

    std::ostream &operator<<(std::ostream &os, const RawMessage &message);

    typedef std::vector<RawMessage> MessageVector;
    typedef std::shared_ptr<MessageVector> SharedMessageVector;

    // 2.2.4.5.1.2 "ADDRESS QUALIFIER" field
    enum class AddressQualifier : unsigned char { ADSB_ICAO = 0, ADSB_OTHER = 1, TISB_ICAO = 2, TISB_TRACKFILE = 3, VEHICLE = 4, FIXED_BEACON = 5, ADSR_OTHER = 6, RESERVED = 7, INVALID = 8 };

    // 2.2.4.5.2.5 "A/G STATE" field
    enum class AirGroundState : unsigned char { AIRBORNE_SUBSONIC = 0, AIRBORNE_SUPERSONIC = 1, ON_GROUND = 2, RESERVED = 3, INVALID = 4 };

    // 2.2.4.5.2.7.1.1 "VV Src" subfield
    enum class VerticalVelocitySource : unsigned char { GEOMETRIC = 0, BAROMETRIC = 1, INVALID = 2 };

    // 2.2.4.5.4.4 "EMERGENCY/PRIORITY STATUS" field
    enum class EmergencyPriorityStatus : unsigned char { NONE = 0, GENERAL = 1, MEDICAL = 2, MINFUEL = 3, NORDO = 4, UNLAWFUL = 5, DOWNED = 6, RESERVED = 7, INVALID = 8 };

    // 2.2.4.5.4.16 SIL Supplement Flag
    enum class SILSupplement : unsigned char { PER_HOUR = 0, PER_SAMPLE = 1, INVALID = 2 };

    // 2.2.4.5.4.12 "CAPABILITY CODES" field
    struct CapabilityCodes {
        bool uat_in : 1;
        bool es_in : 1;
        bool tcas_operational : 1;

        bool operator==(const CapabilityCodes &o) const { return (uat_in == o.uat_in && es_in == o.es_in && tcas_operational == o.tcas_operational); }

        bool operator!=(const CapabilityCodes &o) const { return !(*this == o); }
    };

    // 2.2.4.5.4.13 "OPERATIONAL MODES" field
    struct OperationalModes {
        bool tcas_ra_active : 1;
        bool ident_active : 1;
        bool atc_services : 1;

        bool operator==(const OperationalModes &o) const { return (tcas_ra_active == o.tcas_ra_active && ident_active == o.ident_active && atc_services == o.atc_services); }

        bool operator!=(const OperationalModes &o) const { return !(*this == o); }
    };

    // 2.2.4.5.6.1 "Selected Altitude Type (SAT)" field
    enum class SelectedAltitudeType : unsigned char { MCP_FCU = 0, FMS = 1, INVALID = 2 };

    // 2.2.4.5.6.5 - 2.2.4.5.6.10 Mode Bits / Mode Indicators
    struct ModeIndicators {
        bool autopilot : 1;
        bool vnav : 1;
        bool altitude_hold : 1;
        bool approach : 1;
        bool lnav : 1;

        bool operator==(const ModeIndicators &o) const { return (autopilot == o.autopilot && vnav == o.vnav && altitude_hold == o.altitude_hold && approach == o.approach && lnav == o.lnav); }

        bool operator!=(const ModeIndicators &o) const { return !(*this == o); }
    };

    typedef std::uint32_t AdsbAddress;

    struct AdsbMessage {
        AdsbMessage(const RawMessage &raw);

        // Metadata copied from the raw message
        std::uint64_t received_at;
        std::uint64_t raw_timestamp;
        unsigned errors;
        float rssi;

        // 2.2.4.5 HEADER Element
        unsigned payload_type;
        AddressQualifier address_qualifier;
        AdsbAddress address;

        // 2.2.4.5.2 STATE VECTOR Element (ADS-B)
        // 2.2.4.5.3 STATE VECTOR Element (TIS-B/ADS-B)
        boost::optional<std::pair<double, double>> position; // latitude, longitude
        boost::optional<int> pressure_altitude;
        boost::optional<int> geometric_altitude;
        boost::optional<unsigned> nic;
        boost::optional<AirGroundState> airground_state;
        boost::optional<int> north_velocity;
        boost::optional<int> east_velocity;
        boost::optional<VerticalVelocitySource> vv_src;
        boost::optional<int> vertical_velocity_barometric;
        boost::optional<int> vertical_velocity_geometric;
        boost::optional<int> ground_speed;
        boost::optional<double> magnetic_heading;
        boost::optional<double> true_heading;
        boost::optional<double> true_track;
        boost::optional<std::pair<double, double>> aircraft_size; // length, width
        boost::optional<double> gps_lateral_offset;
        boost::optional<double> gps_longitudinal_offset;
        boost::optional<bool> gps_position_offset_applied;
        boost::optional<bool> utc_coupled;         // ADS-B
        boost::optional<unsigned> uplink_feedback; // ADS-B
        boost::optional<unsigned> tisb_site_id;    // TIS-B/ADS-R

        // 2.2.4.5.4 MODE STATUS element
        boost::optional<unsigned> emitter_category;
        boost::optional<std::string> callsign;
        boost::optional<std::string> flightplan_id; // aka Mode 3/A squawk
        boost::optional<EmergencyPriorityStatus> emergency;
        boost::optional<unsigned> mops_version;
        boost::optional<unsigned> sil;
        boost::optional<unsigned> transmit_mso;
        boost::optional<unsigned> sda;
        boost::optional<unsigned> nac_p;
        boost::optional<unsigned> nac_v;
        boost::optional<unsigned> nic_baro;
        boost::optional<CapabilityCodes> capability_codes;
        boost::optional<OperationalModes> operational_modes;
        boost::optional<SILSupplement> sil_supplement;
        boost::optional<unsigned> gva;
        boost::optional<bool> single_antenna;
        boost::optional<bool> nic_supplement;

        // 2.2.4.5.5 AUXILIARY STATE VECTOR element
        // (included above)

        // 2.2.4.5.6 TARGET STATE element
        boost::optional<SelectedAltitudeType> selected_altitude_type;
        boost::optional<int> selected_altitude_mcp;
        boost::optional<int> selected_altitude_fms;
        boost::optional<double> barometric_pressure_setting;
        boost::optional<double> selected_heading;
        boost::optional<ModeIndicators> mode_indicators;

        nlohmann::json ToJson() const;

      private:
        void DecodeSV(const RawMessage &raw);
        void DecodeTS(const RawMessage &raw, unsigned startbyte);
        void DecodeMS(const RawMessage &raw);
        void DecodeAUXSV(const RawMessage &raw);
    };
  }; // namespace flightaware::uat
};   // namespace flightaware

#endif
