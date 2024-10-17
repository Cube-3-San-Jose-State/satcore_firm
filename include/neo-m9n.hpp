#pragma once

#include <libhal/serial.hpp>
#include <array>
#include "geo_coord.hpp"
#include <string_view>
#include <cstdio>

#include <libhal-util/serial.hpp>

class neo_m9n {
public:
    neo_m9n(hal::serial& p_serial) : m_serial(p_serial) {

    };

    /**
     * @brief Parse the latest data received by the GPS.
     * 
     * @return True if a NMEA Message was fully parsed. False otherwise.
     */
    inline bool update() {
        std::array<hal::byte, 512> serial_msg;
        auto result = m_serial.read(serial_msg);
        return parse(result.data);
    }

    /**
     * @brief Empty the internal buffer. 
     */
    inline void flush() {
        _buffer_pos = _internal_buffer.begin();
    }

    /**
     * @brief Returns true when the GPS has a position fix. False otherwise.
     */
    inline bool is_fixed() {
        return _last_gga_update.is_locked;
    }

    /**
     * @brief Returns the last GPS location recieved by the reciever.
     * @warning Valid only when the GPS is fixed.
     */
    inline geo_coord coord() {
        return _last_gga_update.coord;
    }

    private:


    /**
     * @brief Parse NMEA Messages contained from a Serial Stream.
     * 
     * @param data Data from a serial stream.
     * @return True if a NMEA Message was fully parsed. False otherwise.
     */
    bool parse(std::span<const hal::byte> data) {
        bool data_updated = false;

        for(auto i = data.begin(); i != data.end(); i ++) {
            if(_buffer_pos == _internal_buffer.begin()) {
                if(*i == '$') {
                    *_buffer_pos = *i;
                    _buffer_pos++;
                }
            }else{
                *_buffer_pos = *i;
                _buffer_pos++;
                
                if(*(_buffer_pos-1) == '\n' && *(_buffer_pos-2) == '\r') {
                    data_updated = true;
                    process_buffered_message();
                }
            }
        }

        return data_updated;
    }

    /**
     * @brief Process the message stored in the internal buffer. 
     * Message is required to a complete NMEA Message.
     * 
     */
    inline void process_buffered_message() {
        (*(_buffer_pos-2)) = '\0';
        _buffer_pos = _internal_buffer.begin();
        auto x = std::span(_internal_buffer).subspan(3, 3);
        if(x.size() == 3 && x[0] == 'G' && x[1] == 'G' && x[2] == 'A') process_GGA();
    }

    /**
     * @brief Process the GGA message stored in the internal buffer.
     * Message has to be GGA.
     * 
     */
    inline void process_GGA() {
        char talker_id[3];

        int lat_degrees = 0;
        float lat_minutes = 0;
        char lat_direction = '?';
        int lng_degrees = 0;
        float lng_minutes;
        char lng_direction = '?';

        float unused;
        
        // https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
        // $GPGGA,172814.0,3723.46587704,N,12202.26957864,W,2,6,1.2,18.893,M,-25.669,M,2.0 0031*4F
        std::sscanf(_internal_buffer.data(), "$%2sGGA,%f,%2d%f,%c,%3d%f,%c,%d,%d,%f,%f,%c,%f,%c,%f,%9s",
            talker_id,
            &_last_gga_update.utc_time,
            &lat_degrees,
            &lat_minutes,
            &lat_direction,
            &lng_degrees,
            &lng_minutes,
            &lng_direction,
            &_last_gga_update.fix_status,
            &_last_gga_update.satellites_used,
            &_last_gga_update.hdop,
            &_last_gga_update.altitude,
            &_last_gga_update.altitude_units,
            &_last_gga_update.height_of_geoid,
            &_last_gga_update.height_of_geoid_units,
            &unused,
            _last_gga_update.dgps_station_id_checksum
        );

        _last_gga_update.coord.lat = 
            (lat_direction == 'S' ? -1 : 1) * 
            (static_cast<float>(lat_degrees) + lat_minutes/60.0f);
        _last_gga_update.coord.lng = 
            (lng_direction == 'W' ? -1 : 1) * 
            (static_cast<float>(lng_degrees) + lng_minutes/60.0f);
        _last_gga_update.coord.alt = _last_gga_update.altitude;

        _last_gga_update.is_locked = _last_gga_update.fix_status != -0;
    }


    /**
     * @brief Data recieved on a successful NMEA-GGA Message.
     * 
     */
    struct gps_parsed
    {
        bool is_locked = false;
        float utc_time;
        geo_coord coord;
        int fix_status;
        int satellites_used;
        float hdop;
        float altitude;
        char altitude_units;
        float height_of_geoid;
        char height_of_geoid_units;
        char time_since_last_dgps_update;
        char dgps_station_id_checksum[10];
    } _last_gga_update;


    std::array<char, 512> _internal_buffer;
    std::array<char, 512>::iterator _buffer_pos = _internal_buffer.begin();


private:
    hal::serial& m_serial;
    std::array<char, 512> m_msg_buffer;
    struct {
        bool is_fixed = false;
    } m_last_data;
};