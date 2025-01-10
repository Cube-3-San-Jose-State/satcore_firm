#pragma once

#include <libhal-util/bit.hpp>
#include <libhal/units.hpp>
/*
  TO DO
This is just a template. Double-check register address to code. Need the module 
part number.

*/ 
namespace xbee
{
    static constexpr hal::byte STATUS          = 0x00;  
    static constexpr hal::byte TX_REQUEST      = 0x01;  
    static constexpr hal::byte RX_INDICATOR    = 0x02;  
    static constexpr hal::byte AT_COMMAND      = 0x03;  
    //static constexpr hal::byte MODEM_STATUS   = 0x04;  
    static constexpr hal::byte TX_STATUS       = 0x05;  
    static constexpr hal::byte ROUTE_RECORD    = 0x06;  
    static constexpr hal::byte TX_PASS         = 0x07;  
    static constexpr hal::byte DEFAULT_ADDRESS = 0x0C;
    static constexpr hal::byte START_DELIMITER = 0x7E;
    static constexpr size_t MAX_FRAME_SIZE     = 256;
    static constexpr size_t CHECKSUM_SIZE      = 1;
    static constexpr size_t LENGTH_SIZE        = 2;
};
