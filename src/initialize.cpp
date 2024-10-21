#include "resource_list.hpp"


io_list initialize_sensors(resource_list& p_list) {
    using namespace std::chrono_literals;
    auto& console = *p_list.console.value();
    auto& clock = *p_list.clock.value();
    static mission_control mc(console);
    
    mc.log("Initializing sensors.");
    hal::delay(clock, 10ms);
    io_list out;

    auto& gps_serial = *p_list.gps_serial.value();
    auto& i2c = *p_list.i2c;

    mc.info("Initializing GPS");
    static neo_m9n gps(gps_serial); 

    mc.info("Initializing IMU");
    static icm20948 imu(i2c);
    mc.info("Initializing Barometer");
    static mpl3115 baro(i2c);


    out.resources = &p_list;
    out.gps = &gps;
    out.imu = &imu;
    out.baro = &baro;
    out.mc = &mc;
    mc.log("Finished initializing sensors.");

    return out;
}