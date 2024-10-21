#include "configure.hpp"
#include "application.hpp"


inline void configure_imu() {
    auto& imu = *io.imu;
    imu.set_accel_full_scale(icm20948::accel_scale::g_2);
    imu.set_gyro_full_scale(icm20948::gyro_scale::dps_250);

    imu.enable_accel_dlpf();
    imu.enable_gyro_dlpf();
    imu.set_dlpf_gyro_sample_rate(100);
    imu.enable_all();
}

inline void configure_gps() {

}

inline void configure_barometer() {

}


void configure() {
    auto& mc = *io.mc;
    mc.info("Configuring Sensors");
    // mc.info("Configuring IMU");
    configure_imu();
    // mc.info("Configured IMU");
    // mc.info("Configuring GPS");
    configure_gps();
    // mc.info("Configured GPS");
    // mc.info("Configuring Barometer");
    configure_barometer();
    // mc.info("Configured Barometer");
    mc.info("Finished Configuring Sensors.");
}