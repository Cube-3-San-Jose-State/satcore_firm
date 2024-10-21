#include "application.hpp"
#include "math.hpp"
#include "debug.hpp"


#include "icm20948/icm20948.hpp"
#include "neo-m9n.hpp"
#include "mpl3115/mpl3115.hpp"

#include <libhal-util/serial.hpp>
#define r_t_d 57.2957795131 


#include "mission_control.hpp"

mission_control* debug;

void application()
{
  using namespace std::chrono_literals;

  auto& led = *hardware.status_led.value();
  auto& clock = *hardware.clock.value();
  auto& console = *hardware.console.value();
  auto& i2c = *hardware.i2c;

  mission_control mc(console);
  debug = &mc;

  icm20948 imu(i2c);
  hal::delay(clock, 10ms);
  mc.log("Configuring Sensors");
  imu.set_accel_full_scale(icm20948::accel_scale::g_2);
  imu.set_gyro_full_scale(icm20948::gyro_scale::dps_250);

  imu.enable_accel_dlpf();
  imu.enable_gyro_dlpf();
  imu.set_dlpf_gyro_sample_rate(100);
  imu.enable_all();
  imu.wake_up();


  mc.log("Calibrating");
  imu.calibrate_accel_gyro(clock, 1000, 1ms);
  mc.log("Finished Calibration");

  // mpl3115 barometer(i2c);
  // hal::print<512>(console, "MPL3115 WHO AM I: %02x\n", barometer.who_am_i());
  // barometer.reset();
  // barometer.set_mode(mpl3115::measure_mode::ALTIMETER);

  math::quarternion orientation(1.0f);


  std::uint64_t i = 0;
  float dt = 0.01f;
  float data_frame_dt = 1/24.0f;
  bool k = false;
  std::uint64_t dt_ticks = static_cast<std::uint64_t>(dt * clock.frequency());
  std::uint64_t then = clock.uptime();
  std::uint64_t data_frame_ticks = static_cast<std::uint64_t>(data_frame_dt / dt);
  math::vec3 position, velocity;

  float true_dt = dt;
  while(true) {

    // Update estimation.
    math::vec3 body_acceleration, body_rates;
    imu.read(body_acceleration, body_rates);

    math::vec3 acceleration = body_acceleration;
    math::quarternion orientation_conj = math::quarternion::conjugate(orientation);
    math::quarternion acceleration_q = orientation_conj * math::quarternion(acceleration) * orientation;
    
    acceleration.x = acceleration_q.x;
    acceleration.y = acceleration_q.y;
    acceleration.z = acceleration_q.z;

    acceleration.z -= 1.0f;
    position += (velocity + 0.5 * acceleration * dt) * dt;
    velocity += acceleration * dt;

    math::vec3 br = 0.5 * dt * body_rates;
    math::quarternion rate_quart(1.0f, br.x, br.y, br.z);
    orientation = rate_quart * orientation;
    orientation.norm();


    if(data_frame_ticks <= i) {
      i = 0;
      mission_control_data_frame data;
      data.time = clock.frequency() * clock.uptime();
      data.orientation = orientation;
      data.body_angular_rates = body_rates;
      data.body_acceleration = body_acceleration;
      data.acceleration = acceleration;
      data.velocity = velocity;
      data.position = position;
      data.dt = true_dt;
      mc.send_data_frame(data);
      
      led.level(k);
      k = !k;
    }
    i++;


    std::uint64_t now = clock.uptime();
    do { now = clock.uptime(); }while((now - then) < dt_ticks); // Spinlock until dt seconds have passed;
    then = clock.uptime();
    true_dt = (now - then) * clock.frequency();
  }

  // neo_m9n gps(console);
  // gps.update();

  // hal::print(console, "I2C\n");
  // hal::print(console, "Will reset after ~10 seconds\n");

  // for (int i = 0; i < 10; i++) {
  //   // Print message
  //   hal::print(console, "Hello, World\n");

  //   // Toggle LED
  //   led.level(true);
  //   hal::delay(clock, 500ms);

  //   led.level(false);
  //   hal::delay(clock, 500ms);
  // }

  // hal::print(console, "Resetting!\n");
  // hal::delay(clock, 100ms);

  // hardware.reset();
}
