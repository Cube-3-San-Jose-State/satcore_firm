
#pragma once
// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <optional>

#include <libhal/functional.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>
#include <libhal/i2c.hpp>

#include "mission_control.hpp"
#include "icm20948/icm20948.hpp"
#include "mpl3115/mpl3115.hpp"
#include "neo-m9n.hpp"
struct resource_list
{
    hal::callback<void()> reset;
    std::optional<hal::output_pin*> status_led;
    std::optional<hal::serial*> console;
    std::optional<hal::steady_clock*> clock;

    std::optional<hal::serial*> gps_serial;


    hal::i2c* i2c;
};

struct io_list {
    resource_list* resources;
    mission_control* mc;
    icm20948* imu; 
    mpl3115* baro;
    neo_m9n* gps;
};

resource_list initialize_platform();

io_list initialize_sensors(resource_list& p_list);

// resource_list resources;