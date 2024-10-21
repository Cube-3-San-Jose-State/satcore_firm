#pragma once
// #include <libhal-exceptions/control.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

#include "resource_list.hpp"


extern resource_list hardware;
extern io_list io;

void application();
