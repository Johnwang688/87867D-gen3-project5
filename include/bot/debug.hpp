#pragma once

#include "bot/bot.hpp"
#include "bot/location.hpp"
#include "../auton.hpp"

namespace debug {
    void print_location();
    void print_sensor_data();
    void print_timestamp();
    void print_all();
    int location_debug_task_fn();
}