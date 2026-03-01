#pragma once

#include "bot/bot.hpp"
#include "bot/drivetrain.hpp"
#include "bot/location.hpp"

namespace bot {

    namespace drivetrains {
        extern bot::Drivetrain dt;
    }

    namespace mcl {
        extern bot::Location location;
    }

    namespace autons {
        void left_7();
        void left_4_3();
        void left_6_3();
        void left_6();
        void right_7();
        void right_6();
        void right_4_3();
        void left_4();
        void right_4();
        void sawp();
        void counter_sawp();
        void skills();
        void test();
        void no_auton();
    }

}