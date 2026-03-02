#include "bot/debug.hpp"
#include "bot/location.hpp"

namespace debug {
    void print_location() {
        Pose pose = bot::mcl::location.get_pose();
        printf("Location: %.1f, %.1f Heading: %.1f\n", pose.x, pose.y, pose.heading);
    }
    void print_sensor_data() {
        printf("Sensors: Left: %.1f, Right: %.1f, Back: %.1f\n", bot::sensors::left_dist.objectDistance(vex::mm), bot::sensors::right_dist.objectDistance(vex::mm), bot::sensors::back_dist.objectDistance(vex::mm));
    }
    void print_timestamp() {
        printf("Time: %.1f\n", bot::Brain.Timer.time(vex::msec));
    }

    void print_all() {
        Pose pose = bot::mcl::location.get_pose();
        printf("Loc: %.1f, %.1f H: %.1f | L: %.1f R: %.1f B: %.1f | T: %.0f\n",
             pose.x, pose.y, pose.heading,
             bot::sensors::left_dist.objectDistance(vex::mm),
             bot::sensors::right_dist.objectDistance(vex::mm),
             bot::sensors::back_dist.objectDistance(vex::mm),
             bot::Brain.Timer.time(vex::msec));
    }

    int location_debug_task_fn() {
        while (bot::mcl::location.is_running()) {
            print_all();
            vex::task::sleep(100);
        }
        return 0;
    }
}