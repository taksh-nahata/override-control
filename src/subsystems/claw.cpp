#include "api.h"
#include "subsystems/claw.hpp"

namespace claw {
    pros::adi::DigitalOut claw_solenoid('A');
    bool isClosed = false;
    void open() { //opening the claw
        claw_solenoid.set_value(false);
        isClosed = false;
    }

    void close() { //closing the claw
        claw_solenoid.set_value(true);
        isClosed = true;
    }

    void toggle() { //simple toggle function that checks if the claw is open or closed and then does the opposite.
        if (isClosed) {
            open();
        } else {
            close();
        }
    }
}
