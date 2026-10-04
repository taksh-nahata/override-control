#pragma once

namespace lift {
    pros::Motor liftMotor1(1, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees); //just setting up the two lift motors.
    pros::Motor liftMotor2(-5, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees); //(port, )
    pros::Rotation liftRot(-8);

    void move(double voltage) { //basic move function for the lift, just moves both motors at the same time at the voltage specified.
        liftMotor1.move(voltage);
        liftMotor2.move(voltage);
    }

    void initialize() {
        liftMotor1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        liftMotor2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }
}