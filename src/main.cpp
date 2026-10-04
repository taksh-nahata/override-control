#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // Drive motor ports (if it is negative then that means that the motor is reversed)
    {-19, -11},     // Left Chassis Ports
    {16, 15},  // Right Chassis Ports
    10,      // IMU Port
    2.75,  // Wheel Diameter
    600,
    48.0/36.0
);   // Wheel RPM = cartridge * (motor gear / wheel gear)

ez::tracking_wheel horiz_tracker(-13, 2, -1.97);  // This tracking wheel is perpendicular to the drive wheels. The last value (-1.97) is the horizontal offset because no odometry wheel is perfectly centered. We tested and found that our wheel was offset from the center of the robot by  1.97 inches so we set it to that. The 2 is the wheel diameter.

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // sets the odom to the back of the robot, which is where the tracking wheel is located.  This is important for odometry to work correctly.
  chassis.odom_tracker_back_set(&horiz_tracker);


  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();
  
  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Cup+Goal\n\nTake a pin and then put into the goal and then grab a cup and then put that onto the goal as well. Scores: 5 points.", drive_example}
      // still have to add more routines
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + ez::util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + ez::util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_motor_temperatures(){ // A basic display for chassis motors.
  if(chassis.left_motors.size() * 2 < 12){
    for(int i = 0; i < chassis.left_motors.size(); i++){
      ez::screen_print("Motor " + std::to_string(chassis.left_motors[i].get_port()) + ": " +
      std::to_string(static_cast<int>(chassis.left_motors[i].get_temperature())) + " *C*\t" + "Motor " + 
      std::to_string(chassis.right_motors[i].get_port()) + ": " + 
      std::to_string(static_cast<int>(chassis.right_motors[i].get_temperature())) +" *C", 
      i); //i iterates the line number
    }
  }
}
  /**                   Temperature key
*  Level 1 - Temp greater than 55 deg C or 131 deg F - 50% Power
*  Level 2 - Temp greater than 60 deg C or 140 deg F - 25% Power - Less than 25% power is useless on a comp robot
*  Level 3 - Temp greater than 65 deg C or 149 deg F - 12.5% Power
*  Level 4 - Temp greater than 70 deg C or 158 deg F - 0% Power
*/

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + ez::util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + ez::util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + ez::util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
        else if(ez::as::page_blank_is_on(1)) ez_motor_temperatures();
        
        // . . .
        // Add your own blank pages here!
        // . . .
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) claw::toggle();  // Toggle claw with L1

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
