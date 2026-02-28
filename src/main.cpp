#include "main.h"

#include "robot.hpp"

ez::Drive chassis(
    {-6, 8, -5},  // Left motors
    {9, -10, 7},  // Right motors (negative for reversed)
    21,           // IMU port
    2.75,
    600);

ez::tracking_wheel horiz_tracker(
    1,     // Port
    2,     // Wheel Diameter. 
    4.0);  // Distance to center of robot

Robot robot(
    2,    // firstStagePort
    20,   // leverPort
    15,   // rotationPort
    'A',  // blockerPort
    'B',  // liftPort
    'C',  // matchloaderPort
    'D'   // wingPort
);
bool lever_up = false;
bool drive_arcade = false;
bool was_outtaking = false;
bool isDown = false;
void drive_mode_task() {
  while (true) {
    if (master.get_digital_new_press(DIGITAL_UP)) {
      drive_arcade = !drive_arcade;
      master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
      master.rumble(drive_arcade ? "." : "..");
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

void initialize() {
  ez::ez_template_print();

  pros::delay(500);
  chassis.odom_tracker_back_set(&horiz_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  default_constants();

  ez::as::auton_selector.autons_add({
  });

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  robot.init();

  pros::Task driveModeTask(drive_mode_task);
  master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
}

void disabled() {

}

void competition_initialize() {

}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  ez::as::auton_selector.selected_auton_call();
}

void screen_print_tracker(ez::tracking_wheel* tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine when B and left are pressed.
    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_LEFT)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    chassis.pid_tuner_iterate();
  }

  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();

    if (drive_arcade)
      chassis.opcontrol_arcade_standard(ez::SPLIT);
    else
      chassis.opcontrol_tank();

    // Toggle lift
    if (master.get_digital(DIGITAL_R1) &&
        master.get_digital_new_press(DIGITAL_L1))
      robot.toggleLift();

    // Toggle intake
    else if (master.get_digital_new_press(DIGITAL_L1))
      robot.toggleIntake();

    // Reverse intake
    if (master.get_digital(DIGITAL_L2)) {
      robot.reverseIntake();
      was_outtaking = true;
    } else if (was_outtaking) {
      robot.intakeOff();  // stop when L2 is released
      was_outtaking = false;
    }

    // Score
// Lever toggle
  if (master.get_digital_new_press(DIGITAL_R2)) {
    int start_time = pros::millis(); 
      robot.moveLeverTo(105, +1, 127);  
      
      pros::delay(400);  
      robot.moveLeverTo(0, -1, 60);    
    
    robot.moveLeverTo(0, -1, 60);  // Ensure lever returns to starting position after timeout
    // Matchloader
    if (master.get_digital_new_press(DIGITAL_DOWN))
      robot.toggleMatchloader();

    // Wing hold
    if (master.get_digital(DIGITAL_R1) && !master.get_digital_new_press(DIGITAL_L1))
      robot.wingUp();
    else
      robot.wingDown();

    pros::delay(ez::util::DELAY_TIME);
  }
}
}
