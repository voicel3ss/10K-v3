#include "main.h"

ez::Drive chassis(
    {-6, 8, -5},  // Left motors
    {9, -10, 7},  // Right motors (negative for reversed)
    21,           // IMU port
    2.75,
    600);

pros::MotorGroup left_motors({-6, 8, -5});
pros::MotorGroup right_motors({9, -10, 7});

ez::tracking_wheel horiz_tracker(
    -3,     // Port
    2,      // Wheel Diameter
    4.45);  // Distance to center of robot

pros::Motor intake(2);
pros::Motor lever(20);

ez::Piston blocker('D');
ez::Piston lift('G');
ez::Piston matchloader('H');
ez::Piston wing('C');

bool drive_arcade = false;
bool intake_toggle = false;
bool reverse_toggle = false;
bool lift_toggle = false;
bool wing_toggle = false;
bool matchloader_toggle = false;
bool start_down = false;
bool score_intake_toggle = false;

void score() {
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle) {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move(127);
    blocker.set(true);
    pros::delay(800);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(700);
    lever.move_velocity(0);
  } else {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move_velocity(70);
    blocker.set(true);
    pros::delay(1200);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void score_driver() {
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle) {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move(127);
    blocker.set(true);
    pros::delay(800);
    while (master.get_digital(DIGITAL_R2)) {
      pros::delay(ez::util::DELAY_TIME);
    }
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  } else {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move_velocity(100);
    blocker.set(true);
    pros::delay(1000);
    while (master.get_digital(DIGITAL_R2)) {
      pros::delay(ez::util::DELAY_TIME);
    }
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(600);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void score_three() {
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle) {
    intake.move(0);
    score_intake_toggle = true;
    // lever.move(127);
    lever.move_relative(500, 127);
    blocker.set(true);
    pros::delay(700);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(450);
    lever.move_velocity(0);
  } else {
    intake.move(0);
    score_intake_toggle = true;
    lever.move_velocity(70);
    blocker.set(true);
    pros::delay(300);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void controls() {
  lever.move(-127);
  pros::delay(800);
  lever.set_zero_position(0);
  lever.move(0);
  while (true) {
    bool r1_new = master.get_digital_new_press(DIGITAL_R1);
    bool l1_new = master.get_digital_new_press(DIGITAL_L1);
    bool r1 = master.get_digital(DIGITAL_R1);
    bool l1 = master.get_digital(DIGITAL_L1);

    if ((r1_new && l1) || (l1_new && r1)) {
      lift_toggle = !lift_toggle;
      blocker.set(false);
      pros::delay(250);
    } else if (l1_new) {
      intake_toggle = !intake_toggle;
      blocker.set(false);
    } else if (lift_toggle) {
      wing_toggle = false;
    } else if (r1) {
      wing_toggle = false;
      start_down = false;
    } else if (start_down) {
      wing_toggle = false;
    } else if (!r1 && !start_down) {
      wing_toggle = true;
    }

    if (master.get_digital_new_press(DIGITAL_X)) {
      lever.move(-127);
      pros::delay(800);
      lever.set_zero_position(0);
      lever.move(0);
    }

    if (master.get_digital_new_press(DIGITAL_UP)) {
      drive_arcade = !drive_arcade;
      master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
      master.rumble(drive_arcade ? "." : "..");
    }

    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      matchloader_toggle = !matchloader_toggle;
    }

    if (master.get_digital(DIGITAL_L2)) {
      intake_toggle = false;
      reverse_toggle = true;
    } else {
      reverse_toggle = false;
    }

    if (master.get_digital_new_press(DIGITAL_R2)) {
      pros::Task score_task(score_driver);
    }

    if (master.get_digital_new_press(DIGITAL_B)) {
      pros::Task score_task(score_three);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

void controller_update() {
  while (true) {
    master.clear();
    master.print(0, 0, "Battery%: %d%%", master.get_battery_capacity());
    master.print(1, 0, "Intake: %dC", intake.get_temperature());
    master.print(2, 0, "Lever: %dC", lever.get_temperature());
    master.print(3, 0, "DriveL: %dC", left_motors.get_temperature());
    master.print(4, 0, "DriveR: %dC", right_motors.get_temperature());
    pros::delay(1000);
  }
}

void initialize() {
  ez::ez_template_print();
  pros::delay(500);
  default_constants();
  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker

  ez::as::auton_selector.autons_add({
      {"SAWP", sawp},
      // {"Nine Ball Right Wing", nine_ball_right_wing},
      // {"Six Ball Right Wing", six_ball_right_wing},
      // {"Six Ball Right Score", six_ball_right_score},
      {"Skills", skills},
  });

  chassis.odom_tracker_back_set(&horiz_tracker);

  chassis.opcontrol_curve_buttons_toggle(true);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);
  chassis.initialize();

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
  ez::as::initialize();
  pros::lcd::initialize();

  // pros::Task screen_task([&]() {
  //   while (true) {
  //     pros::lcd::print(0, "x: %f", chassis.odom_x_get());
  //     pros::lcd::print(1, "y: %f", chassis.odom_y_get());
  //     pros::lcd::print(2, "theta: %f", chassis.odom_theta_get());

  //     pros::lcd::print(4, "rotation: %f", horiz_tracker.get());
  //     pros::lcd::print(5, "distance to center: %f", horiz_tracker.distance_to_center_get());
  //     pros::delay(20);
  //   }
  // });

  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  // pros::Task controller_task(controller_update);
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.drive_brake_set(MOTOR_BRAKE_BRAKE);
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

void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    // if (master.get_digital_new_press(DIGITAL_X))
    //   chassis.pid_tuner_toggle();

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
  pros::Task controlTask(controls);

  while (true) {
    ez_template_extras();

    if (drive_arcade)
      chassis.opcontrol_arcade_standard(ez::SPLIT);
    else
      chassis.opcontrol_tank();

    if (master.get_digital(DIGITAL_Y)) {
      intake.move(-127);
    } else {
      if (reverse_toggle) {
        intake.move(-55);
      }

      if (score_intake_toggle) {
        intake.move(-50);
      } else if (intake_toggle && !reverse_toggle) {
        intake.move(127);
      } else if (!reverse_toggle) {
        intake.move(0);
      }
    }
    matchloader.set(matchloader_toggle);
    lift.set(lift_toggle);
    wing.set(wing_toggle);
    pros::delay(ez::util::DELAY_TIME);
  }
}
