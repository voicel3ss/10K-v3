#include "main.h"

ez::Drive chassis(
    {-6, 8, -5},  // Left motors
    {9, -10, 7},  // Right motors (negative for reversed)
    21,           // IMU port
    2.75,
    600);

ez::tracking_wheel horiz_tracker(
    1,     // Port
    2,     // Wheel Diameter
    4.0);  // Distance to center of robot

pros::Motor intake(2);
pros::Motor lever(20);
pros::Rotation lever_rotation(15);

ez::Piston blocker('D');
ez::Piston lift('G');
ez::Piston matchloader('H');
ez::Piston wing('C');

bool drive_arcade = false;
bool intake_toggle = false;
bool reverse_toggle = false;
bool lift_toggle = false;
bool wing_toggle = false;

void score(){
  if (!lift_toggle){
    intake.move(127);
    lever.move_absolute(650, 600);
    blocker.set(true);
    pros::delay(800);
    intake.move(0);
    lever.move_absolute(0, 600);
    pros::delay(400);
  } else {
    intake.move(127);
    lever.move_absolute(750, 100);
    blocker.set(true);
    pros::delay(800);
    intake.move(0);
    lever.move_absolute(0, 600);
    pros::delay(400);
  }
}

void controls() {
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
    } else if (r1) {
      wing_toggle = true;
    } else if (!r1) {
      wing_toggle = false;
    }

    if (master.get_digital_new_press(DIGITAL_UP)) {
      drive_arcade = !drive_arcade;
      master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
      master.rumble(drive_arcade ? "." : "..");
    }

    if (master.get_digital(DIGITAL_L2)) {
      intake_toggle = false;
      reverse_toggle = true;
    } else {
      reverse_toggle = false;
    }

    if (master.get_digital_new_press(DIGITAL_R2)) {
      pros::Task score_task(score);
    }
    pros::delay(ez::util::DELAY_TIME);
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
      {"Drive\n\nDrive forward and come back", drive_example},
      {"Turn\n\nTurn 3 times.", turn_example},
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

  lever_rotation.reset_position();

  master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
  pros::Task controlTask(controls);
  // ez::as::initialize();
  pros::lcd::initialize();

  pros::Task screen_task([&]() {
    while (true) {
      pros::lcd::print(0, "x: %f", chassis.odom_x_get());
      pros::lcd::print(1, "y: %f", chassis.odom_y_get());
      pros::lcd::print(2, "theta: %f", chassis.odom_theta_get());

      pros::lcd::print(4, "lever: %d", lever_rotation.get_position());
      pros::lcd::print(5, "lever velocity: %d", lever_rotation.get_velocity());
      pros::delay(20);
    }
  });

  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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

    if (reverse_toggle) {
      intake.move(-127);
    }

    if (intake_toggle && !reverse_toggle) {
      intake.move(127);
    } else if (!reverse_toggle) {
      intake.move(0);
    }

    matchloader.button_toggle(DIGITAL_DOWN);
    lift.set(lift_toggle);
    wing.set(wing_toggle);
    pros::delay(ez::util::DELAY_TIME);
  }
}
