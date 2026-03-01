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
    2,     // Wheel Diameter
    4.0);  // Distance to center of robot

Robot robot(
    2,    // firstStagePort
    20,   // leverPort
    15,   // rotationPort
    'D',  // blockerPort
    'G',  // liftPort
    'H',  // matchloaderPort
    'C'   // wingPort
);

static constexpr int R1L1_TOLERANCE_MS = 250;

bool drive_arcade = false;
bool was_outtaking = false;
bool intake_was_on_before_outtake = false;

uint32_t last_r1_press_ms = 0;
uint32_t last_l1_press_ms = 0;
bool r1l1_combo_latched = false;

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
  // ez::ez_template_print();

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

  robot.init();

  master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
  pros::Task driveModeTask(drive_mode_task);
  // ez::as::initialize();
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

void print_task() {
  while (true) {
    pros::lcd::set_text(0, "Lever Rotation: " + util::to_string_with_precision(robot.getLeverRotation()));
    pros::delay(10);
  }
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
          ez::screen_print("rotation: " + util::to_string_with_precision(robot.getLeverRotation()), 2);
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
// pros::Task ezScreenTask(ez_screen_task);
pros::Task printTask(print_task);
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

    uint32_t now = pros::millis();

    // record press times
    if (master.get_digital_new_press(DIGITAL_R1)) last_r1_press_ms = now;
    if (master.get_digital_new_press(DIGITAL_L1)) last_l1_press_ms = now;

    // combo if pressed within tolerance, and still "recent"
    uint32_t newest = (last_r1_press_ms > last_l1_press_ms) ? last_r1_press_ms : last_l1_press_ms;
    uint32_t diff = (last_r1_press_ms > last_l1_press_ms) ? (last_r1_press_ms - last_l1_press_ms)
                                                          : (last_l1_press_ms - last_r1_press_ms);

    bool r1Held = master.get_digital(DIGITAL_R1);
    bool l1Held = master.get_digital(DIGITAL_L1);

    bool combo_now = r1Held && l1Held && (diff <= (uint32_t)R1L1_TOLERANCE_MS) &&
                     ((now - newest) <= (uint32_t)R1L1_TOLERANCE_MS);

    // Fire combo once (NO DELAY)
    if (combo_now && !r1l1_combo_latched) {
      robot.toggleLift();
      r1l1_combo_latched = true;
    }

    // L1 alone toggles intake, but NOT during combo and NOT during L2 outtake override
    if (master.get_digital_new_press(DIGITAL_L1) && !combo_now && !master.get_digital(DIGITAL_L2)) {
      robot.toggleIntake();
    }

    // reset latch when both released
    if (!r1Held && !l1Held) {
      r1l1_combo_latched = false;
    }
    // L1 alone toggles intake (only if not combo)

    // Reverse intake
    // Reverse intake (temporary override)
    if (master.get_digital_new_press(DIGITAL_L2)) {
      intake_was_on_before_outtake = robot.isIntakeRunning();  // or add getter
      robot.reverseIntake();
      was_outtaking = true;
    }

    if (master.get_digital(DIGITAL_L2)) {
      // keep reversing while held
      robot.reverseIntake();
      was_outtaking = true;
    } else if (was_outtaking) {
      // restore what intake was doing before L2
      if (intake_was_on_before_outtake)
        robot.intakeOn();
      else
        robot.intakeOff();
      was_outtaking = false;
    }

    // Score
    if (master.get_digital_new_press(DIGITAL_R2))
      robot.score();

    // Matchloader
    if (master.get_digital_new_press(DIGITAL_DOWN))
      robot.toggleMatchloader();

    // Wing hold
    if (!combo_now && !r1l1_combo_latched) {
      if (master.get_digital(DIGITAL_R1))
        robot.wingUp();
      else
        robot.wingDown();
    } else {
      // optional: force wing down during combo
      robot.wingDown();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
