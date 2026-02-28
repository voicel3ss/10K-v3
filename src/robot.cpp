#include "robot.hpp"

Robot::Robot(int firstStagePort,
             int leverPort,
             int rotationPort,
             char blockerPort,
             char liftPort,
             char matchloaderPort,
             char wingPort)
    : first_stage(firstStagePort),
      lever(leverPort),
      lever_rotation(rotationPort),
      blocker(blockerPort),
      lift(liftPort),
      matchloader(matchloaderPort),
      wing(wingPort),
      leverTask(Robot::lever_task_trampoline, this) {

  lift.set_value(piston_up_value);
  lift_up = true;

  blocker.set_value(blocker_closed_value);

  matchloader.set_value(piston_down_value);
  wing.set_value(piston_down_value);
  matchloader_up = false;
  wing_up = false;
}

void Robot::init() {
  lift.set_value(piston_up_value);
  lift_up = true;

  blocker.set_value(blocker_closed_value);

  matchloader.set_value(piston_down_value);
  matchloader_up = false;

  wing.set_value(piston_down_value);
  wing_up = false;

  enforceWingRule();

  lever.set_brake_mode(MOTOR_BRAKE_HOLD);
  lever.move(0);

  first_stage.move(0);
  intake_running = false;

  lever_state = LeverState::IDLE;
  lever_requested = false;
}

void Robot::enforceWingRule() {
  if (!lift_up) {
    wing.set_value(piston_down_value);
    wing_up = false;
  }
}

// -------- Intake --------

void Robot::intakeOn() { first_stage.move(127); intake_running = true; }
void Robot::intakeOff() { first_stage.move(0); intake_running = false; }
void Robot::toggleIntake() { intake_running = !intake_running; first_stage.move(intake_running ? 127 : 0); }
void Robot::reverseIntake() { first_stage.move(-127); }

// -------- Lift --------

void Robot::raise() { lift.set_value(piston_up_value); lift_up = true; }
void Robot::lower() { lift.set_value(piston_down_value); lift_up = false; enforceWingRule(); }
void Robot::toggleLift() { lift_up ? lower() : raise(); }

// -------- Matchloader --------

void Robot::matchloaderUp() { matchloader.set_value(piston_up_value); matchloader_up = true; }
void Robot::matchloaderDown() { matchloader.set_value(piston_down_value); matchloader_up = false; }
void Robot::toggleMatchloader() { matchloader_up ? matchloaderDown() : matchloaderUp(); }

// -------- Wing --------

void Robot::wingUp() {
  enforceWingRule();
  if (!lift_up) return;
  wing.set_value(piston_up_value);
  wing_up = true;
}

void Robot::wingDown() {
  wing.set_value(piston_down_value);
  wing_up = false;
}

void Robot::toggleWing() {
  enforceWingRule();
  if (!lift_up) return;
  wing_up ? wingDown() : wingUp();
}

// -------- Lever Move (NO PID) --------

void Robot::moveLeverTo(double target_deg, int dir, int power) {
  if (lever_state != LeverState::IDLE) return;

  lever_target_deg = target_deg;
  lever_dir = (dir >= 0) ? 1 : -1;
  lever_power = std::clamp(std::abs(power), 0, 127);

  lever_requested = true;
}

void Robot::lever_task_trampoline(void* param) {
  static_cast<Robot*>(param)->lever_task();
}

void Robot::lever_task() {
  pros::delay(2000);

  while (true) {
    if (lever_requested && lever_state == LeverState::IDLE) {
      lever_requested = false;
      lever_state = LeverState::MOVE_TO_TARGET;
    }

    switch (lever_state) {
      case LeverState::IDLE:
        break;

      case LeverState::MOVE_TO_TARGET: {
        double current = lever_deg();
        double err = lever_target_deg - current;

        if (std::abs(err) <= lever_window) {
          lever.move(0);
          lever_state = LeverState::IDLE;
          break;
        }

        lever.move(lever_power * lever_dir);
      } break;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
