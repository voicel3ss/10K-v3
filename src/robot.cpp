// ===================== robot.cpp =====================
#include "robot.hpp"

#include <cmath>

static inline int clamp127(int v) {
  if (v > 127) return 127;
  if (v < -127) return -127;
  return v;
}

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
      scoreTask(Robot::score_task_trampoline, this) {
  // Initial piston states
  // Lift requirement from earlier: starts UP
  lift.set_value(piston_up_value);
  lift_up = true;

  // Blocker (stopper): closed by default
  blocker.set_value(blocker_closed_value);

  // New pistons: both default DOWN
  matchloader.set_value(piston_down_value);
  wing.set_value(piston_down_value);
  matchloader_up = false;
  wing_up = false;

  // PID exit conditions (tune later)
  // Format: (small_time, small_error, big_time, big_error, vel_time, current_time)
  leverPID.exit_condition_set(120, 2.0, 350, 6.0, 250, 400);
}

double Robot::lever_deg() const {
  // PROS Rotation returns centidegrees
  return static_cast<double>(lever_rotation.get_position()) / 100.0;
}

void Robot::move_lever(int power) {
  lever.move(clamp127(power));
}

void Robot::enforceWingRule() {
  // Wing is uncontrollable when lift is lowered: force down
  if (!lift_up) {
    wing.set_value(piston_down_value);
    wing_up = false;
  }
}

void Robot::init() {
  // Default piston states
  // both pistons default down (per your latest requirement)
  lift.set_value(piston_up_value);
  lift_up = true;

  blocker.set_value(blocker_closed_value);

  matchloader.set_value(piston_down_value);
  matchloader_up = false;

  wing.set_value(piston_down_value);
  wing_up = false;

  // Make sure wing rule is enforced when lift is lowered
  enforceWingRule();

  // Safety: stop motors at init
  first_stage.move(0);
  lever.move(0);
  intake_running = false;

  lever_rotation.reset_position();
}

// -------- First stage --------

void Robot::intakeOn() {
  intake_running = true;
  first_stage.move(127);
}

void Robot::intakeOff() {
  intake_running = false;
  first_stage.move(0);
}

void Robot::toggleIntake() {
  intake_running = !intake_running;
  first_stage.move(intake_running ? 127 : 0);
}

void Robot::reverseIntake() {
  first_stage.move(-127);
}

// -------- Lift piston --------

void Robot::raise() {
  lift.set_value(piston_up_value);
  lift_up = true;
  // Now wing is allowed again (but stays whatever it currently is).
}

void Robot::lower() {
  lift.set_value(piston_down_value);
  lift_up = false;

  // Force wing down when lift is lowered
  enforceWingRule();
}

void Robot::toggleLift() {
  if (lift_up)
    lower();
  else
    raise();
}

// -------- Matchloader piston --------

void Robot::matchloaderUp() {
  matchloader.set_value(piston_up_value);
  matchloader_up = true;
}

void Robot::matchloaderDown() {
  matchloader.set_value(piston_down_value);
  matchloader_up = false;
}

void Robot::toggleMatchloader() {
  if (matchloader_up)
    matchloaderDown();
  else
    matchloaderUp();
}

// -------- Wing piston (restricted by lift) --------

void Robot::wingUp() {
  // If lift is lowered, wing cannot go up
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
  // If lift is lowered, wing is forced down and cannot toggle up
  enforceWingRule();
  if (!lift_up) return;

  if (wing_up)
    wingDown();
  else
    wingUp();
}

// -------- Lever score sequence (async) --------

void Robot::score() {
  // Only accept a new request if we're idle
  if (score_state != ScoreState::IDLE) return;

  // Choose speed cap based on current lift position
  active_lever_speed = lift_up ? lever_full_speed : lever_slow_speed;

  score_requested = true;
}

void Robot::score_task_trampoline(void* param) {
  static_cast<Robot*>(param)->score_task();
}

void Robot::score_task() {
  while (true) {
    // Start sequence if requested and idle
    if (score_requested && score_state == ScoreState::IDLE) {
      score_requested = false;

      // Open blocker immediately, then wait before moving lever
      blocker.set_value(blocker_open_value);
      state_start_ms = pros::millis();
      score_state = ScoreState::OPEN_BLOCKER_DELAY;
    }

    switch (score_state) {
      case ScoreState::IDLE:
        // do nothing
        break;

      case ScoreState::OPEN_BLOCKER_DELAY:
        if (pros::millis() - state_start_ms >= blocker_open_delay_ms) {
          leverPID.timers_reset();
          leverPID.target_set(lever_score_position);
          score_state = ScoreState::MOVE_TO_SCORE;
        }
        break;

      case ScoreState::MOVE_TO_SCORE: {
        const double pos = lever_deg();
        int power = static_cast<int>(std::lround(leverPID.compute(pos)));

        // Cap by selected speed
        power = clamp127(power);
        if (power > active_lever_speed) power = active_lever_speed;
        if (power < -active_lever_speed) power = -active_lever_speed;

        move_lever(power);

        // If PID says we're done, stop and hold
        if (leverPID.exit_condition(false) != ez::RUNNING) {
          move_lever(0);
          state_start_ms = pros::millis();
          score_state = ScoreState::HOLD_SCORE;
        }
        break;
      }

      case ScoreState::HOLD_SCORE:
        move_lever(0);
        if (pros::millis() - state_start_ms >= lever_hold_ms) {
          leverPID.timers_reset();
          leverPID.target_set(lever_home_position);
          score_state = ScoreState::MOVE_TO_HOME;
        }
        break;

      case ScoreState::MOVE_TO_HOME: {
        const double pos = lever_deg();
        int power = static_cast<int>(std::lround(leverPID.compute(pos)));

        power = clamp127(power);
        if (power > active_lever_speed) power = active_lever_speed;
        if (power < -active_lever_speed) power = -active_lever_speed;

        move_lever(power);

        if (leverPID.exit_condition(false) != ez::RUNNING) {
          move_lever(0);

          // Close blocker AFTER lever is back home
          blocker.set_value(blocker_closed_value);

          score_state = ScoreState::IDLE;
        }
        break;
      }
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}