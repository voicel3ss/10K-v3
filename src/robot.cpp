#include "robot.hpp"
#include "okapi/api.hpp"

#include <cmath>

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
}
//utility 
static inline double rot_deg(const pros::Rotation& r) {
  // Rotation sensor is centidegrees (0.01 deg)
  return r.get_position() / 100.0;
}


double Robot::getLeverRotation() {
  return rot_deg(lever_rotation);
}
void Robot::enforceWingRule() {
  // Wing is uncontrollable when lift is lowered: force down
  if (!lift_up) {
    wing.set_value(piston_down_value);
    wing_up = false;
  }
}
double Robot::decidePoseLever() {
  return lift_up ? lever_score_pose_high : lever_score_pose_mid;
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

  lever.set_brake_mode(MOTOR_BRAKE_BRAKE);
  lever.set_zero_position(0);
  intake_running = false;
  
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

bool Robot::isIntakeRunning() const { return intake_running; }

// -------- Lever score sequence (async) --------

void Robot::score() {
  // Only accept a new request if we're idle
  if (score_state != ScoreState::IDLE) return;

  // Choose speed cap based on current lift position
  active_lever_speed = lift_up ? lever_full_speed : lever_slow_speed; //until pneumatics 

  score_requested = true;
}

void Robot::score_task_trampoline(void* param) {
  static_cast<Robot*>(param)->score_task();
}

void Robot::score_task() {
  while (true) {
    if (score_requested && score_state == ScoreState::IDLE) {
      score_requested = false;

      blocker.set_value(blocker_open_value);
      
      state_start_ms = pros::millis();
      score_state = ScoreState::OPEN_BLOCKER_DELAY;
      pros::delay(10);
    }

    switch (score_state) {
      case ScoreState::IDLE:
        //lever.move(0);
        break;

      case ScoreState::OPEN_BLOCKER_DELAY:
        if (pros::millis() - state_start_ms >= blocker_open_delay_ms) { 
          lever.move_absolute(
              decidePoseLever(),
              active_lever_speed);
          
          state_start_ms = pros::millis();
          score_state = ScoreState::MOVE_TO_SCORE;
        }
        break;

      case ScoreState::MOVE_TO_SCORE: 
        if (std::abs(lever.get_position() - decidePoseLever()) < lever_settle_window_deg || pros::millis() - state_start_ms > score_time_ms) {
          state_start_ms = pros::millis();
          score_state = ScoreState::HOLD_SCORE;
        }
        break;

      case ScoreState::HOLD_SCORE:
        lever.move(0);
        if (pros::millis() - state_start_ms >= lever_hold_ms) {
          score_state = ScoreState::MOVE_TO_HOME;
          move_home_start = pros::millis();
        }
        break;

      case ScoreState::MOVE_TO_HOME: {
        lever.move_absolute(lever_home_position, active_lever_speed);
        if (pros::millis() - move_home_start > move_home_time_ms) {  
          lever.move(0);
          blocker.set_value(blocker_closed_value);
          score_state = ScoreState::IDLE;
        }
        break;
      }

      case ScoreState::MOVE_HOME_START:
        double pos = rot_deg(lever_rotation);
        double err = lever_home_position - pos;
        lever.move(-1 * active_lever_speed);
        if (std::abs(err) < lever_settle_window_deg || pros::millis() - move_home_start > move_home_time_ms) {  
          lever.move(0);
          blocker.set_value(blocker_closed_value);
          score_state = ScoreState::IDLE;
        }
        break;  
    }
  }
}
