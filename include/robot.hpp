// ===================== robot.hpp =====================
#pragma once
#include "main.h"

class Robot {
 public:
  Robot(int firstStagePort,
        int leverPort,
        int rotationPort,
        char blockerPort,
        char liftPort,
        char matchloaderPort,
        char wingPort);

  void init();
  // First stage (roller)
  void intakeOn();
  void intakeOff();
  void toggleIntake();
  void reverseIntake();

  // Lift piston
  void toggleLift();
  void raise();
  void lower();

  // Blocker piston (stopper)
  // Note: blocker is handled automatically during scoring, but you can still expose controls later if you want.

  // Matchloader piston
  void matchloaderUp();
  void matchloaderDown();
  void toggleMatchloader();

  // Wing piston (cannot be up when lift is lowered)
  void wingUp();
  void wingDown();
  void toggleWing();

  // Lever scoring sequence (async)
  void score();

 private:
  // Hardware
  pros::Motor first_stage;
  pros::Motor lever;
  pros::Rotation lever_rotation;

  // Pistons
  pros::ADIDigitalOut blocker;
  pros::ADIDigitalOut lift;
  pros::ADIDigitalOut matchloader;
  pros::ADIDigitalOut wing;

  // State
  bool intake_running = false;

  bool lift_up = true;

  bool matchloader_up = false;  // defaults DOWN
  bool wing_up = false;         // defaults DOWN (and forced DOWN when lift is lowered)

  // ===== Lever constants (easy tuning) =====
  // Speeds (cap PID output)
  const int lever_full_speed = 127;  // max
  const int lever_slow_speed = 63;   // half max

  // Positions (degrees)
  const double lever_score_position = 90.0;
  const double lever_home_position = 0.0;

  // Timing
  const uint32_t blocker_open_delay_ms = 200;
  const uint32_t lever_hold_ms = 750;

  // Piston values (assumption: true=up/open, false=down/closed)
  const bool piston_up_value = true;
  const bool piston_down_value = false;

  // Blocker values
  const bool blocker_open_value = true;
  const bool blocker_closed_value = false;

  // Active speed cap chosen at the moment score() is requested
  int active_lever_speed = 127;

  // EZ PID (tune these later)
  ez::PID leverPID{1.2, 0.0, 6.0, 0.0, "Lever"};

  // Score task state machine
  enum class ScoreState {
    IDLE,
    OPEN_BLOCKER_DELAY,
    MOVE_TO_SCORE,
    HOLD_SCORE,
    MOVE_TO_HOME
  };

  volatile ScoreState score_state = ScoreState::IDLE;
  volatile bool score_requested = false;

  uint32_t state_start_ms = 0;  // used for blocker delay + hold timer

  // Task
  static void score_task_trampoline(void* param);
  void score_task();
  pros::Task scoreTask;

  // Helpers
  double lever_deg() const;    // rotation -> degrees
  void move_lever(int power);  // clamp and send
  void enforceWingRule();      // force wing down if lift is lowered
};