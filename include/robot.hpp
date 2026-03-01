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
  void initialize();

  // Wing piston (cannot be up when lift is lowered)
  void wingUp();
  void wingDown();
  void toggleWing();

  // Lever scoring sequence (async)
  void score();
  void scoreMid();
  double getLeverRotation();
  double decidePoseLever();

  bool isIntakeRunning() const;
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
  // Speeds
  const int lever_full_speed = 127;  // max
  const int lever_slow_speed = 70;   // TODO tune this max for now 
  

  // Positions (degrees)
  const double lever_score_pose_high = 600.0;
  const double lever_score_pose_mid = 700.0;//4 bar changes angle to stop
  const double lever_home_position = 0.0;
  

  // Timing
  const uint32_t score_time_ms = 2000;
  const uint32_t move_home_time_ms = 800; // time to reset cuz start time too old 
  const uint32_t blocker_open_delay_ms = 100; //
  const uint32_t lever_hold_ms = 300; //time waiting at top 

  // Piston values (assumption: true=up/open, false=down/closed)
  const bool piston_up_value = false;
  const bool piston_down_value = true;

  // Blocker values
  const bool blocker_open_value = true;
  const bool blocker_closed_value = false;

  // Lever settling window
  const double lever_settle_window_deg = 10.0;
  bool startLoop = true;

  // Active speed cap chosen at the moment score() is requested
  int active_lever_speed = 127;

  // Score task state machine
  enum class ScoreState {
    IDLE,
    OPEN_BLOCKER_DELAY,
    MOVE_TO_SCORE,
    HOLD_SCORE,
    MOVE_TO_HOME, 
    MOVE_HOME_START
  };

  volatile ScoreState score_state = ScoreState::IDLE;
  volatile bool score_requested = false;

  uint32_t state_start_ms = 0;  // used for blocker delay + hold timer
  uint32_t move_home_start = 0; // used for move to home timeout

  // Task
  static void score_task_trampoline(void* param);
  void score_task();
  pros::Task scoreTask;

  // Helpers
  void enforceWingRule();  // force wing down if lift is lowered
};
