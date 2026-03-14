#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

extern pros::Motor intake;
extern pros::Motor lever;
extern Piston blocker;
extern Piston lift;
extern Piston matchloader;
extern Piston wing;
extern bool lift_toggle;
extern bool wing_toggle;
extern bool start_down;

void score();



// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');