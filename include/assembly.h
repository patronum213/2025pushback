#pragma once

#include "vex.h"

using namespace vex;

enum lift_positions : int { IDLE = 229, LOADING = 206, SCORING = 99 };

class Assembly {
public:
    Assembly(
        mik::motor intake_motor, 
        mik::motor outtake_motor, 
        mik::piston tougue,
        mik::piston ramp,
        mik::piston gate,
        mik::piston wing,
        mik::piston odom_piston
    );
    
    void init();
    void control();

    /*void move_lift_arm();
    void lift_arm_control();
    void intake_motors_control();
    void long_piston_control();*/
    void odom_piston_control(bool state);

    int lift_arm_position = IDLE;
    vex::task lift_task;
    
    mik::motor intake_motor;
    mik::motor outtake_motor; 
    mik::piston tougue;
    mik::piston ramp;
    mik::piston gate;
    mik::piston wing;
    mik::piston odom_piston;
};