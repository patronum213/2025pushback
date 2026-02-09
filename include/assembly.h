#pragma once

#include "vex.h"

using namespace vex;

enum lift_positions : int { IDLE = 229, LOADING = 206, SCORING = 99 };

class Assembly {
public:
    Assembly(
        mik::motor IntakeMotor, 
        mik::motor OuttakeMotor, 
        mik::piston tongue,
        mik::piston ramp,
        mik::piston gate,
        mik::piston wing,
        mik::piston odom_piston
    );
    
    void init();
    void control();

    void odom_piston_control(bool state);
    void S_system_control(int systemState);

    int lift_arm_position = IDLE;
    vex::task lift_task;
    
    mik::motor IntakeMotor;
    mik::motor OuttakeMotor; 
    mik::piston tongue;
    mik::piston ramp;
    mik::piston gate;
    mik::piston wing;
    mik::piston odom_piston;
};