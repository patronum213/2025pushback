#include "vex.h"

using namespace vex;
using namespace mik;

void default_constants(void) {
    chassis.set_control_constants(5, 10, 1.019, 5, 10, 1.019);

    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
    chassis.set_turn_constants(10, 0.4, 0.04, 2.5, 10);
    chassis.set_drive_constants(10, 1.55, 1.05, 9, 0.75);
    chassis.set_heading_constants(6, .5, 0, 1.8, 0);
    chassis.set_swing_constants(12, .437, .0295, 3.486, 15);

    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_turn_exit_conditions(1.5, 75, 2000);
    chassis.set_drive_exit_conditions(1, 75, 3000);
    chassis.set_swing_exit_conditions(1.25, 75, 3000);
}

void odom_constants(void) {
    default_constants();
    chassis.heading_max_voltage = 10;
    chassis.drive_max_voltage = 8;
    chassis.drive_settle_error = 3;
    chassis.boomerang_lead = .5;
    chassis.boomerang_setback = 2;    
}

std::string template_auto(bool calibrate, auto_variation var, bool get_name) {
    /* The first variation will be this auto */
    if (var == one) {}

    /* We declare and allow a second variation of this auto; 
    You may want this if you want a different movements in the same starting configuration */
    if (var == two) { return template_auto_other_variation(calibrate, get_name); }

    if (get_name) { /* Give a desciption of your auto */ return "template auto 1 (3 objs)"; }
    if (calibrate) {
        /* Initialize robots starting position "https://path.jerryio.com/" and/or add extra movements to line up robots 
        starting position **IF MOVING DURING CALIBRATION DO BEFORE FIELD CONTROLLER PLUG IN** */
        chassis.set_coordinates(55, 23.5, 90);
    
        /* Example of turning before auto is ran */
        chassis.turn_max_voltage = 6; 
        chassis.turn_to_angle(45);

        return "";
    }
    
    /* We now run the auto */ 
    chassis.drive_distance(10);
    chassis.drive_distance(-10);

    return "";
}
std::string template_auto_other_variation(bool calibrate, bool get_name) {
    if (get_name) { return "template auto 2 (4 objs)"; }
    
    // Mirror template_auto() from the x-axis
    chassis.mirror_all_auton_y_pos();
    
    if (calibrate) {
        // Coordinates will be set to (55, -23.5) as y_pos is mirrored
        template_auto(calibrate, one, get_name);
        return "";
    }
    
    // Run auto, make sure to pass in one as var.
    template_auto(calibrate, one, get_name);

    return "";
}


std::string blue_left_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 90);

        return "";
    }
    chassis.set_brake_type(brakeType::brake);
    assembly.odom_piston.set(true);
    chassis.drive_distance(29);//drive out to the goal

    //take from the first chute
    chassis.turn_to_angle(180);//turn to goal
    assembly.tongue.set(true);
    assembly.S_system_control(1);//start intake
    wait(1000, msec);
    chassis.drive_distance(24, {.max_voltage = 5, .timeout = 2000});//drive in to chute
    chassis.drive_distance(-1);
    chassis.drive_distance(10, {.timeout = 1250});//jostle slightly to get all the balls

    //move over to the other side
    chassis.drive_distance(-7);//drive out enough to turn
    assembly.tongue.set(false);
    assembly.S_system_control(0);//stop intake
    chassis.turn_to_angle(200);//turn diagonal towards the wall-goal channel
    chassis.drive_distance(-23.5);//drive towards the wall
    chassis.turn_to_angle(180);//turn to just over 90 (towards the wall)
    chassis.drive_distance(-60, {.heading = 174});//drive along the goal, turning slightly towards it

    //line up with the goal and score
    chassis.left_swing_to_angle(30, {.max_voltage = 8, .turn_direction  = ccw});//swing around to the goal's mouth
    chassis.turn_to_angle(0);
    chassis.drive_distance(-20, {.timeout = 1500});//drive in to it
    assembly.S_system_control(2);//outtake
    wait(500, msec);
    assembly.S_system_control(4);
    wait(150, msec);
    assembly.S_system_control(2);
    wait(1100, msec);
    assembly.tongue.set(true);//outtake
    wait(500, msec);
    
    //take from the 2nd chute and score
    chassis.drive_distance(35, {.max_voltage = 5, .timeout = 2000, .heading = -2});//drive away from it in to the other chute
    assembly.S_system_control(1);//go back to intaking
    wait(900, msec);
    chassis.drive_distance(-1);
    chassis.drive_distance(10, {.timeout = 1250});//jostle it slightly
    chassis.drive_distance(-35, {.timeout = 1500, .heading = 0, .max_voltage = 7});//drive out back in to the goal
    assembly.S_system_control(2);//outtake
    wait(500, msec);
    assembly.S_system_control(4);
    wait(150, msec);
    assembly.S_system_control(2);
    wait(1100, msec);

    //move over to the other side of the field
    chassis.drive_distance(10);//back out
    chassis.turn_to_angle(90);//turn to the otherside of the field
    chassis.drive_distance(-94.5, {.heading = 90});//drive over to the otherside 
    chassis.turn_to_angle(0); //turn towards the other chute




    
    //repeat otherside code here

    //take balls from the 3rd chute
    assembly.S_system_control(1);//start intake
    chassis.drive_distance(24, {.max_voltage = 5, .timeout = 2000});//drive in to goal
    chassis.drive_distance(-1);
    chassis.drive_distance(10, {.timeout = 1250});//jostle slightly to get all the balls
    chassis.drive_distance(-35, {.timeout = 1500, .max_voltage = 8});//drive in to the goal just to center ourselves
    assembly.S_system_control(0);//stop intake
    chassis.drive_distance(24, {.heading = -2});
    assembly.tongue.set(false);
    chassis.turn_to_angle(20);//turn diagonal towards the wall-goal channel
    chassis.drive_distance(-26);//drive towards the wall
    chassis.turn_to_angle(0);//turn to just over 90 (towards the wall)
    chassis.drive_distance(-60, {.heading = 0});//drive along the goal, turning slightly towards it


    chassis.left_swing_to_angle(-135, {.max_voltage = 8, .turn_direction  = ccw});//swing around to the goal's mouth
    chassis.turn_to_angle(-180);
    chassis.drive_distance(-20, {.timeout = 1500});//drive in to it

    assembly.S_system_control(2);//outtake
    wait(500, msec);
    assembly.S_system_control(4);
    wait(150, msec);
    assembly.S_system_control(2);
    wait(1100, msec);
    assembly.tongue.set(true);
    wait(500, msec);

    
    chassis.drive_distance(35, {.max_voltage = 5, .timeout = 2000, .heading = -180});//drive away from it in to the other chute
    assembly.S_system_control(1);//go back to intaking
    wait(900, msec);
    chassis.drive_distance(-1);
    chassis.drive_distance(10, {.timeout = 1250});//jostle it slightly
    chassis.drive_distance(-35, {.timeout = 1500, .heading = -182, .max_voltage = 8});//drive out back in to the goal
    assembly.S_system_control(2);//outtake
    wait(500, msec);
    assembly.S_system_control(4);
    wait(150, msec);
    assembly.S_system_control(2);
    wait(1100, msec);


    //back out and line up to the park zone
    assembly.tongue.set(false);
    assembly.S_system_control(1);//intake in prep for clear
    chassis.drive_distance(35, {.heading = 135});
    chassis.drive_distance(5, {.heading = -260});
    chassis.drive_distance(6, {.timeout = 550});
    assembly.odom_piston.set(false);
    assembly.tongue.set(true);
    wait(500, msec);
    chassis.drive_distance(9999, {.timeout = 1150, .wait = false});
    wait(500, msec);
    assembly.tongue.set(false);
    /**/
    return "";
}
std::string blue_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_left_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "blue left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    chassis.drive_distance(24);
    chassis.turn_to_angle(-135);
    chassis.drive_distance(-10);
    assembly.S_system_control(3);
    wait(500, msec);
    assembly.S_system_control(4);
    wait(100, msec);
    assembly.S_system_control(3);
    wait(1000, msec);
    assembly.S_system_control(1);
    chassis.drive_distance(45);
    chassis.turn_to_angle(180);
    chassis.drive_distance(24, {.max_voltage = 7, .timeout = 2000});
    chassis.drive_distance(-1);
    chassis.drive_distance(10, {.timeout = 1000});
    chassis.drive_distance(35, {.timeout = 2000});
    assembly.S_system_control(2);
    wait(5000, msec);
    return "";
}
std::string blue_right_winpoint(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "blue right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string blue_right_elim(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "blue right elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}

std::string red_left_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }
    
    return "";
}
std::string red_left_sawp(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}
std::string red_left_elim(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red left elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }
    
    return "";
}
std::string red_right_winpoint(bool calibrate, auto_variation var, bool get_name) { 
    if (get_name) { return "red right winpoint"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }

    return "";
}
std::string red_right_sawp(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "red right sawp"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);
        
        return "";
    }

    return "";
}
std::string red_right_elim(bool calibrate, auto_variation var, bool get_name) {   
    if (get_name) { return "red right elim"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}

std::string skills(bool calibrate, auto_variation var, bool get_name) {
    if (get_name) { return "skills"; }
    if (calibrate) {
        chassis.set_coordinates(0, 0, 0);

        return "";
    }

    return "";
}