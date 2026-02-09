#include "vex.h"

using namespace vex;
using namespace mik;

task UI;

static int run_UI() {
    UI_init();
    UI_controller_auton_selector();
    UI_render();
    return 0;
}

void pre_auton() {
    init();
    default_constants();
    UI = task(run_UI);
}

void auton(void) {
    assembly.odom_piston_control(true);
    UI.stop();
    auton_scr->start_auton();
}
float distributeExponentially (float input, float tuning = 1.001) {//increasing tuning makes curve steeper
  return (input > 0 ? 
  (std::pow(tuning, 100*input)-1)/(std::pow(tuning, 100)-1):
  -(std::pow(tuning, std::abs(100*input))-1)/(std::pow(tuning, 100)-1)
  );
};
void user_control(void) {
    while (calibrating) { task::sleep(50); }
    //assembly.init();
    /*chassis.set_brake_type(brakeType::coast);
    assembly.odom_piston.set(false);//retract odometry wheels
    assembly.tongue.set(true);
    assembly.ramp.set(true);
    */float FBsensitivity = 0.8;
    float LRsensitivity = 0.25;
    float PIDIncrement = 0.25;
    float PIDTolerancePct = 5;
    float rampRate = 250;//time to reach full power in ms
    float rampFloor = 4;//cycles of ramp that it resets too

    float LeftSidePower = 0.0;
    float RightSidePower = 0.0;
    float FBmult = FBsensitivity;
    float LRmult = LRsensitivity;
    bool L1pressed = false;
    bool L2pressed = false;
    bool R1pressed = false;
    bool R2pressed = false;
    bool Bpressed = false;
    bool Downpressed = false;
    bool Uppressed = false;
    float LeftRampProgress = rampFloor;
    float RightRampProgress = rampFloor;
    int systemState = 1;//0 is at rest, 1 is intaking, 2 is top outtaking, 3 is bottom outtaking
    int timer1 = 0;
    bool turbo = false;
    while (false) {//TODO: change to true
        if (!control_disabled()) {
            //Driving Control
            //controller dead zone
            int deadzonepct  = 15;
            float Axis3 = Controller.Axis3.position(percent);
            float Axis1 = Controller.Axis1.position(percent);
            float Axis3Dead = Axis3 > deadzonepct ? ((Axis3 - deadzonepct)*1.00/(100-deadzonepct))*100 : 
            Axis3Dead = Axis3 < -deadzonepct ? ((Axis3 + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
            float Axis1Dead = Axis1 > deadzonepct ? ((Axis1 - deadzonepct)*1.00/(100-deadzonepct))*100 : 
            Axis1Dead = Axis1 < -deadzonepct ? ((Axis1 + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
            //joystick curve, taking place after deadzoning
            float Axis1Curved = distributeExponentially(Axis1Dead/100.0, 1.025)*100.0;
            float Axis3Curved = distributeExponentially(Axis3Dead/100.0, 1.025)*100.0;
            //sensitivity
            Axis1Curved *= LRmult;
            Axis3Curved *= FBmult;
            //set motor powers
            LeftSidePower = (Axis3Curved + Axis1Curved)/2;
            RightSidePower = (Axis3Curved - Axis1Curved)/2;

            //ramping
            if (abs(LeftSidePower) > 0) {LeftRampProgress += 1; LeftRampProgress = std::min(LeftRampProgress, rampRate/25);}
            else {LeftRampProgress = rampFloor;}
            if (abs(RightSidePower) > 0) {RightRampProgress += 1; RightRampProgress = std::min(RightRampProgress, rampRate/25);}
            else {RightRampProgress = rampFloor;}
            LeftSidePower *= LeftRampProgress/(rampRate/25);
            RightSidePower *= RightRampProgress/(rampRate/25);

            

            if (abs(Axis3Dead) > 0 && abs(Axis1Dead) > 0) {//if we're not turning, use PID to make sure the robot driving straight
            float basePower = Axis3Curved;
            float RightSidePower = Axis3Curved;
            float LeftSidePower = Axis3Curved;
            float PIDIncrementSigned = Axis3Curved >= 0 ? PIDIncrement : -PIDIncrement; 
            /*if ((abs(LeftMotor1.velocity(pct)) - abs(RightMotor1.velocity(pct))) < -PIDTolerancePct) {
                if (abs(LeftSidePower) >= basePower) {
                RightSidePower -= PIDIncrementSigned;
                }
                else {
                LeftSidePower += PIDIncrementSigned;
                }
            }
            else if ((abs(RightMotor1.velocity(pct)) - abs(LeftMotor1.velocity(pct))) < -PIDTolerancePct) {
                if (abs(RightSidePower) >= basePower) {
                LeftSidePower -= PIDIncrementSigned;
                }
                else {
                RightSidePower += PIDIncrementSigned;
                }
            }*/
            }
            /////////////////////////////////////////////////////////////////////////1

            LeftSidePower = (LeftSidePower/100.0)*127.0;
            RightSidePower = (RightSidePower/100.0)*127.0;
            
            chassis.left_drive.spin(fwd, LeftSidePower, volt);
            chassis.right_drive.spin(fwd, RightSidePower, volt);
            

            if (timer1 >= 0) {timer1 -= 1;};
            
            if (Controller.ButtonUp.pressing() && !Uppressed) {
            if (turbo) {turbo = false; FBmult = FBsensitivity; LRmult = LRsensitivity; }
            else {turbo = true; FBmult = 1; LRmult = 1;}
            Uppressed = true;
            }
            if (!Controller.ButtonUp.pressing()) {
            Uppressed = false;
            };
            //descoring assembly.wing
            if (Controller.ButtonDown.pressing() && !Downpressed) {
            if (assembly.wing.state()) {assembly.wing.set(false);}
            else {assembly.wing.set(true);}
            Downpressed = true;
            }
            if (!Controller.ButtonDown.pressing()) {
            Downpressed = false;
            };
            /*if (Controller.ButtonDown.pressing()) {
                assembly.wing.set(false);
            }
            else {assembly.wing.set(true);}*/
            //tongue
            if (Controller.ButtonB.pressing() && !Bpressed) {
            if (assembly.tongue.state()) {assembly.tongue.set(false);}
            else {assembly.tongue.set(true);
            systemState = 1;  
            }
            Bpressed = true;
            }
            if (!Controller.ButtonB.pressing()) {
            Bpressed = false;
            };
            //intaking
            if (Controller.ButtonR2.pressing() && !R2pressed) {
            if (systemState == 1) {systemState=0;}
            else {systemState = 1;}
            R2pressed = true;
            }
            if (!Controller.ButtonR2.pressing()) {
            R2pressed = false;
            };
            //down outtaking
            if (Controller.ButtonR1.pressing() && !R1pressed) {
            if (systemState == 4) {systemState=0;}
            else {systemState = 4;}
            R1pressed = true;
            }
            if (!Controller.ButtonR2.pressing()) {
            R1pressed = false;
            };
            //middle outtaking
            if (Controller.ButtonL1.pressing() && !L1pressed) {
            if (systemState == 3) {systemState=0;}
            else {systemState = 3;}
            L2pressed = true;
            }
            if (!Controller.ButtonL1.pressing()) {
            L1pressed = false;
            };
            //top outtaking
            if (Controller.ButtonL2.pressing() && !L2pressed) {
            systemState = 4; timer1 = 1;
            /*if (systemState == 2) {systemState=0;}
            else {systemState = 2;}*/
            
            L2pressed = true;
            }
            if (!Controller.ButtonR2.pressing()) {
            L2pressed = false;
            };
            //brief backtake to loosen balls
            if (timer1 == 0) {systemState = 2;}

            if (Controller.ButtonA.pressing()) {
            systemState = 0;
            }

            assembly.S_system_control(systemState);
            
            
            Brain.Screen.setCursor(1, 1);
            Brain.Screen.print("left = %.2f    ", LeftSidePower);
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("right = %.2f    ", LeftSidePower);
            Brain.Screen.setCursor(3, 1);
            Brain.Screen.print("cont1 = %.2f    ", Controller.Axis1.position(percent));
            Brain.Screen.setCursor(4, 1);
            Brain.Screen.print("cont3 = %.2f    ", Controller.Axis3.position(percent));
            Brain.Screen.setCursor(2, 1);
            Brain.Screen.print("sysState = %.2f    ", systemState);

        }
        task::sleep(5);
    }
}

int main() {
    Competition.autonomous(auton);
    Competition.drivercontrol(user_control);
    
    pre_auton();

    while (true) {
        task::sleep(100);
    }
}
