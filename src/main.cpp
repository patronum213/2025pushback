#include "vex.h"

using namespace vex;
using namespace mik;

task UI;
//initalisations of usercontrol devices
controller Controller1 = controller(primary);
vex::motor LeftMotor1 = vex::motor(PORT18, ratio6_1, true);
vex::motor LeftMotor2 = vex::motor(PORT19, ratio6_1, true);
vex::motor LeftMotor3 = vex::motor(PORT20, ratio6_1, true);
vex::motor RightMotor1 = vex::motor(PORT8, ratio6_1, false);
vex::motor RightMotor2 = vex::motor(PORT9, ratio6_1, false);
vex::motor RightMotor3 = vex::motor(PORT10, ratio6_1, false);
vex::motor IntakeMotor = vex::motor(PORT16, ratio18_1, true);
vex::motor OuttakeMotor = vex::motor(PORT6, ratio18_1, true);
triport ThreeWirePort = vex::triport( vex::PORT22 );
digital_out toungue = vex::digital_out(ThreeWirePort.A);
digital_out ramp = vex::digital_out(ThreeWirePort.B);
digital_out gate = vex::digital_out(ThreeWirePort.F);
digital_out wing = vex::digital_out(ThreeWirePort.C);
digital_out odometryWheels = vex::digital_out(ThreeWirePort.D);
digital_in limitSwitch = vex::digital_in(ThreeWirePort.E);
rotation leftOdometry = rotation(PORT7, true);
rotation rightOdometry = rotation(PORT4, false);
inertial Inertial = inertial(PORT17);
//my math functions
//made for drive curve off, takes input of 0-1, only works positively
float distributeNormally (float input) {
  return std::pow(2.71828, -std::pow(((4*input)-2), 2));
};
//made for drive curve off, takes input of 0-1, only works positively
float distributeParabolically (float input) {
  return ((-3.5*std::pow((input-0.5), 2)) + 1);//-3.5 is the curving varible, power starts at 1/n%
};
//made for joystick curving, takes input of 0-1, works both ways
float distributeExponentially (float input, float tuning = 1.001) {//increasing tuning makes curve steeper
  return (input > 0 ? 
  (std::pow(tuning, 100*input)-1)/(std::pow(tuning, 100)-1):
  -(std::pow(tuning, std::abs(100*input))-1)/(std::pow(tuning, 100)-1)
  );
};

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
    UI.stop();
    auton_scr->start_auton();
}

void user_control(void) {
    while (calibrating) { task::sleep(50); }

    // How you want your drivetrain to stop during driver
    /*chassis.set_brake_type(brakeType::coast);
    
    assembly.init();

    while (true) {
        if (!control_disabled()) {
            // Add your user control code here
            chassis.control(drive_mode::SPLIT_ARCADE_CURVED);
            assembly.control();
        }
        task::sleep(5);

    }*/

    //my code
    LeftMotor1.setStopping(coast); 
  LeftMotor2.setStopping(coast); 
  LeftMotor3.setStopping(coast);
  RightMotor1.setStopping(coast);
  RightMotor2.setStopping(coast);
  RightMotor3.setStopping(coast);
  OuttakeMotor.setStopping(coast);
  odometryWheels.set(false);//retract odometry wheels
  toungue.set(true);
  ramp.set(true);
  float FBsensitivity = 0.8;
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
  // User control code here, inside the loop
  while (1) {
    //Driving Control
    //controller dead zone
    int deadzonepct  = 15;
    float Axis3 = Controller1.Axis3.position(percent);
    float Axis1 = Controller1.Axis1.position(percent);
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
      if ((abs(LeftMotor1.velocity(pct)) - abs(RightMotor1.velocity(pct))) < -PIDTolerancePct) {
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
      }
    }


    LeftSidePower = (LeftSidePower/100.0)*127.0;
    RightSidePower = (RightSidePower/100.0)*127.0;
    
    LeftMotor1.spin(directionType::fwd, LeftSidePower, voltageUnits::volt); 
    LeftMotor2.spin(directionType::fwd, LeftSidePower, voltageUnits::volt); 
    LeftMotor3.spin(directionType::fwd, LeftSidePower, voltageUnits::volt);
    RightMotor1.spin(directionType::fwd, RightSidePower, voltageUnits::volt);
    RightMotor2.spin(directionType::fwd, RightSidePower, voltageUnits::volt);
    RightMotor3.spin(directionType::fwd, RightSidePower, voltageUnits::volt);
    

    if (timer1 >= 0) {timer1 -= 1;};
    
    if (Controller1.ButtonUp.pressing() && !Uppressed) {
      if (turbo) {turbo = false; FBmult = FBsensitivity; LRmult = LRsensitivity; }
      else {turbo = true; FBmult = 1; LRmult = 1;}
      Uppressed = true;
    }
    if (!Controller1.ButtonUp.pressing()) {
      Uppressed = false;
    };
    //descoring wing
    if (Controller1.ButtonDown.pressing() && !Downpressed) {
      if (wing.value()) {wing.set(false);}
      else {wing.set(true);}
      Downpressed = true;
    }
    if (!Controller1.ButtonDown.pressing()) {
      Downpressed = false;
    };
    /*if (Controller1.ButtonDown.pressing()) {
        wing.set(false);
    }
    else {wing.set(true);}*/
    //toungue
    if (Controller1.ButtonB.pressing() && !Bpressed) {
      if (toungue.value()) {toungue.set(false);}
      else {toungue.set(true);
      systemState = 1;  
      }
      Bpressed = true;
    }
    if (!Controller1.ButtonB.pressing()) {
      Bpressed = false;
    };
    //intaking
    if (Controller1.ButtonR2.pressing() && !R2pressed) {
      if (systemState == 1) {systemState=0;}
      else {systemState = 1;}
      R2pressed = true;
    }
    if (!Controller1.ButtonR2.pressing()) {
      R2pressed = false;
    };
    //down outtaking
    if (Controller1.ButtonR1.pressing() && !R1pressed) {
      if (systemState == 4) {systemState=0;}
      else {systemState = 4;}
      R1pressed = true;
    }
    if (!Controller1.ButtonR2.pressing()) {
      R1pressed = false;
    };
    //middle outtaking
    if (Controller1.ButtonL1.pressing() && !L1pressed) {
      if (systemState == 3) {systemState=0;}
      else {systemState = 3;}
      L2pressed = true;
    }
    if (!Controller1.ButtonL1.pressing()) {
      L1pressed = false;
    };
    //top outtaking
    if (Controller1.ButtonL2.pressing() && !L2pressed) {
      systemState = 4; timer1 = 1;
      /*if (systemState == 2) {systemState=0;}
      else {systemState = 2;}*/
      
      L2pressed = true;
    }
    if (!Controller1.ButtonR2.pressing()) {
      L2pressed = false;
    };
    //brief backtake to loosen balls
    if (timer1 == 0) {systemState = 2;}

    if (Controller1.ButtonA.pressing()) {
      systemState = 0;
    }

    switch (systemState) {
      case 4://down outtaking
      IntakeMotor.spin(directionType::rev, 60, velocityUnits::pct);
      OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
      gate.set(true);
      break; 
      case 3://middle outtaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::fwd, 60, velocityUnits::pct);
      ramp.set(false);
      gate.set(false);
      break; 
      case 2://top outtaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      ramp.set(true);
      gate.set(false);
      break; 
      case 1://intaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::fwd, 50, velocityUnits::pct);
      //if (limitSwitch) {OuttakeMotor.spin(directionType::fwd, 10, velocityUnits::pct);}
      //else {OuttakeMotor.stop();}
      //OuttakeMotor.stop(coast);
      ramp.set(true);
      gate.set(true);
      //OuttakeMotor.stop();
      break; 
      case 0:
      default: 
      IntakeMotor.stop();
      OuttakeMotor.stop();
      break;
    }

    
    float avgDriveHeat = 
    (LeftMotor1.temperature(pct)+
    LeftMotor2.temperature(pct)+
    LeftMotor3.temperature(pct)+
    RightMotor1.temperature(pct)+
    RightMotor2.temperature(pct)+
    RightMotor3.temperature(pct))/6;
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(IntakeMotor.temperature(pct));
    Controller1.Screen.setCursor(1, 9);
    Controller1.Screen.print(LRmult);
    Controller1.Screen.setCursor(1, 17);
    Controller1.Screen.print(OuttakeMotor.temperature(pct));
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("left1 = %.2f    ", LeftMotor1.torque(Nm));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("left2 = %.2f    ", LeftMotor2.torque(Nm));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("left3 = %.2f    ", LeftMotor3.torque(Nm));
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("right1 = %.2f    ", RightMotor1.torque(Nm));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("right2 = %.2f    ", RightMotor2.torque(Nm));
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("right3 = %.2f    ", RightMotor3.torque(Nm));


    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
