/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
triport ThreeWirePort = vex::triport( vex::PORT22 );
digital_out toungue = vex::digital_out(ThreeWirePort.A);
digital_out leftWing = vex::digital_out(ThreeWirePort.B);
digital_out rightWing = vex::digital_out(ThreeWirePort.C);

rotation Odometry = rotation(PORT20, false);
inertial Inertial = inertial(PORT16);

void resetMotorEncoders(void) {
  LeftMotor1.resetPosition(); 
  LeftMotor2.resetPosition(); 
  LeftMotor3.resetPosition();
  RightMotor1.resetPosition();
  RightMotor2.resetPosition();
  RightMotor3.resetPosition();
  Odometry.resetPosition();
};
void setDriveMotorStopping(vex::brakeType type) {
  LeftMotor1.setStopping(type);
  LeftMotor2.setStopping(type);
  LeftMotor3.setStopping(type);
  RightMotor1.setStopping(type);
  RightMotor2.setStopping(type);
  RightMotor3.setStopping(type);
};
//made for drive curve off, takes input of 0-1, only works positively
float distributeNormally (float input) {
  return std::pow(2.71828, -std::pow(((4*input)-2), 2));
};
//made for drive curve off, takes input of 0-1, only works positively
float distributeParabolically (float input) {
  return ((-3.5*std::pow((input-0.5), 2)) + 1);//-3.5 is the curving varible, power starts at 1/n%
};
//made for joystick curving, takes input of 0-1, works both ways
float distributeExponentially (float input) {
  return (input > 0 ? 
  (std::pow(1.025, 100*input)-1)/(std::pow(1.025, 100)-1):
  -(std::pow(1.025, std::abs(100*input))-1)/(std::pow(1.025, 100)-1)
  );
};
//distance, maxSpeed, fowards
void MoveStraight(float distance, int maxSpeed, bool fowards) {
  resetMotorEncoders();
  //odometry wheels are 2 inches in diamametere, times pi means curcumernce is 6.28318 in
  //times 4/3 for the gearing (48/36)
  //gives us the final multiplier of 8.3775733333 inches per revolution
  //distance (in inches) is divided by wheel curcumfrence mutiplied by gear raito
  //TODO: switch this to pid for greater accuracy.
  float tuningConstant = 0.713;
  float gearingConstant = (3.0/4.0);
  float distanceRev = ((distance/6.28318)/gearingConstant);
  distanceRev *= tuningConstant;
  if (fowards) {
    while (LeftMotor2.position(rev) < distanceRev or RightMotor2.position(rev) < distanceRev) {
    float distanceTraveledPctLeft = (LeftMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeedLeft = distributeParabolically(distanceTraveledPctLeft/100.0)*100.0;
    float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed/100.0)), 10.0);

    float distanceTraveledPctRight = (RightMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeedRight = distributeParabolically(distanceTraveledPctRight/100.0)*100.0;
    float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed/100.0)), 10.0);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("leftmotor2 %.2f  ", LeftMotor2.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("distanceTraveledPctLeft = %.2f    ", distanceTraveledPctLeft);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("distributedSpeedLeft %.2f  ", distributedSpeedLeft);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Rightmotor2 %.2f  ", RightMotor2.position(rev));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("distanceTraveledPctRight = %.2f    ", distanceTraveledPctRight);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("distributedSpeedRight %.2f  ", distributedSpeedRight);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);


    LeftMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
    if (LeftMotor2.position(rev) > distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
    }
    if (RightMotor2.position(rev) > distanceRev) {
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }
    };
  }
  else if (!fowards) {
    while (LeftMotor2.position(rev) > -distanceRev or RightMotor2.position(rev) > -distanceRev ) {
    float distanceTraveledPctLeft = (LeftMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeedLeft = distributeParabolically(-distanceTraveledPctLeft/100.0)*100.0;
    float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed/100.0)), 10.0);

    float distanceTraveledPctRight = (RightMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeedRight = distributeParabolically(-distanceTraveledPctRight/100.0)*100.0;
    float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed/100.0)), 10.0);
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("leftmotor2 %.2f  ", LeftMotor2.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("distanceTraveledPctLeft = %.2f    ", distanceTraveledPctLeft);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("distributedSpeedLeft %.2f  ", distributedSpeedLeft);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Rightmotor2 %.2f  ", RightMotor2.position(rev));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("distanceTraveledPctRight = %.2f    ", distanceTraveledPctRight);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("distributedSpeedRight %.2f  ", distributedSpeedRight);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);

    LeftMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
    if (LeftMotor2.position(rev) < -distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
    }
    if (RightMotor2.position(rev) < -distanceRev) {
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }
    };
  }
}
//degrees, maxSpeed; positive is right negative is left
void MoveTurning(float degrees, int maxSpeed) {
  float tolernceConstant = 0.3;
  resetMotorEncoders();
  Inertial.resetRotation();
    while (std::abs(Inertial.rotation(deg) - degrees) >= tolernceConstant) {//to turn right, left wheels must go fowards while right wheels must go backwards

    float targetDifferncePct = std::abs(((std::abs(Inertial.rotation(deg) - degrees))/degrees)*100.0);
    float distributedSpeed = distributeParabolically((100.0-(std::min(targetDifferncePct*1.0, 100.0)))/100.0)*100.0;
    float ajustedSpeedRight = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
    float ajustedSpeedLeft = std::max((distributedSpeed * (maxSpeed/100.0)), 10.0);
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Inertial degrees = %.2f    ", Inertial.rotation(deg));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("targetDifferncePct = %.2f    ", targetDifferncePct);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("distributedSpeed %.2f  ", distributedSpeed);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);

    if ((degrees - Inertial.rotation())>0) {
      LeftMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
      RightMotor1.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      RightMotor2.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      RightMotor3.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
    }
    else {
      LeftMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
      RightMotor1.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      RightMotor2.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      RightMotor3.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    }
      if (abs(Inertial.rotation() - degrees) <= tolernceConstant) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
        RightMotor1.stop(); 
        RightMotor2.stop(); 
        RightMotor3.stop();
      }
    };
    
  
};
//degrees, maxSpeed, isTurningRight
void MoveTurningOld(float degrees, int maxSpeed, bool isturningright) {
  resetMotorEncoders();
  //wheels are 2 inches in diamametere, times pi means curcumernce is 7.853975 in
  //wheel to wheel width is 14.35, time pi means one 360 degree turn is 45.0818165 in covered
  //divided by 360 is 0.125227 inches covered per degree of turning
  float tuningConstant = 0.778;//note: tuned for 50% power
  float gearingConstant = 3.0/4.0;
  float distanceInch = degrees*0.125227;
  //using the same inch to revolution from driveStraight
  float distanceRev = (distanceInch/7.853975)/gearingConstant;
  distanceRev *= tuningConstant;
  if (isturningright) {
    while (LeftMotor2.position(rev) < distanceRev or RightMotor2.position(rev) > -distanceRev) {//to turn right, left wheels must go fowards while right wheels must go backwards
    
    float distanceTraveledPctRight = (RightMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeedRight = distributeParabolically(-distanceTraveledPctRight/100.0)*100.0;
    float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed/100.0)), 10.0);

    float distanceTraveledPctLeft = (LeftMotor2.position(rev)/distanceRev)*100.0;
    float distributedSpeedLeft = distributeParabolically(distanceTraveledPctLeft/100.0)*100.0;
    float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed/100.0)), 10.0);
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("leftmotor2 %.2f  ", LeftMotor2.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("distanceTraveledPctLeft = %.2f    ", distanceTraveledPctLeft);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("distributedSpeedLeft %.2f  ", distributedSpeedLeft);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Rightmotor2 %.2f  ", RightMotor2.position(rev));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("distanceTraveledPctRight = %.2f    ", distanceTraveledPctRight);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("distributedSpeedRight %.2f  ", distributedSpeedRight);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);

    LeftMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      if (LeftMotor2.position(rev) > distanceRev) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
      }
      if (RightMotor2.position(rev) < -distanceRev) {
        RightMotor1.stop(); 
        RightMotor2.stop(); 
        RightMotor3.stop();
      }
    };
  }
  else if (!isturningright) {
    while (RightMotor2.position(rev) < distanceRev or LeftMotor2.position(rev) > -distanceRev) {
    
    float distanceTraveledPctLeft = (LeftMotor2.position(rev)/distanceRev);
    float distributedSpeedLeft = distributeParabolically(-distanceTraveledPctLeft);
    float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed * 1.0)), 5.0);
    
    float distanceTraveledPctRight = (RightMotor2.position(rev)/distanceRev);
    float distributedSpeedRight = distributeParabolically(distanceTraveledPctRight);
    float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed * 1.0)), 5.0);
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("leftmotor2 %.2f  ", LeftMotor2.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("distanceTraveledPctLeft = %.2f    ", distanceTraveledPctLeft);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("distributedSpeedLeft %.2f  ", distributedSpeedLeft);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Rightmotor2 %.2f  ", RightMotor2.position(rev));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("distanceTraveledPctRight = %.2f    ", distanceTraveledPctRight);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("distributedSpeedRight %.2f  ", distributedSpeedRight);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);

    LeftMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    if (LeftMotor2.position(rev) < -distanceRev) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
    }
    if (RightMotor2.position(rev) > distanceRev) {
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }
    };
  }
};
//distance traveled by the greater side; leftToRightRaito determines the turning direction.
//robot turns right if it is greater than one, turns left if it is lesser
void TurnWithRatio(float distance, int maxSpeed, double LeftToRightRatio, bool fowards) {
  resetMotorEncoders();
  //turn with ratio uses the ratio of left wheel power to right wheel power to determine direction
  //if LtoRratio is greater than 1, it turns right, less than one and it turns left
  //ratios should be given in fractions anyway to help keep track of turns 
  //TODO: switch this to pid for greater accuracy.
  float tuningConstant = 1;
  float gearingConstant = 3.0/4.0;
  float distanceRev = (distance/7.853975)/gearingConstant;
  distanceRev *= tuningConstant;
  float leftSideMultiplier = 1;
  float rightSideMultiplier = 1;
    if (LeftToRightRatio > 1) {
      if (1/LeftToRightRatio <= 0.00001) {rightSideMultiplier = 0;}
      else {rightSideMultiplier = 1/LeftToRightRatio;}
    }
    else if (LeftToRightRatio < 1) {
      leftSideMultiplier = LeftToRightRatio;
    };
    float distanceRevRight = distanceRev*rightSideMultiplier;
    float distanceRevLeft = distanceRev*leftSideMultiplier;

    

    if (fowards) {
      while ((LeftMotor2.position(rev) < distanceRevLeft or RightMotor2.position(rev) < distanceRevRight)) {
      float distanceTraveledPctRight = (RightMotor2.position(rev)/distanceRevRight)*100.0;
      float distributedSpeedRight = distributeParabolically(distanceTraveledPctRight/100.0)*100.0;
      float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed/100.0)), 10.0);

      float distanceTraveledPctLeft = (LeftMotor2.position(rev)/distanceRevLeft)*100.0;
      float distributedSpeedLeft = distributeParabolically(distanceTraveledPctLeft/100.0)*100.0;
      float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed/100.0)), 10.0);
      
      ajustedSpeedLeft *= leftSideMultiplier;
      ajustedSpeedRight *= rightSideMultiplier;

      LeftMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
      RightMotor1.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      RightMotor2.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      RightMotor3.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      if (LeftMotor2.position(rev) > distanceRev) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
      }
      if (RightMotor2.position(rev) > distanceRev) {
        RightMotor1.stop(); 
        RightMotor2.stop(); 
        RightMotor3.stop();
      }
      };
    }
    else if (!fowards) {
      while ((LeftMotor2.position(rev) > -distanceRevLeft or RightMotor2.position(rev) > -distanceRevRight)) {
      float distanceTraveledPctRight = (RightMotor2.position(rev)/distanceRevRight)*100.0;
      float distributedSpeedRight = distributeParabolically(distanceTraveledPctRight/100.0)*100.0;
      float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed/100.0)), 10.0);

      float distanceTraveledPctLeft = (LeftMotor2.position(rev)/distanceRevLeft)*100.0;
      float distributedSpeedLeft = distributeParabolically(distanceTraveledPctLeft/100.0)*100.0;
      float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed/100.0)), 10.0);
      
      ajustedSpeedLeft *= leftSideMultiplier;
      ajustedSpeedRight *= rightSideMultiplier;

      LeftMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
      RightMotor1.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      RightMotor2.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      RightMotor3.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      if (LeftMotor2.position(rev) < -distanceRev) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
      }
      if (RightMotor2.position(rev) < -distanceRev) {
        RightMotor1.stop(); 
        RightMotor2.stop(); 
        RightMotor3.stop();
      }
      };
    }  
};
void MoveFree(float timemsec, bool isFoward, int power) {
  if (isFoward) {
    LeftMotor1.spin(directionType::fwd, power, velocityUnits::pct);
    LeftMotor2.spin(directionType::fwd, power, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, power, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, power, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, power, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, power, velocityUnits::pct);
  }
  else {
    LeftMotor1.spin(directionType::rev, power, velocityUnits::pct);
    LeftMotor2.spin(directionType::rev, power, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, power, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, power, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, power, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, power, velocityUnits::pct);
  }
  wait(timemsec, msec);
  LeftMotor1.stop(coast);
  LeftMotor2.stop(coast); 
  LeftMotor3.stop(coast);
  RightMotor1.stop(coast);
  RightMotor2.stop(coast);
  RightMotor3.stop(coast);
};

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  LeftMotor1.setStopping(brake); 
  LeftMotor2.setStopping(brake); 
  LeftMotor3.setStopping(brake);
  RightMotor1.setStopping(brake);
  RightMotor2.setStopping(brake);
  RightMotor3.setStopping(brake);
  OuttakeMotor.setStopping(coast);
  toungue.set(false);
  /////////////////////////////////////left side
  while (Inertial.isCalibrating()) {
    wait(20, msec);
  }
  MoveStraight(31.25, 40, true);
  MoveTurning(-90, 30);
  toungue.set(true);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(200, msec);
  MoveFree(1000, true, 30);
  wait(1000, msec);
  MoveFree(4000, false, 20);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(2500, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(500, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  LeftMotor1.setStopping(coast); 
  LeftMotor2.setStopping(coast); 
  LeftMotor3.setStopping(coast);
  RightMotor1.setStopping(coast);
  RightMotor2.setStopping(coast);
  RightMotor3.setStopping(coast);
  float FBsensitivity = 1.0;
  float LRsensitivity = 0.6;
  bool L1pressed = false;
  bool L2pressed = false;
  bool R2pressed = false;
  int systemState = 0;//0 is at rest, 1 is intaking, 2 is top outtaking, 3 is bottom outtaking
  // User control code here, inside the loop
  while (1) {
    //Driving Control
    //controller dead zone
    int deadzonepct  = 10;
    float Axis3 = Controller1.Axis3.position(percent);
    float Axis1 = Controller1.Axis1.position(percent);
    float Axis3Dead = Axis3 > deadzonepct ? ((Axis3 - deadzonepct)*1.00/(100-deadzonepct))*100 : 
    Axis3Dead = Axis3 < -deadzonepct ? ((Axis3 + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
    float Axis1Dead = Axis1 > deadzonepct ? ((Axis1 - deadzonepct)*1.00/(100-deadzonepct))*100 : 
    Axis1Dead = Axis1 < -deadzonepct ? ((Axis1 + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
    //joystick curve, taking place after deadzoning
    float Axis1Curved = distributeExponentially(Axis1Dead/100.0)*100.0;
    float Axis3Curved = distributeExponentially(Axis3Dead/100.0)*100.0;
    //sensitivity
    Axis1Curved *= LRsensitivity;
    Axis3Curved *= FBsensitivity;
    //set motor powers
    int leftsidepower = (Axis3Curved + Axis1Curved);
    int rightsidepower = (Axis3Curved - Axis1Curved);
    LeftMotor1.spin(directionType::fwd, leftsidepower, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, leftsidepower, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, leftsidepower, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, rightsidepower, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, rightsidepower, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, rightsidepower, velocityUnits::pct);
    
    //toungue action
    if (Controller1.ButtonB.pressing()) {toungue.set(true);}
    else {toungue.set(false);};
    //descoring wings
    if (Controller1.ButtonUp.pressing()) {leftWing.set(true);}
    else {leftWing.set(false);};
    if (Controller1.ButtonX.pressing()) {rightWing.set(true);}
    else {rightWing.set(false);};
    //down outtaking
    if (Controller1.ButtonL1.pressing() && !L1pressed) {
      if (systemState == 3) {systemState=0;}
      else {systemState = 3;}
      L1pressed = true;
    }
    if (!Controller1.ButtonL1.pressing()) {
      L1pressed = false;
    };
    //top outtaking
    if (Controller1.ButtonL2.pressing() && !L2pressed) {
      if (systemState == 2) {systemState=0;}
      else {systemState = 2;}
      L2pressed = true;
    }
    if (!Controller1.ButtonL2.pressing()) {
      L2pressed = false;
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

    if (Controller1.ButtonA.pressing()) {
      systemState = 0;
    }

    switch (systemState) {
      case 3://down outtaking
      IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
      break; 
      case 2://top outtaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      break; 
      case 1://intaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.stop();
      break; 
      case 0:
      default: 
      IntakeMotor.stop();
      OuttakeMotor.stop();
      break;
    }

    

    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(IntakeMotor.temperature(pct));
    Controller1.Screen.setCursor(1, 9);
    Controller1.Screen.print(Inertial.isCalibrating());
    Controller1.Screen.setCursor(1, 17);
    Controller1.Screen.print(OuttakeMotor.temperature(pct));



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(autonomous);//usercontrol

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
