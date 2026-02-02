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
digital_out ramp = vex::digital_out(ThreeWirePort.B);
digital_out gate = vex::digital_out(ThreeWirePort.F);
digital_out wing = vex::digital_out(ThreeWirePort.C);
digital_out odometryWheels = vex::digital_out(ThreeWirePort.D);
digital_in limitSwitch = vex::digital_in(ThreeWirePort.E);

rotation leftOdometry = rotation(PORT5, true);
rotation rightOdometry = rotation(PORT4, false);
inertial Inertial = inertial(PORT17);

void resetMotorEncoders(void) {
  LeftMotor1.resetPosition(); 
  LeftMotor2.resetPosition(); 
  LeftMotor3.resetPosition();
  RightMotor1.resetPosition();
  RightMotor2.resetPosition();
  RightMotor3.resetPosition();
  leftOdometry.resetPosition();
  rightOdometry.resetPosition();
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
float distributeExponentially (float input, float tuning = 1.001) {//increasing tuning makes curve steeper
  return (input > 0 ? 
  (std::pow(tuning, 100*input)-1)/(std::pow(tuning, 100)-1):
  -(std::pow(tuning, std::abs(100*input))-1)/(std::pow(tuning, 100)-1)
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
  float tuningConstant = 0.955;//tuning modifier for distance  
  float gearingConstant = 1;//gear ratio
  double toleranceDeg = 1;//PID tolerance, in degrees
  double increment = 0.25;//PID increment

  float distanceRev = ((distance/6.28318)/gearingConstant);
  distanceRev *= tuningConstant;
  if (fowards) {
    while (leftOdometry.position(rev) < distanceRev or rightOdometry.position(rev) < distanceRev /*||
    (rightOdometry.position(deg) - leftOdometry.position(deg)) < -toleranceDeg ||
    (leftOdometry.position(deg) - rightOdometry.position(deg)) < -toleranceDeg*/
    ) {
    
    
    float distanceTraveledPctLeft = (leftOdometry.position(rev)/distanceRev)*100.0;
    float distanceTraveledPctRight = (rightOdometry.position(rev)/distanceRev)*100.0;
    float distanceTraveledPctAvg = (distanceTraveledPctLeft+distanceTraveledPctRight)/2;
    float distributedSpeed = distributeParabolically(distanceTraveledPctAvg/100.0)*100.0;
    float cappedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 15.0);
    float ajustedSpeedLeft = cappedSpeed;
    float ajustedSpeedRight = cappedSpeed;
    

    if ((leftOdometry.position(deg) - rightOdometry.position(deg)) < -toleranceDeg) {
      if (ajustedSpeedLeft >= cappedSpeed) {
        ajustedSpeedRight = ajustedSpeedRight - increment;
      }
      else {
        ajustedSpeedLeft = ajustedSpeedLeft + increment;
      }
    }
    else if ((rightOdometry.position(deg) - leftOdometry.position(deg)) < -toleranceDeg) {
      if (ajustedSpeedRight >= cappedSpeed) {
        ajustedSpeedLeft = ajustedSpeedLeft - increment;
      }
      else {
        ajustedSpeedRight = ajustedSpeedRight + increment;
      }
    }
    ajustedSpeedRight = std::min(ajustedSpeedRight, cappedSpeed);
    ajustedSpeedLeft = std::min(ajustedSpeedLeft, cappedSpeed);
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("leftOdometry %.2f  ", leftOdometry.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("rightOdometry %.2f  ", rightOdometry.position(rev));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("distanceTraveledPctAvg = %.2f    ", distanceTraveledPctAvg);
    if (abs(distanceTraveledPctAvg - 50) < 0.1) {
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("mid left = %.2f    ", leftOdometry.position(rev));
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("mid right = %.2f    ", rightOdometry.position(rev));
    }

    LeftMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
    if (leftOdometry.position(rev) > distanceRev || rightOdometry.position(rev) > distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }
    /*if (leftOdometry.position(rev) > distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
    }
    if (rightOdometry.position(rev) > distanceRev) {
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }*/
  }
  }
  else if (!fowards) {
    while (leftOdometry.position(rev) > -distanceRev or rightOdometry.position(rev) > -distanceRev ) {
    float distanceTraveledPctLeft = (leftOdometry.position(rev)/distanceRev)*100.0;
    float distanceTraveledPctRight = (rightOdometry.position(rev)/distanceRev)*100.0;
    float distanceTraveledPctAvg = (distanceTraveledPctLeft+distanceTraveledPctRight)/2;
    float distributedSpeed = distributeParabolically(-distanceTraveledPctAvg/100.0)*100.0;
    float cappedSpeed = std::max((distributedSpeed * (maxSpeed/100.0)), 15.0);
    float ajustedSpeedLeft = cappedSpeed;
    float ajustedSpeedRight = cappedSpeed;
    
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("leftOdometry %.2f  ", leftOdometry.position(rev));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("rightOdometry %.2f  ", rightOdometry.position(rev));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("distanceTraveledPctAvg = %.2f    ", distanceTraveledPctAvg);

    LeftMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
    LeftMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
    RightMotor1.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
    RightMotor2.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
    RightMotor3.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
    if (leftOdometry.position(rev) < -distanceRev || rightOdometry.position(rev) < -distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }
    };
  }
}
//distance, maxSpeed, fowards
void MoveStraightOld(float distance, int maxSpeed, bool fowards) {
  resetMotorEncoders();
  //wheels are 2 inches in diamametere, times pi means curcumernce is 7.853975 in
  //divided by 360 to get the inces per degree (0.0349065556)
  //times 1.1669779538 for the gearing
  //gives us the final multiplier of 0.0139626222 
  /*float distancedeg = (distance*0.0349065556)*1.1669779538;
  Brain.Screen.setCursor(4, 1);
  Brain.Screen.print("dist in deg = %.2f  ", (distance*0.0349065556)*1.1669779538);*/
  //alright screw it it's revolution time
  //distance (in inches) divided by wheel curcumfrence mutiplied by gear raito
  //TODO: switch this to pid for greater accuracy.
  float gearningConstant = 1.1669779538;
  float distanceRev = (distance/7.853975)*gearningConstant;
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
  //odometry wheels are 2 inches in diameter, times pi means curcumernce is 6.28318 in
  //with 1:1 gearing
  //gives us the final multiplier of 6.28318 inches per revolution
  //TODO: switch this to pid for greater accuracy.
  float tuningConstant = 0.99;
  float gearingConstant = 1.00;
  float distanceRev = ((distance/6.28318)/gearingConstant);
  distanceRev *= tuningConstant;
  float leftSideMultiplier = 1.00;
  float rightSideMultiplier = 1.00;
    if (LeftToRightRatio > 1.00) {
      if (1/LeftToRightRatio <= 0.00001) {rightSideMultiplier = 0;}
      else {rightSideMultiplier = 1/LeftToRightRatio;}
    }
    else if (LeftToRightRatio < 1.00) {
      leftSideMultiplier = LeftToRightRatio;
    };
    float distanceRevRight = distanceRev*rightSideMultiplier;
    float distanceRevLeft = distanceRev*leftSideMultiplier;

    

    if (fowards) {
      while (leftOdometry.position(rev) < distanceRevLeft or rightOdometry.position(rev) < distanceRevRight) {
      float distanceTraveledPctLeft = (leftOdometry.position(rev)/distanceRevLeft)*100.0;
      float distributedSpeedLeft = distributeParabolically(distanceTraveledPctLeft/100.0)*100.0;
      float ajustedSpeedLeft = std::max((distributedSpeedLeft * ((maxSpeed*leftSideMultiplier)/100.0)), 10.0);

      float distanceTraveledPctRight = (rightOdometry.position(rev)/distanceRevRight)*100.0;
      float distributedSpeedRight = distributeParabolically(distanceTraveledPctRight/100.0)*100.0;
      float ajustedSpeedRight = std::max((distributedSpeedRight * ((maxSpeed*rightSideMultiplier)/100.0)), 10.0);
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("leftOdometry %.2f  ", leftOdometry.position(rev));
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("distanceTraveledPctLeft = %.2f    ", distanceTraveledPctLeft);
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("distributedSpeedLeft %.2f  ", distributedSpeedLeft);
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
      Brain.Screen.setCursor(6, 1);
      Brain.Screen.print("rightOdometry %.2f  ", rightOdometry.position(rev));
      Brain.Screen.setCursor(7, 1);
      Brain.Screen.print("distanceTraveledPctRight = %.2f    ", distanceTraveledPctRight);
      Brain.Screen.setCursor(8, 1);
      Brain.Screen.print("distributedSpeedRight %.2f  ", distributedSpeedRight);
      Brain.Screen.setCursor(9, 1);
      Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);
      ajustedSpeedLeft = 30*1.5;
      ajustedSpeedRight = 30;
      LeftMotor1.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor2.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor3.spin(directionType::fwd, ajustedSpeedLeft, velocityUnits::pct);
      RightMotor1.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      RightMotor2.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      RightMotor3.spin(directionType::fwd, ajustedSpeedRight, velocityUnits::pct);
      if (leftOdometry.position(rev) > distanceRevLeft) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
      }
      if (rightOdometry.position(rev) > distanceRevRight) {
        RightMotor1.stop(); 
        RightMotor2.stop(); 
        RightMotor3.stop();
      }
      };
    }
    else if (!fowards) {
      while (leftOdometry.position(rev) > -distanceRevLeft or rightOdometry.position(rev) > -distanceRevRight ) {
      float distanceTraveledPctLeft = (leftOdometry.position(rev)/distanceRevLeft)*100.0;
      float distributedSpeedLeft = distributeParabolically(-distanceTraveledPctLeft/100.0)*100.0;
      float ajustedSpeedLeft = std::max((distributedSpeedLeft * ((maxSpeed*leftSideMultiplier)/100.0)), 10.0);
      
      float distanceTraveledPctRight = (rightOdometry.position(rev)/distanceRevRight)*100.0;
      float distributedSpeedRight = distributeParabolically(-distanceTraveledPctRight/100.0)*100.0;
      float ajustedSpeedRight = std::max((distributedSpeedRight * ((maxSpeed*rightSideMultiplier)/100.0)), 10.0);
      
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("distanceRev = %.2f    ", distanceRev);
      Brain.Screen.setCursor(2, 1);
      Brain.Screen.print("leftOdometry %.2f  ", leftOdometry.position(rev));
      Brain.Screen.setCursor(3, 1);
      Brain.Screen.print("distanceTraveledPctLeft = %.2f    ", distanceTraveledPctLeft);
      Brain.Screen.setCursor(4, 1);
      Brain.Screen.print("distributedSpeedLeft %.2f  ", distributedSpeedLeft);
      Brain.Screen.setCursor(5, 1);
      Brain.Screen.print("ajustedSpeedLeft = %.2f    ", ajustedSpeedLeft);
      Brain.Screen.setCursor(6, 1);
      Brain.Screen.print("rightOdometry %.2f  ", rightOdometry.position(rev));
      Brain.Screen.setCursor(7, 1);
      Brain.Screen.print("distanceTraveledPctRight = %.2f    ", distanceTraveledPctRight);
      Brain.Screen.setCursor(8, 1);
      Brain.Screen.print("distributedSpeedRight %.2f  ", distributedSpeedRight);
      Brain.Screen.setCursor(9, 1);
      Brain.Screen.print("ajustedSpeedRight = %.2f    ", ajustedSpeedRight);
      ajustedSpeedLeft = 30;
      ajustedSpeedRight = 30;
      LeftMotor1.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor2.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct); 
      LeftMotor3.spin(directionType::rev, ajustedSpeedLeft, velocityUnits::pct);
      RightMotor1.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      RightMotor2.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      RightMotor3.spin(directionType::rev, ajustedSpeedRight, velocityUnits::pct);
      if (leftOdometry.position(rev) < -distanceRevLeft) {
        LeftMotor1.stop(); 
        LeftMotor2.stop(); 
        LeftMotor3.stop();
      }
      if (rightOdometry.position(rev) < -distanceRevRight) {
        RightMotor1.stop(); 
        RightMotor2.stop(); 
        RightMotor3.stop();
      }
      };
    }
};
void TurnWithRatioOld(float distance, int maxSpeed, double LeftToRightRatio, bool fowards) {
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
//////////////////////////////////////////////////////left side auto, 
void LeftAuto(void) {
  MoveStraight(33, 65, true);//move to the intake//31.25
  toungue.set(true);
  ramp.set(true);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  MoveTurning(-90, 25);//turn towards it
  wait(100, msec);
  MoveFree(900, true, 35);//move in to it
  wait(100, msec);
  MoveStraight(25, 40, false);
  MoveFree(300, false, 30);
  //MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  ramp.set(false);

  
  //*/
};
void LeftAutoExpirimental(void) {
  //temp inverted because of the half field
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  toungue.set(false);
  ramp.set(false);
  MoveStraight(33, 70, true);//move in to and intake the center cluster 
  toungue.set(true);
  MoveTurning(-85, 30);//turn towards the middle goal
  wait(300, msec);//wait to finish turning
  MoveStraight(13.5, 40, false);//move to it
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtaking to the middle
  wait(2000, msec);//wait to finish outtaking
  OuttakeMotor.stop();
  //TurnWithRatio(48, 60, 1.5, true);//move to alignment with the chute
  

  float distanceRev = 5.9;
  float maxSpeed = 70;
  float rightMult = 1.04;
  while (leftOdometry.position(rev) < distanceRev or rightOdometry.position(rev) < distanceRev) {
    float distanceTraveledPctLeft = (leftOdometry.position(rev)/distanceRev)*100.0;
    float distributedSpeedLeft = distributeParabolically(distanceTraveledPctLeft/100.0)*100.0;
    float ajustedSpeedLeft = std::max((distributedSpeedLeft * (maxSpeed/100.0)), 10.0);

    float distanceTraveledPctRight = (rightOdometry.position(rev)/(distanceRev*rightMult))*100.0;
    float distributedSpeedRight = distributeParabolically(distanceTraveledPctRight/100.0)*100.0;
    float ajustedSpeedRight = std::max((distributedSpeedRight * (maxSpeed/100.0)), 10.0);


    float fixedSpeed = 30; 
    LeftMotor1.spin(directionType::fwd, fixedSpeed, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, fixedSpeed, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, fixedSpeed, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, fixedSpeed*rightMult, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, fixedSpeed*rightMult, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, fixedSpeed*rightMult, velocityUnits::pct);
    if (leftOdometry.position(rev) > distanceRev) {
      LeftMotor1.stop(); 
      LeftMotor2.stop(); 
      LeftMotor3.stop();
    }
    if (rightOdometry.position(rev) > distanceRev*rightMult) {
      RightMotor1.stop(); 
      RightMotor2.stop(); 
      RightMotor3.stop();
    }
    };

  //MoveStraight(48, 60, true); //move to alignment with the chute
  MoveTurning(-45, 30);//turn towards it
  wait(100, msec);//wait to finish turning
  toungue.set(true);//put tounge out
  ramp.set(true);
  MoveFree(1000, true, 50);//move in to it
  MoveFree(1000, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (2000, msec);//wait till all the balls are scored
  //*/
};
//////////////////////////////////////////////////////right side auto, 
void RightAuto(void) {
  MoveStraight(33, 65, true);//move to the intake//31.25
  toungue.set(true);
  ramp.set(true);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  MoveTurning(90, 25);//turn towards it
  wait(100, msec);
  MoveFree(900, true, 35);//move in to it
  wait(100, msec);
  MoveStraight(25, 40, false);
  MoveFree(300, false, 30);
  //MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (1000, msec);//wait till all the balls are scored
  ramp.set(false);





  //expirimental auto
  /*MoveStraight(20.5, 40, true);//back out
  OuttakeMotor.stop();
  toungue.set(false);
  MoveTurning(135, 30);//turn towards the group of 3
  MoveStraight(53, 70, true);//run in to and intake them
  toungue.set(true);
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  //*/
};
//////////////////////////////////////////////////////skills auto, 
void SkillsAutoSafe(void) {
  MoveStraight(33, 40, true);//move to the intake
  toungue.set(true);
  ramp.set(true);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  MoveTurning(90, 25);//turn towards it
  wait(100, msec);
  //for (int i = 0; i < 1; i++) {
  MoveFree(1000, true, 45);//move in to it
  OuttakeMotor.spin(directionType::fwd, 20, velocityUnits::pct);
  MoveFree(250, true, 60);//move in to it
  wait(500, msec);
  MoveFree(250, true, 60);//move in to it
  wait(500, msec);
  MoveFree(250, true, 60);//move in to it
  OuttakeMotor.stop();
  wait(1000, msec);
  ramp.set(true);
  MoveStraight(25, 40, false);
  MoveFree(500, false, 30);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(1000, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(2500, msec);//wait till all the balls are scored
  ramp.set(false);
  wait(500, msec);
  //}
  MoveStraight(20, 40, true);
  OuttakeMotor.stop();
  MoveTurning(90, 30);
  MoveFree(2000, false, 30);
  MoveStraight(13, 30, true);
  toungue.set(false);
  MoveTurning(-25, 20);
  MoveStraight(10, 30, true);
  odometryWheels.set(false);//leap of faith
  MoveFree(800, true, 45);
  toungue.set(true);
  MoveFree(1100, true, 30);
  toungue.set(false);
  MoveFree(800, true, 30);
  LeftMotor1.setStopping(coast); 
  LeftMotor2.setStopping(coast); 
  LeftMotor3.setStopping(coast);
  RightMotor1.setStopping(coast);
  RightMotor2.setStopping(coast);
  RightMotor3.setStopping(coast);
};
void SkillsAuto(void) {
  MoveStraight(32, 40, true);//move to the intake
  toungue.set(true);
  ramp.set(true);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  MoveTurning(90, 30);//turn towards it
  wait(100, msec);
  MoveFree(1000, true, 45);//move in to it
  OuttakeMotor.spin(directionType::fwd, 30, velocityUnits::pct);
  MoveFree(250, true, 60);//move in to it
  wait(200, msec);
  MoveFree(250, true, 60);//move in to it
  wait(200, msec);
  MoveFree(250, true, 60);//move in to it
  OuttakeMotor.stop();
  wait(700, msec);
  MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(300, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(1000, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(300, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (2000, msec);//wait till all the balls are scored
  toungue.set(false);
  //move to the other side and align
  MoveStraight(13, 40, true);
  OuttakeMotor.stop();
  MoveTurning(90, 30);
  MoveFree(2000, false, 30);
  MoveStraight(110, 100, true);//move to the other side
  //MoveTurning(180, 30);
  MoveFree(1300, true, 40);//touch the wall
  MoveStraight(14.5, 30, false);//back out inline with the goal and chute
  MoveTurning(-90, 30);
  toungue.set(true);
  //otherside code
  wait(100, msec);
  MoveFree(1500, true, 45);//move in to it
  wait(700, msec);
  MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(300, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(1000, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(300, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (2000, msec);//wait till all the balls are scored
  MoveStraight(10, 30, true);
  toungue.set(false);
  MoveTurning(-45, 30);
  odometryWheels.set(false);//leap of faith
  MoveFree(4000, true, 30);
  //expirimental auto
  /*MoveStraight(21, 55, true);//back out
  OuttakeMotor.stop();
  toungue.set(false);
  MoveTurning(-135, 40);//turn towards the group of 3
  MoveStraight(34, 40, true);//run in to and intake them
  MoveTurning(-180, 50);//turn around
  toungue.set(true);//put the tounge down to make sure they don't get away
  ramp.set(false);//switch to middle level scoring
  MoveStraight(14, 40, false);//move in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);*/
};
void SkillsAuto2(void) {
  MoveStraight(31.3, 65, true);//move to the intake
  toungue.set(true);
  ramp.set(true);
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  MoveTurning(90, 30);//turn towards it
  wait(100, msec);
  MoveFree(1000, true, 45);//move in to it
  OuttakeMotor.spin(directionType::fwd, 30, velocityUnits::pct);
  MoveFree(500, true, 45);//move in to it
  OuttakeMotor.stop();
  wait(700, msec);
  /*MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (2000, msec);//wait till all the balls are scored
  //move to the other side and align
  MoveStraight(16, 40, true);
  OuttakeMotor.stop();*/
  toungue.set(false);//instead of scoring, them simply move out and begin moving to the otherside
  MoveStraight(15, 40, false);
  MoveTurning(-90, 30);
  MoveFree(2000, true, 30);

  //divergence point <-- wrong

  MoveStraight(4, 20, false);//move away from the wall
  MoveTurning(-90, 30);//turn totheotherside
  MoveStraight(100, 100, true);//move to the othe side
  MoveFree(1000, true, 50);
  MoveStraight(16, 30, false);
  MoveTurning(-90, 30);
  MoveStraight(15, 30, true);
  MoveTurning(90, 30);

  MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait (3000, msec);//wait till all the balls are scored
  OuttakeMotor.stop();

  MoveFree(1200, true, 45);//move in to the chute
  OuttakeMotor.spin(directionType::fwd, 30, velocityUnits::pct);
  MoveFree(500, true, 45);//move in to it
  OuttakeMotor.stop();
  wait(700, msec);
  MoveFree(1800, false, 50);//move directly backwards in to the goal
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//outtake
  wait(800, msec);
  OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);//brienfly reverse to unstick stuck balls
  IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
  wait(200, msec);
  OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);//go back to outtaking
  IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
  wait(2000, msec);//wait till all the balls are scored
  toungue.set(false);



};
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
  odometryWheels.set(true);
  while (Inertial.isCalibrating()) {
    wait(20, msec);
  }
  ////the all important
  LeftAuto();
  
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

void motorTesting(void) {
  LeftMotor1.setStopping(coast); 
  while (1) {

    if (Controller1.ButtonLeft.pressing()) {LeftMotor1.spin(directionType::fwd, 5, velocityUnits::pct);}
    if (Controller1.ButtonRight.pressing()) {LeftMotor1.spin(directionType::rev, 5, velocityUnits::pct);}
    if (Controller1.ButtonA.pressing()) {LeftMotor1.stop();}

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("temp(F) = %.2f    ", LeftMotor1.temperature(fahrenheit));
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("temp(pct) = %.2f    ", LeftMotor1.temperature(pct));
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("torque(Nm) = %.2f    ", LeftMotor1.torque(Nm));
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("watt(watt) = %.2f    ", LeftMotor1.power(watt));
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("current(amp) = %.2f    ", LeftMotor1.current(amp));
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("voltage(volt) = %.2f    ", LeftMotor1.voltage(volt));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("efficiency(pct) = %.2f    ", LeftMotor1.efficiency(pct));



  }


}

void usercontrolChris(void) {
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
  float FBsensitivity = 1.0;
  float LRsensitivity = 0.4;
  float PIDIncrement = 0.25;
  float PIDTolerancePct = 5;
  
  float LeftSidePower = 0.0;
  float RightSidePower = 0.0;
  bool L1pressed = false;
  bool L2pressed = false;
  bool R1pressed = false;
  bool R2pressed = false;
  bool Bpressed = false;
  bool Downpressed = false;

  int systemState = 1;//0 is at rest, 1 is intaking, 2 is top outtaking, 3 is bottom outtaking
  int timer1 = 0;
  // User control code here, inside the loop
  while (1) {
    //Driving Control
    //controller dead zone
    int deadzonepct  = 10;
    float Axis3 = Controller1.Axis3.position(percent);// left/right
    float Axis1 = Controller1.Axis1.position(percent);// fwd/back
    float Axis3Dead = Axis3 > deadzonepct ? ((Axis3 - deadzonepct)*1.00/(100-deadzonepct))*100 : 
    Axis3Dead = Axis3 < -deadzonepct ? ((Axis3 + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
    float Axis1Dead = Axis1 > deadzonepct ? ((Axis1 - deadzonepct)*1.00/(100-deadzonepct))*100 : 
    Axis1Dead = Axis1 < -deadzonepct ? ((Axis1 + deadzonepct)*1.00/(100-deadzonepct))*100 : 0;
    //joystick curve, taking place after deadzoning
    float Axis1Curved = distributeExponentially(Axis1Dead/100.0, 1.025)*100.0;
    float Axis3Curved = distributeExponentially(Axis3Dead/100.0, 1.025)*100.0;
    //sensitivity
    Axis1Curved *= LRsensitivity;
    Axis3Curved *= FBsensitivity;
    //set motor powers
    LeftSidePower = (Axis3Curved + Axis1Curved);
    RightSidePower = (Axis3Curved - Axis1Curved);
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
    
    LeftMotor1.spin(directionType::fwd, LeftSidePower, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, LeftSidePower, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, LeftSidePower, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, RightSidePower, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, RightSidePower, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, RightSidePower, velocityUnits::pct);
    

    if (timer1 >= 0) {timer1 -= 1;};
    
    
    
    //top outtaking
    if (Controller1.ButtonL1.pressing() && !L1pressed) {
      /*if (systemState == 2) {systemState = 0;}
      //else {systemState = 2; toungue.set(true);}
      //brief backtake to loosen balls
      else {systemState = 3; timer1 = 2;}
      */
      systemState = 3; timer1 = 1;
      L1pressed = true;
    }
    if (!Controller1.ButtonL1.pressing()) {
      L1pressed = false;
    };
    //down outtaking
    if (Controller1.ButtonL2.pressing() && !L2pressed) {
      if (systemState == 3) {systemState=0;}
      else {systemState = 3;}
      L2pressed = true;
    }
    if (!Controller1.ButtonL2.pressing()) {
      L2pressed = false;
    };
    //brief backtake to loosen balls
    if (timer1 == 0) {systemState = 2;}


    //intaking
    if (Controller1.ButtonR2.pressing() && !R2pressed) {
      if (systemState == 1) {systemState=0;}
      else {systemState = 1;}
      R2pressed = true;
    }
    if (!Controller1.ButtonR2.pressing()) {
      R2pressed = false;
    };
    
    //descoring wing
    if (Controller1.ButtonB.pressing()) {wing.set(true);}
    else {wing.set(false);};
    
    //outtaking level, true (top level) by default 
    if (Controller1.ButtonDown.pressing()) {ramp.set(false);}
    else {ramp.set(true);};

    //toungue
    if (Controller1.ButtonR1.pressing() && !R1pressed) {
      if (toungue.value()) {toungue.set(false);}
      else {toungue.set(true);}
      R1pressed = true;
    }
    if (!Controller1.ButtonR1.pressing()) {
      R1pressed = false;
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
    Controller1.Screen.print(avgDriveHeat);
    Controller1.Screen.setCursor(1, 17);
    Controller1.Screen.print(OuttakeMotor.temperature(pct));



    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
void usercontrolElliot(void) {
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
    /*if (Controller1.ButtonDown.pressing() && !Downpressed) {
      if (wing.value()) {wing.set(false);}
      else {wing.set(true);}
      Downpressed = true;
    }
    if (!Controller1.ButtonDown.pressing()) {
      Downpressed = false;
    };*/
    if (Controller1.ButtonDown.pressing()) {
        wing.set(false);
    }
    else {wing.set(true);}
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
      OuttakeMotor.spin(directionType::fwd, 15, velocityUnits::pct);
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

void usercontrol(void) {
  enum driver {elliot, chris, collin};
  enum driver Driver = elliot;


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

  float FBsensitivity = 1.0;
  float LRsensitivity = 1.0;
  float curveConstant = 1.025;

  if (Driver == elliot) {
  FBsensitivity = 1.0;
  LRsensitivity = 0.6;
  curveConstant = 1.025;
  }
  else if (Driver == chris) 
  {
  FBsensitivity = 1.0;
  LRsensitivity = 0.4;
  curveConstant = 1.001;
  }

  float PIDIncrement = 0.25;
  float PIDTolerancePct = 5;
  float LeftSidePower = 0.0;
  float RightSidePower = 0.0;
  bool L1pressed = false;
  bool L2pressed = false;
  bool R1pressed = false;
  bool R2pressed = false;
  bool Bpressed = false;
  bool Downpressed = false;

  int systemState = 1;//0 is at rest, 1 is intaking, 2 is top outtaking, 3 is bottom outtaking
  int timer1 = 0;
  toungue.set(true);
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
    float Axis1Curved = distributeExponentially(Axis1Dead/100.0, 1.025)*100.0;
    float Axis3Curved = distributeExponentially(Axis3Dead/100.0, 1.025)*100.0;
    //sensitivity
    Axis1Curved *= LRsensitivity;
    Axis3Curved *= FBsensitivity;
    //set motor powers
    LeftSidePower = (Axis3Curved + Axis1Curved);
    RightSidePower = (Axis3Curved - Axis1Curved);
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

    LeftMotor1.spin(directionType::fwd, LeftSidePower, velocityUnits::pct); 
    LeftMotor2.spin(directionType::fwd, LeftSidePower, velocityUnits::pct); 
    LeftMotor3.spin(directionType::fwd, LeftSidePower, velocityUnits::pct);
    RightMotor1.spin(directionType::fwd, RightSidePower, velocityUnits::pct);
    RightMotor2.spin(directionType::fwd, RightSidePower, velocityUnits::pct);
    RightMotor3.spin(directionType::fwd, RightSidePower, velocityUnits::pct);
    

    if (timer1 >= 0) {timer1 -= 1;};
    
    vex::controller::button buttonWing = Controller1.ButtonDown;
    vex::controller::button buttonToungue = Controller1.ButtonDown;

    
    //descoring wing
    if (Controller1.ButtonDown.pressing() && !Downpressed) {
      if (wing.value()) {wing.set(false);}
      else {wing.set(true);}
      Downpressed = true;
    }
    if (!Controller1.ButtonDown.pressing()) {
      Downpressed = false;
    };
    //toungue
    if (Controller1.ButtonB.pressing() && !Bpressed) {
      if (toungue.value()) {toungue.set(false);}
      else {toungue.set(true);}
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
      else {systemState = 2;}
      */
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
      IntakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::rev, 100, velocityUnits::pct);
      break; 
      case 3://middle outtaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      ramp.set(false);
      break; 
      case 2://top outtaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      OuttakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      ramp.set(true);
      break; 
      case 1://intaking
      IntakeMotor.spin(directionType::fwd, 100, velocityUnits::pct);
      if (limitSwitch) {OuttakeMotor.spin(directionType::fwd, 10, velocityUnits::pct);}
      else {}
      ramp.set(false);
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
    Controller1.Screen.print(avgDriveHeat);
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
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrolElliot);//usercontrol

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
