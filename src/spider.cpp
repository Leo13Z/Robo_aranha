#include"spider.hpp"
#include <math.h>
// #define PI 3.14159265



//  - - - - - Defining the size, in cm, of the limb's parts   - - - - - - - - //
#define L1 7.8
#define L2 10
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Defining the Servo Driver's object  - - - - - - - - - - - - - - //
Adafruit_PWMServoDriver pwm;
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Performs the Inverse Cinematic, to determine the distance and-  //
//  - - - - - height of the tip of the foot in relation to the knee servo - - //

void spiderbot::_inverseKinematic(double x,double y, int* a1, int* a2)
{
  double B = sqrt((x*x)+(y*y));
  double alpha1 = acos(y/B);
  double alpha2 = acos(((L2*L2)-(B*B)-(L1*L1))/(-2*B*L1));
  alpha1 = int((alpha1 + alpha2)* (180 / PI));

  alpha2 = int(acos((-(L1*L1)-(L2*L2)+(B*B))/(-2*L2*L1))* (180 / PI));
  alpha2 = int(180 -alpha2 +0);//45

  if ((alpha1>0) && (alpha1<180)){
    *a1=alpha1;
  }
  if ((alpha2>0) && (alpha2<180)){
  *a2=alpha2;
  }
}
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

void spiderbot::_moveJointAngle(joint _joint, int degree)
{
  int internal_degree = degree + _joint.servo_offset;
  if (_joint.bIsInverted)
  {
    internal_degree = 180 - degree + _joint.servo_offset;
  }
    pwm.setPWM(_joint.servo_num,0,(map(internal_degree,0,180,SERVOMIN,SERVOMAX)));
}

void spiderbot::_setLimbPosition(limb _limb, limbPosition desired_position) {
  _moveJointAngle(_limb.joints[0], desired_position.thighAngle);
  _moveJointAngle(_limb.joints[1], desired_position.kneeAngle);
  _moveJointAngle(_limb.joints[2], desired_position.footAngle);
}

void spiderbot::moveLimbPolar(limb _limb, double r, double z, double theta)
{
  // r = radius
  // z = height
  // theta = rotation (at base)
  limbPosition desired;
  _inverseKinematic(r, z, &desired.kneeAngle, &desired.footAngle);
  desired.thighAngle = theta;
  _setLimbPosition(_limb, desired);
}

void spiderbot::setToInitialPosition()
{
  for(int j=0;j<=3;j++){
    moveLimbPolar(robot0.limbs[j], 5, 4, 90);
  }
}

void spiderbot::standUp()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(robot0.limbs[i], 5, 5, 90);
  }
  delay(200);
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(robot0.limbs[i], 5, 15, 90);
  }
}

void spiderbot::riseSlowly()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(robot0.limbs[i], 5, 4, 90);
  }
  delay(200);
  double a;
  for (int j = 400;j<1500;j=j+10){
    for(int i=0; i<=3; i++)
      {
      a=double(j);
    moveLimbPolar(robot0.limbs[i], 5, a/100 , 90);
    }
  }
}

void spiderbot::rest()
{
  // for(int i=0; i<=3; i++)
  // {
  //   moveLimbPolar(robot0.limbs[i], 5, 8, 90);
  // }
  // delay(500);
  // for(int i=0; i<=3; i++)
  // {
  //   moveLimbPolar(robot0.limbs[i], 5, 6, 90);
  // }
  // delay(500);
  double a;
  for (int j = 1500;j>350;j=j-10){
    for(int i=0; i<=3; i++)
      {
      a=double(j);
    moveLimbPolar(robot0.limbs[i], 5, a/100 , 90);
    }
  }
}

void spiderbot::goDoggy()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(robot0.limbs[i], 5, 12, 135);
  }
}


void spiderbot::begin()//inilicializo a placa dos servos
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz update
}
