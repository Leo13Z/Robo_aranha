#include"spider.hpp"



//  - - - - - Defining the size, in cm, of the limb's parts   - - - - - - - - //
#define L1 8
#define L2 10
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Defining the Servo Driver's object  - - - - - - - - - - - - - - //
Adafruit_PWMServoDriver pwm;
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Performs the Inverse Cinematic, to determine the distance and-  //
//  - - - - - height of the tip of the foot in relation to the knee servo - - //
void spiderbot::_inverseKinematic(double x,double y, int* a1, int* a2)
{
  double B=sqrt((x*x)+(y*y));

  double q1=atan2((double)y,(double)x);
  q1=q1*57.2958;

  double q2= acos(((L1*L1)+(B*B)-(L2*L2))/(2*B*L1));
  q2=q2*57.2958;

  double r1,r2;
  r1=q1+q2;
  if ((r1>0) && (r1>180)){
    *a1=int(180-r1);
  }
  r2=acos(((L1*L1)-(B*B)+(L2*L2))/(2*L2*L1));
  r2=(r2)*57.2958;
  if ((r2>0) && (r2>180)){
  *a2=int(r2-45);//125 é  180 devido a  montagem
}
  //testar 135
}
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

void spiderbot::_moveJointAngle(joint _joint, int degree)
{
  int internal_degree = degree + _joint.servo_offset;
  if (_joint.bIsInverted)
  {
    internal_degree = 180 - internal_degree;
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
  _inverseKinematic(z, r, &desired.kneeAngle, &desired.footAngle);
  desired.thighAngle = theta;
  _setLimbPosition(_limb, desired);
}

void spiderbot::setToInitialPosition()
{
  for(int i=0;i<=3;i++)
  {
    moveLimbPolar(body0.limbs[i], 5, 12, 90);
  }
}
void spiderbot::begin()//inilicializo a placa dos servos
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz update
}

// void spiderbot::frente(int junta,int dist,int alt_atual,int amp,int vel)
// {
//   movePolar(junta,dist,alt_atual-3,90-amp);
//   delay(vel);
//   movePolar(junta,dist,alt_atual,90-amp);
//   delay(vel);
//   movePolar(junta,dist,alt_atual,90+amp);
//   delay(vel);
// }
