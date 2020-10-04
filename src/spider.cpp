#include"spider.hpp"
#include <math.h>
// #define PI 3.14159265



//  - - - - - Defining the size, in cm, of the limb's parts   - - - - - - - - //
#define L1 8
#define L2 10
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Defining the Servo Driver's object  - - - - - - - - - - - - - - //
Adafruit_PWMServoDriver pwm;
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


Point::Point() {}

Point::Point(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

float Point::GetDistance(Point point1, Point point2)
{
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}


RobotLegsPoints::RobotLegsPoints() {}

RobotLegsPoints::RobotLegsPoints(Point leg1, Point leg2, Point leg3, Point leg4)
{
  this->leg1 = leg1;
  this->leg2 = leg2;
  this->leg3 = leg3;
  this->leg4 = leg4;
}


RobotJoint::RobotJoint() {}

void RobotJoint::Set(int servoNum, float offset, bool jointDir, float jointMinAngle, float jointMaxAngle)
{
  // According to current assembly:
  //servoNum = the slot in the PCA9685 the servo is connected
  //jointDir = some servos are asembled inverted. This indicates that.
  //jointZero = 90
  //jointMinAngle = 0 --- may be changed
  //jointMaxAngle = 180 --- may be changed
  //offset = due to assembly, the joint must have minor angular corrections
  this->servoNum = servoNum;
  this->offset = offset;
  this->jointDir = jointDir;
  this->jointMinAngle = jointMinAngle;
  this->jointMaxAngle = jointMaxAngle;
}

void RobotJoint::RotateToDirectly(float jointAngle)
{
  if (!CheckJointAngle(jointAngle))
    return;

  float servoAngle;

  servoAngle = (jointDir ? jointAngle : 180 - jointAngle) + offset;

  pwm.setPWM(servoNum,0,(map(servoAngle,0,180,SERVOMIN,SERVOMAX)));

  jointAngleNow = jointAngle;
  servoAngleNow = servoAngle;
}

float RobotJoint::GetJointAngle(float servoAngle)
{
  return (servoAngle);
}

bool RobotJoint::CheckJointAngle(float jointAngle)
{

  if (jointAngle >= jointMinAngle && jointAngle <= jointMaxAngle)
    return true;
  else
    return false;
}


RobotLeg::RobotLeg() {}

void RobotLeg::Set(float xOrigin, float yOrigin)
{
  this->xOrigin = xOrigin;
  this->yOrigin = yOrigin;
}


void RobotLeg::CalculatePoint(float alpha, float beta, float gamma, volatile float &x, volatile float &y, volatile float &z)
{
  // transform angle to radian
  alpha = alpha * PI / 180;
  beta = beta * PI / 180;
  gamma = gamma * PI / 180;
  // calculate u-v coordinate
  float u, v;
  u = RobotShape::d + RobotShape::e * sin(beta) + RobotShape::f * sin(gamma - beta);
  v = RobotShape::c + RobotShape::e * cos(beta) - RobotShape::f * cos(gamma - beta);
  // calculate x-y-z coordinate
  x = xOrigin + u * cos(alpha);
  y = yOrigin + u * sin(alpha);
  z = v;
}

void RobotLeg::CalculatePoint(float alpha, float beta, float gamma, Point &point)
{
  CalculatePoint(alpha, beta, gamma, point.x, point.y, point.z);
}

void RobotLeg::CalculateAngle(float x, float y, float z, float &alpha, float &beta, float &gamma)
{
  // calculate u-v angle
  float u, v;
  u = sqrt(pow(x - xOrigin, 2) + pow(y - yOrigin, 2));
  v = z;
  beta = PI / 2 - acos((pow(RobotShape::e, 2) + (pow(u - RobotShape::d, 2) + pow(v - RobotShape::c, 2)) - pow(RobotShape::f, 2)) / (2 * RobotShape::e * sqrt(pow(u - RobotShape::d, 2) + pow(v - RobotShape::c, 2)))) - atan2(v - RobotShape::c, u - RobotShape::d);
  gamma = acos((pow(RobotShape::e, 2) + pow(RobotShape::f, 2) - (pow(u - RobotShape::d, 2) + pow(v - RobotShape::c, 2))) / (2 * RobotShape::e * RobotShape::f));
  // calculate x-y-z angle
  alpha = atan2(y - yOrigin, x - xOrigin);
  if (xOrigin < 0 && yOrigin < 0)
    alpha = alpha + PI;
  if (xOrigin < 0 && yOrigin < 0)
    alpha = alpha + PI;
  // transform radian to angle
  alpha = alpha * 180 / PI;
  beta = beta * 180 / PI;
  gamma = gamma * 180 / PI;
}

void RobotLeg::CalculateAngle(Point point, float &alpha, float &beta, float &gamma)
{
  CalculateAngle(point.x, point.y, point.z, alpha, beta, gamma);
}

bool RobotLeg::CheckPoint(Point point)
{
  float alpha, beta, gamma;
  CalculateAngle(point, alpha, beta, gamma);
  if (CheckAngle(alpha, beta, gamma))
  {
    Point pointNew;
    CalculatePoint(alpha, beta, gamma, pointNew);
    if (Point::GetDistance(point, pointNew) < negligibleDistance)
      return true;
  }
  return false;
}

bool RobotLeg::CheckAngle(float alpha, float beta, float gamma)
{
  if (jointA.CheckJointAngle(alpha) && jointB.CheckJointAngle(beta) && jointC.CheckJointAngle(gamma))
    return true;
  else
    return false;
}

void RobotLeg::MoveTo(Point point)
{
  pointGoal = point;
  isBusy = true;
}

void RobotLeg::MoveToRelatively(Point point)
{
  point = Point(pointGoal.x + point.x, pointGoal.y + point.y, pointGoal.z + point.z);
  MoveTo(point);
}

void RobotLeg::WaitUntilFree()
{
  while (isBusy);
}

void RobotLeg::ServosRotateTo(float degreeA, float degreeB, float degreeC)
{
  float alpha = jointA.GetJointAngle(degreeA);
  float beta = jointB.GetJointAngle(degreeB);
  float gamma = jointC.GetJointAngle(degreeC);

  Point point;
  CalculatePoint(alpha, beta, gamma, point);

  MoveTo(point);
}

void RobotLeg::MoveToDirectly(Point point)
{
  float alpha, beta, gamma;
  CalculateAngle(point, alpha, beta, gamma);
  RotateToDirectly(alpha, beta, gamma);
}

void RobotLeg::MoveToDirectlyRelatively(Point point)
{
  point = Point(pointGoal.x + point.x, pointGoal.y + point.y, pointGoal.z + point.z);
  MoveToDirectly(point);
}

void RobotLeg::RotateToDirectly(float alpha, float beta, float gamma)
{
  jointC.RotateToDirectly(gamma);
  jointB.RotateToDirectly(beta);
  jointA.RotateToDirectly(alpha);

  Point point;
  CalculatePoint(alpha, beta, gamma, point);

  if (isFirstMove)
  {
    isFirstMove = false;
    pointGoal = point;
  }

  pointNow = point;
}







//  - - - - - Performs the Inverse Cinematic, to determine the distance and-  //
//  - - - - - height of the tip of the foot in relation to the knee servo - - //
void spiderbot::_inverseKinematic(double x,double y, float* a1, float* a2)
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

void spiderbot::_setLimbPosition(limb* _limb, float* desired_position) {
  Serial.print("_limb->joints[0]: ");
  // Serial.println(_limb.joints[0]);
  _moveJointAngle(_limb->joints[0], desired_position[0]);
  Serial.print("_limb->joints[1]: ");
  // Serial.println(_limb->joints[1]);
  _moveJointAngle(_limb->joints[1], desired_position[1]);
  Serial.print("_limb->joints[2]: ");
  // Serial.println(_limb->joints[2]);
  _moveJointAngle(_limb->joints[2], desired_position[2]);
  _limb->angles[0] = desired_position[0];
  Serial.print("_limb->angles[0]: ");
  Serial.println(_limb->angles[0]);
  _limb->angles[1] = desired_position[1];
  Serial.print("_limb->angles[1]: ");
  Serial.println(_limb->angles[1]);
  _limb->angles[2] = desired_position[2];
  Serial.print("_limb->angles[2]: ");
  Serial.println(_limb->angles[2]);
}

void spiderbot::moveLimbPolar(limb* _limb, double r, double z, double theta)
{
  // r = radius
  // z = height
  // theta = rotation (at base)
  float desired [3];
  // limbPosition desired;
  _inverseKinematic(r, z, &desired[1], &desired[2]);
  desired[0] = theta;
  _setLimbPosition(_limb, desired);
}

void spiderbot::setToInitialPosition()
{
  for(int j=0;j<=3;j++){
    moveLimbPolar(&spider0.body0.limbs[j], 5, 4, 90);
  }
}

void spiderbot::standUp()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(&spider0.body0.limbs[i], 5, 5, 90);
  }
  delay(200);
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(&spider0.body0.limbs[i], 5, 15, 90);
  }
}

void spiderbot::riseSlowly()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(&spider0.body0.limbs[i], 5, 4, 90);
  }
  delay(200);
  double a;
  for (int j = 400;j<1500;j=j+10){
    for(int i=0; i<=3; i++)
      {
      a=double(j);
    moveLimbPolar(&spider0.body0.limbs[i], 5, a/100 , 90);
    }
  }
}


void spiderbot::assumeStablePosition()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(&spider0.body0.limbs[i], 6, 4, 90);
  }
  delay(200);
  double a;
  for (int j = 400;j<1200;j=j+10){
    for(int i=0; i<=3; i++)
      {
      a=double(j);
    moveLimbPolar(&spider0.body0.limbs[i], 6, a/100 , 90);
    }
  }
}


void spiderbot::rest()
{
  double a;
  for (int j = 1000;j>350;j=j-10){
    for(int i=0; i<=3; i++)
      {
      a=double(j);
    moveLimbPolar(&spider0.body0.limbs[i], 8, a/100 , 90);
    }
  }
}

void spiderbot::goDoggy()
{
  for(int i=0; i<=3; i++)
  {
    moveLimbPolar(&spider0.body0.limbs[i], 5, 12, 135);
  }
}


void spiderbot::begin()//inilicializo a placa dos servos
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz update
}


void spiderbot::creepingGait(int stepSize, int n_steps)
{
  int footMoving[4] = {0, 2, 1, 3};
  assumeStablePosition();
  int increment = 5;
  delay(1000);
  int stepsTaken = 0;
  int stepFootIndex = 0;
  while (stepsTaken < n_steps)
  {
    // move the forwarding foot
    moveLimbPolar(&spider0.body0.limbs[footMoving[stepFootIndex]], 6, 9, 90);
    // Serial.println(spider0.body0.limbs[footMoving[stepFootIndex]].angles[0]);
    delay(100);
    moveLimbPolar(&spider0.body0.limbs[footMoving[stepFootIndex]], 6, 9, 90+stepSize);
    // Serial.println(spider0.body0.limbs[footMoving[stepFootIndex]].angles[0]);
    delay(100);
    moveLimbPolar(&spider0.body0.limbs[footMoving[stepFootIndex]], 6, 12, 90+stepSize);
    delay(100);


    // move the other feet
    for (int k=0;k<stepSize;k++)
    {
      // temporary solution to move slowly
      for (int j=0;j<=3;j++)
      {
        // Serial.println(spider0.body0.limbs[j].angles[0]);
        moveLimbPolar(&spider0.body0.limbs[j], 6, 12, spider0.body0.limbs[j].angles[0]+increment);
        // delay(5000);
      }
      // delay(5000);
    }
    // change the foot moving forward
    stepsTaken ++;
    stepFootIndex++;
    if (stepFootIndex>3)
    {
      stepFootIndex = 0;
    }

  }
  //
  // for each leg
}
