#include"spider.hpp"
#include <math.h>
// #define PI 3.14159265


//  - - - - - Defining the Servo Driver's object  - - - - - - - - - - - - - - //
Adafruit_PWMServoDriver pwm;
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Class Point - - - - -   - - - - - - - - - - - - - - - - - - - - //
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
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


//  - - - - - Class RobotLegsPoints - - - - - - - - - - - - - - - - - - - - - //
RobotLegsPoints::RobotLegsPoints() {}

RobotLegsPoints::RobotLegsPoints(Point leg0, Point leg1, Point leg2, Point leg3)
{
  this->leg0 = leg0;
  this->leg1 = leg1;
  this->leg2 = leg2;
  this->leg3 = leg3;
}
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


//  - - - - - Class RobotJoint - - - - -  - - - - - - - - - - - - - - - - - - //
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

float RobotJoint::GetJointAngle()
{
  return (servoAngleNow);
}

bool RobotJoint::CheckJointAngle(float jointAngle)
{

  if (jointAngle >= jointMinAngle && jointAngle <= jointMaxAngle)
    return true;
  else
    return false;
}
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //


//  - - - - - Class RobotLeg   - - - - -  - - - - - - - - - - - - - - - - - - //
RobotLeg::RobotLeg() {}

void RobotLeg::Set(float xOrigin, float yOrigin, float zOrigin)
{
  this->xOrigin = xOrigin;
  this->yOrigin = yOrigin;
  this->zOrigin = zOrigin;
}

void RobotLeg::CalculatePoint(float alpha, float beta, float gamma, volatile float &x, volatile float &y, volatile float &z)
{
  // transform angle to radian
  alpha = (alpha + 45) * PI / 180;
  beta = beta * PI / 180;
  gamma = (90 - gamma) * PI / 180;
  // calculate u-v coordinate
  float u, v;

  u = RobotShape::d2 + RobotShape::d3 * sin(beta) + RobotShape::d4 * sin(beta - gamma);
  v = RobotShape::d3 * cos(beta) + RobotShape::d4 * cos(beta - gamma);
  // calculate x-y-z coordinate
  x = xOrigin + u * cos(alpha);
  y = yOrigin + u * sin(alpha);
  z = zOrigin + v;
}

void RobotLeg::CalculatePoint(float alpha, float beta, float gamma, Point &point)
{
  CalculatePoint(alpha, beta, gamma, point.x, point.y, point.z);
}

void RobotLeg::CalculateAngle(float x, float y, float z, float &alpha, float &beta, float &gamma)
{
  float u, v, alpha_l, Dist, cos_gamma_l, gamma_l, senB;
  u = (sqrt(pow(x - xOrigin, 2) + pow(y - yOrigin, 2)))- RobotShape::d2;
  alpha_l = atan2(y - yOrigin, x - xOrigin);
  if (xOrigin < 0 && yOrigin < 0)
    alpha_l = alpha_l + PI;
  alpha = (alpha_l * 180 / PI) + 45;

  v = z - zOrigin;
  Dist = sqrt(pow(u, 2) + pow(v, 2));
  cos_gamma_l = (pow(Dist, 2) - pow(RobotShape::d3, 2) - pow(RobotShape::d4, 2))/(2 * RobotShape::d3 * RobotShape::d4);
  gamma_l = atan2((sqrt(1 - pow(cos_gamma_l, 2))), cos_gamma_l);
  gamma = 90 - (gamma_l * 180 / PI);

  senB = RobotShape::d4 * sin(gamma_l) / Dist;
  beta = atan2(u,v) + atan2(senB, sqrt(1 - pow(senB, 2)));
  beta = beta * 180 / PI;
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
  Point point;
  CalculatePoint(degreeA, degreeB, degreeC, point);
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
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Class Robot - -  - - - - -  - - - - - - - - - - - - - - - - - - //
Robot::Robot() {}

void Robot::Start()
{
  leg0.Set(-RobotShape::offset_x, RobotShape::offset_y, -RobotShape::offset_z);
  leg1.Set(RobotShape::offset_x, RobotShape::offset_y, -RobotShape::offset_z);
  leg2.Set(RobotShape::offset_x, -RobotShape::offset_y, -RobotShape::offset_z);
  leg3.Set(-RobotShape::offset_x, -RobotShape::offset_y, -RobotShape::offset_z);

  leg0.jointA.Set(0, -5, true, 0, 180);
  leg0.jointB.Set(4, -10, true, 0, 180);
  leg0.jointC.Set(8, 5, true, 0, 180);
  leg1.jointA.Set(1, -5, false, 0, 180);
  leg1.jointB.Set(5, 5, false, 0, 180);
  leg1.jointC.Set(9, -5, false, 0, 180);
  leg2.jointA.Set(2, -10, true, 0, 180);
  leg2.jointB.Set(6, -7, true, 0, 180);
  leg2.jointC.Set(10, 5, true, 0, 180);
  leg3.jointA.Set(3, -3, false, 0, 180);
  leg3.jointB.Set(7, 7, false, 0, 180);
  leg3.jointC.Set(11, 7, false, 0, 180);

  MoveToDirectly(bootPoints);
}

void Robot::BootState()
{
  SetSpeed(RobotLeg::defaultStepDistance);
  MoveTo(bootPoints);
  WaitUntilFree();
  state = State::Boot;
}

void Robot::MoveTo(RobotLegsPoints points)
{
  leg0.MoveTo(points.leg0);
  leg1.MoveTo(points.leg1);
  leg2.MoveTo(points.leg2);
  leg3.MoveTo(points.leg3);
}

void Robot::MoveTo(RobotLegsPoints points, float speed)
{
  SetSpeed(speed);
  MoveTo(points);
}

void Robot::MoveToRelatively(Point point)
{
  leg0.MoveToRelatively(point);
  leg1.MoveToRelatively(point);
  leg2.MoveToRelatively(point);
  leg3.MoveToRelatively(point);
}

void Robot::MoveToRelatively(Point point, float speed)
{
  SetSpeed(speed);
  MoveToRelatively(point);
}

void Robot::WaitUntilFree()
{
  while (leg0.isBusy || leg1.isBusy || leg2.isBusy || leg3.isBusy);
}

void Robot::SetSpeed(float speed)
{
  leg0.stepDistance = speed;
  leg1.stepDistance = speed;
  leg2.stepDistance = speed;
  leg3.stepDistance = speed;
}

void Robot::SetSpeed(float speed0, float speed1, float speed2, float speed3)
{
  leg0.stepDistance = speed0;
  leg1.stepDistance = speed1;
  leg2.stepDistance = speed2;
  leg3.stepDistance = speed3;
}

bool Robot::CheckPoints(RobotLegsPoints points)
{
  if (leg0.CheckPoint(points.leg0) &&
      leg1.CheckPoint(points.leg1) &&
      leg2.CheckPoint(points.leg2) &&
      leg3.CheckPoint(points.leg3))
    return true;
  return false;
}

void Robot::GetPointsNow(RobotLegsPoints & points)
{
  points.leg0 = leg0.pointNow;
  points.leg1 = leg1.pointNow;
  points.leg2 = leg2.pointNow;
  points.leg3 = leg3.pointNow;
}

void Robot::Update()
{
  UpdateLegAction(leg0);
  UpdateLegAction(leg1);
  UpdateLegAction(leg2);
  UpdateLegAction(leg3);
}

void Robot::UpdateLegAction(RobotLeg &leg)
{
  float distance = Point::GetDistance(leg.pointNow, leg.pointGoal);
  float xDistance = leg.pointGoal.x - leg.pointNow.x;
  float yDistance = leg.pointGoal.y - leg.pointNow.y;
  float zDistance = leg.pointGoal.z - leg.pointNow.z;
  float xStep = xDistance / distance * leg.stepDistance;
  float yStep = yDistance / distance * leg.stepDistance;
  float zStep = zDistance / distance * leg.stepDistance;
  Point pointGoal = Point(leg.pointNow.x + xStep, leg.pointNow.y + yStep, leg.pointNow.z + zStep);

  if (distance >= leg.stepDistance && distance >= RobotLeg::negligibleDistance)
  {
    leg.isBusy = true;
    leg.MoveToDirectly(pointGoal);
  }
  else if (leg.isBusy)
  {
    leg.MoveToDirectly(leg.pointGoal);
    leg.isBusy = false;
  }
}

void Robot::MoveToDirectly(RobotLegsPoints points)
{
  leg0.MoveToDirectly(points.leg0);
  leg1.MoveToDirectly(points.leg1);
  leg2.MoveToDirectly(points.leg2);
  leg3.MoveToDirectly(points.leg3);
}
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Class RobotAction- - - - -  - - - - - - - - - - - - - - - - - - //
RobotAction::RobotAction() {}

void RobotAction::Start()
{
  robot.Start();

  initialPoints = robot.bootPoints;
  GetCrawlPoints(initialPoints, Point(0, 0, bodyLift));
}

void RobotAction::ActiveMode()
{
  ActionState();
  if (legsState == LegsState::Twist)
    InitialState();
  if (mode == Mode::Active)
    return;

  LegsMoveToRelatively(Point(0, 0, bodyLift), crawlSpeedBody);
  mode = Mode::Active;
}

void RobotAction::SleepMode()
{
  ActionState();
  if (legsState == LegsState::Twist)
    InitialState();
  if (mode == Mode::Sleep)
    return;

  LegsMoveToRelatively(Point(0, 0, -bodyLift), crawlSpeedBody);
  mode = Mode::Sleep;
}

void RobotAction::SwitchMode()
{
  ActionState();
  if (mode == Mode::Active)
    SleepMode();
  else
    ActiveMode();
}

void RobotAction::CrawlForward()
{
  ActionState();
  if (legsState == LegsState::Twist)
    InitialState();
  if (mode != Mode::Active)
    ActiveMode();

  if (legsState == LegsState::Initial)
  {
    FirstStepForward();
  }
  else if (legsState == LegsState::Feet34Long)
  {
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg0.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, 0, legLift));
    robot.leg3.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg1.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg0.WaitUntilFree();

    robot.leg0.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg0.WaitUntilFree();

    robot.leg0.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg2.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg1.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, 0, legLift));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet12Long;
  }
  else
  {
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg1.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg1.MoveToRelatively(Point(0, 0, legLift));
    robot.leg2.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg3.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, 0, legLift));
    robot.leg1.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet34Long;
  }
}

void RobotAction::CrawlBackward()
{
  ActionState();
  if (legsState == LegsState::Twist)
    InitialState();
  if (mode != Mode::Active)
    ActiveMode();

  if (legsState == LegsState::Initial)
  {
    FirstStepBackward();
  }
  else if (legsState == LegsState::Feet12Long)
  {
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg2.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg1.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, 0, legLift));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg0.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, 0, legLift));
    robot.leg3.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg1.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg0.WaitUntilFree();

    robot.leg0.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg0.WaitUntilFree();

    robot.leg0.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet34Long;
  }
  else
  {
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg3.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, 0, legLift));
    robot.leg1.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg1.stepDistance = crawlSpeedLeg;

    robot.leg0.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg1.MoveToRelatively(Point(0, 0, legLift));
    robot.leg2.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet12Long;
  }
}

void RobotAction::TurnLeft()
{
  Turn(-turnAngle);
}

void RobotAction::TurnRight()
{
  Turn(turnAngle);
}

void RobotAction::MoveBody(float x, float y, float z)
{
  TwistBody(Point(x, y, z), Point(0, 0, 0));
}

void RobotAction::RotateBody(float x, float y, float z)
{
  TwistBody(Point(0, 0, 0), Point(x, y, z));
}

void RobotAction::TwistBody(Point move, Point rotate)
{
  float angle = sqrt(pow(rotate.x, 2) + pow(rotate.y, 2) + pow(rotate.z, 2));
  TwistBody(move, rotate, angle);
}

void RobotAction::InitialState()
{
  ActionState();
  if (legsState == LegsState::Initial)
  {
    if (mode != Mode::Active)
      ActiveMode();
  }
  else if (legsState == LegsState::Feet34Long)
  {
    LegsMoveToRelatively(Point(-crawlOffsetX, crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg0, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, -2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg3, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(2 * crawlOffsetX, 0, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg2, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, 2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg1, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(-crawlOffsetX, -crawlOffsetY, 0), turnSpeedBody);
  }
  else if (legsState == LegsState::Feet12Long)
  {
    LegsMoveToRelatively(Point(crawlOffsetX, crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg1, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, -2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg2, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(-2 * crawlOffsetX, 0, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg3, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, 2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg0, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(crawlOffsetX, -crawlOffsetY, 0), turnSpeedBody);
  }
  else if (legsState == LegsState::Twist)
  {
    TwistBody(Point(0, 0, 0), Point(0, 0, 0), 0);
  }
  legsState = LegsState::Initial;
}

void RobotAction::LegMoveToRelativelyDirectly(int leg, Point point)
{
  switch (leg)
  {
  case 0:
    robot.leg0.MoveToRelatively(point);
    break;
  case 1:
    robot.leg1.MoveToRelatively(point);
    break;
  case 2:
    robot.leg2.MoveToRelatively(point);
    break;
  case 3:
    robot.leg3.MoveToRelatively(point);
    break;
  }
}

void RobotAction::ActionState()
{
  if (robot.state != Robot::State::Action)
  {
    robot.BootState();
    robot.state = Robot::State::Action;
    mode = Mode::Sleep;
    legsState = LegsState::Initial;
  }
}

void RobotAction::FirstStepForward()
{
  //
  LegsMoveToRelatively(Point(crawlOffsetX, 0, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg1, Point(0, 4 * crawlOffsetY, 0), crawlSpeedLeg);
  LegsMoveToRelatively(Point(-crawlOffsetX, -2 * crawlOffsetY, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg3, Point(0, 4 * crawlOffsetY, 0), crawlSpeedLeg);
  ////
  legsState = LegsState::Feet34Long;
}

void RobotAction::FirstStepBackward()
{
  //
  LegsMoveToRelatively(Point(-crawlOffsetX, 0, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg3, Point(0, -4 * crawlOffsetY, 0), crawlSpeedLeg);
  LegsMoveToRelatively(Point(crawlOffsetX, 2 * crawlOffsetY, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg1, Point(0, -4 * crawlOffsetY, 0), crawlSpeedLeg);
  ////
  legsState = LegsState::Feet12Long;
}

void RobotAction::LegStepTo(RobotLeg & leg, Point point, float speed)
{
  leg.stepDistance = speed;
  leg.MoveToRelatively(Point(0, 0, legLift));
  leg.WaitUntilFree();
  leg.MoveTo(Point(point.x, point.y, leg.pointNow.z));
  leg.WaitUntilFree();
  leg.MoveToRelatively(Point(0, 0, -legLift));
  leg.WaitUntilFree();
}

void RobotAction::LegStepToRelatively(RobotLeg &leg, Point point, float speed)
{
  leg.stepDistance = speed;
  leg.MoveToRelatively(Point(0, 0, legLift));
  leg.WaitUntilFree();
  leg.MoveToRelatively(Point(point.x, point.y, 0));
  leg.WaitUntilFree();
  leg.MoveToRelatively(Point(0, 0, -legLift));
  leg.WaitUntilFree();
}

void RobotAction::LegMoveToRelatively(RobotLeg & leg, Point point, float speed)
{
  leg.stepDistance = speed;
  leg.MoveToRelatively(point);
  leg.WaitUntilFree();
}

void RobotAction::LegsMoveTo(RobotLegsPoints points)
{
  robot.MoveTo(points);
  robot.WaitUntilFree();
}

void RobotAction::LegsMoveTo(RobotLegsPoints points, float speed)
{
  robot.SetSpeed(speed);
  robot.MoveTo(points);
  robot.WaitUntilFree();
}

void RobotAction::LegsMoveToRelatively(Point point, float speed)
{
  robot.SetSpeed(speed);
  robot.MoveToRelatively(point);
  robot.WaitUntilFree();
}

void RobotAction::GetCrawlPoints(RobotLegsPoints & points, Point point)
{
  GetCrawlPoint(points.leg0, point);
  GetCrawlPoint(points.leg1, point);
  GetCrawlPoint(points.leg2, point);
  GetCrawlPoint(points.leg3, point);
}

void RobotAction::GetCrawlPoint(Point & point, Point direction)
{
  point = Point(point.x + direction.x, point.y + direction.y, point.z + direction.z);
}

void RobotAction::Turn(float angle)
{
  ActionState();
  if (legsState == LegsState::Twist)
    InitialState();
  if (mode != Mode::Active)
    ActiveMode();

  angle /= 4;

  RobotLegsPoints points;

  if (angle > 0)
  {
    if (legsState == LegsState::Feet34Long)
    {
      // body move away from leg3
      LegsMoveToRelatively(Point(-turnOffset, -turnOffset, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 3);
      GetCrawlPoint(points.leg3, Point(-turnOffset, -turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg2
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 2);
      GetCrawlPoint(points.leg2, Point(turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg1
      LegsMoveToRelatively(Point(0, 2 * turnOffset, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 1);
      GetCrawlPoint(points.leg1, Point(turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg0
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 0 up
      LegMoveToRelatively(robot.leg0, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 0 point
      points.leg0 = robot.bootPoints.leg0;
      GetTurnPoint(points.leg0, angle * 0);
      GetCrawlPoint(points.leg0, Point(-turnOffset, turnOffset, robot.leg0.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 0 down
      LegMoveToRelatively(robot.leg0, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(turnOffset, -turnOffset, 0), turnSpeedBody);
    }
    else
    {
      // body move away from leg1
      LegsMoveToRelatively(Point(turnOffset, turnOffset, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 3);
      GetCrawlPoint(points.leg1, Point(turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg0
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 0 up
      LegMoveToRelatively(robot.leg0, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 0 point
      points.leg0 = robot.bootPoints.leg0;
      GetTurnPoint(points.leg0, angle * 2);
      GetCrawlPoint(points.leg0, Point(-turnOffset, turnOffset, robot.leg0.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 0 down
      LegMoveToRelatively(robot.leg0, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg3
      LegsMoveToRelatively(Point(0, -2 * turnOffset, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 1);
      GetCrawlPoint(points.leg3, Point(-turnOffset, -turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg2
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 0);
      GetCrawlPoint(points.leg2, Point(turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(-turnOffset, turnOffset, 0), turnSpeedBody);
    }
  }
  else
  {
    if (legsState == LegsState::Feet12Long)
    {
      // body move away from leg2
      LegsMoveToRelatively(Point(turnOffset, -turnOffset, 0), turnSpeedBody);
      // leg 4 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 4 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 3);
      GetCrawlPoint(points.leg2, Point(turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 4 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg3
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 2);
      GetCrawlPoint(points.leg3, Point(-turnOffset, -turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg0
      LegsMoveToRelatively(Point(0, 2 * turnOffset, 0), turnSpeedBody);
      // leg 0 up
      LegMoveToRelatively(robot.leg0, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 0 point
      points.leg0 = robot.bootPoints.leg0;
      GetTurnPoint(points.leg0, angle * 1);
      GetCrawlPoint(points.leg0, Point(-turnOffset, turnOffset, robot.leg0.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 0 down
      LegMoveToRelatively(robot.leg0, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg1
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 0);
      GetCrawlPoint(points.leg1, Point(turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(-turnOffset, -turnOffset, 0), turnSpeedBody);
    }
    else
    {
      // body move away from leg0
      LegsMoveToRelatively(Point(-turnOffset, turnOffset, 0), turnSpeedBody);
      // leg 0 up
      LegMoveToRelatively(robot.leg0, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 0 point
      points.leg0 = robot.bootPoints.leg0;
      GetTurnPoint(points.leg0, angle * 3);
      GetCrawlPoint(points.leg0, Point(-turnOffset, turnOffset, robot.leg0.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 0 down
      LegMoveToRelatively(robot.leg0, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg1
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 2);
      GetCrawlPoint(points.leg1, Point(turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg2
      LegsMoveToRelatively(Point(0, -2 * turnOffset, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 1);
      GetCrawlPoint(points.leg2, Point(turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg3
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 0);
      GetCrawlPoint(points.leg3, Point(-turnOffset, -turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(turnOffset, turnOffset, 0), turnSpeedBody);
    }
  }
  legsState = LegsState::Initial;
}

void RobotAction::GetTurnPoints(RobotLegsPoints & points, float angle)
{
  GetTurnPoint(points.leg0, angle);
  GetTurnPoint(points.leg1, angle);
  GetTurnPoint(points.leg2, angle);
  GetTurnPoint(points.leg3, angle);
}

void RobotAction::GetTurnPoint(Point & point, float angle)
{
  float radian = angle * PI / 180;
  float radius = sqrt(pow(point.x, 2) + pow(point.y, 2));

  float x = radius * cos(atan2(point.y, point.x) + radian);
  float y = radius * sin(atan2(point.y, point.x) + radian);

  point = Point(x, y, point.z);
}

void RobotAction::TwistBody(Point move, Point rotateAxis, float rotateAngle)
{
  ActionState();
  if (legsState != LegsState::Twist)
    InitialState();

  RobotLegsPoints points = initialPoints;

  // move body
  move.x = constrain(move.x, -30, 30);
  move.y = constrain(move.y, -30, 30);
  move.z = constrain(move.z, 0, 45);
  GetMoveBodyPoints(points, move);

  // rotate body
  rotateAngle = constrain(rotateAngle, -15, 15);
  GetRotateBodyPoints(points, rotateAxis, rotateAngle);

  if (!robot.CheckPoints(points))
    return;

  LegsMoveTo(points, speedTwistBody);

  legsState = LegsState::Twist;
}

void RobotAction::GetMoveBodyPoints(RobotLegsPoints & points, Point point)
{
  GetMoveBodyPoint(points.leg0, point);
  GetMoveBodyPoint(points.leg1, point);
  GetMoveBodyPoint(points.leg2, point);
  GetMoveBodyPoint(points.leg3, point);
}

void RobotAction::GetMoveBodyPoint(Point & point, Point direction)
{
  point = Point(point.x - direction.x, point.y - direction.y, point.z - direction.z);
}

void RobotAction::GetRotateBodyPoints(RobotLegsPoints &points, Point rotateAxis, float rotateAngle)
{
  float rotateAxisLength = sqrt(pow(rotateAxis.x, 2) + pow(rotateAxis.y, 2) + pow(rotateAxis.z, 2));
  if (rotateAxisLength == 0)
  {
    rotateAxis.x = 0;
    rotateAxis.y = 0;
    rotateAxis.z = 1;
  }
  else
  {
    rotateAxis.x /= rotateAxisLength;
    rotateAxis.y /= rotateAxisLength;
    rotateAxis.z /= rotateAxisLength;
  }

  GetRotateBodyPoint(points.leg0, rotateAxis, rotateAngle);
  GetRotateBodyPoint(points.leg1, rotateAxis, rotateAngle);
  GetRotateBodyPoint(points.leg2, rotateAxis, rotateAngle);
  GetRotateBodyPoint(points.leg3, rotateAxis, rotateAngle);
}

void RobotAction::GetRotateBodyPoint(Point & point, Point rotateAxis, float rotateAngle)
{
  Point oldPoint = point;

  rotateAngle = rotateAngle * PI / 180;
  float c = cos(rotateAngle);
  float s = sin(rotateAngle);

  point.x = (rotateAxis.x * rotateAxis.x * (1 - c) + c) * oldPoint.x + (rotateAxis.x * rotateAxis.y * (1 - c) - rotateAxis.z * s) * oldPoint.y + (rotateAxis.x * rotateAxis.z * (1 - c) + rotateAxis.y * s) * oldPoint.z;
  point.y = (rotateAxis.y * rotateAxis.x * (1 - c) + rotateAxis.z * s) * oldPoint.x + (rotateAxis.y * rotateAxis.y * (1 - c) + c) * oldPoint.y + (rotateAxis.y * rotateAxis.z * (1 - c) - rotateAxis.x * s) * oldPoint.z;
  point.z = (rotateAxis.x * rotateAxis.z * (1 - c) - rotateAxis.y * s) * oldPoint.x + (rotateAxis.y * rotateAxis.z * (1 - c) + rotateAxis.x * s) * oldPoint.y + (rotateAxis.z * rotateAxis.z * (1 - c) + c) * oldPoint.z;
}
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //




































//
//
// //  - - - - - Performs the Inverse Cinematic, to determine the distance and-  //
// //  - - - - - height of the tip of the foot in relation to the knee servo - - //
// void spiderbot::_inverseKinematic(double x,double y, float* a1, float* a2)
// {
//   double B = sqrt((x*x)+(y*y));
//   double alpha1 = acos(y/B);
//   double alpha2 = acos(((L2*L2)-(B*B)-(L1*L1))/(-2*B*L1));
//   alpha1 = int((alpha1 + alpha2)* (180 / PI));
//
//   alpha2 = int(acos((-(L1*L1)-(L2*L2)+(B*B))/(-2*L2*L1))* (180 / PI));
//   alpha2 = int(180 -alpha2 +0);//45
//
//   if ((alpha1>0) && (alpha1<180)){
//     *a1=alpha1;
//   }
//   if ((alpha2>0) && (alpha2<180)){
//   *a2=alpha2;
//   }
// }
// //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
//
// void spiderbot::_moveJointAngle(joint _joint, int degree)
// {
//   int internal_degree = degree + _joint.servo_offset;
//   if (_joint.bIsInverted)
//   {
//     internal_degree = 180 - degree + _joint.servo_offset;
//   }
//     pwm.setPWM(_joint.servo_num,0,(map(internal_degree,0,180,SERVOMIN,SERVOMAX)));
// }
//
// void spiderbot::_setLimbPosition(limb* _limb, float* desired_position) {
//   Serial.print("_limb->joints[0]: ");
//   // Serial.println(_limb.joints[0]);
//   _moveJointAngle(_limb->joints[0], desired_position[0]);
//   Serial.print("_limb->joints[1]: ");
//   // Serial.println(_limb->joints[1]);
//   _moveJointAngle(_limb->joints[1], desired_position[1]);
//   Serial.print("_limb->joints[2]: ");
//   // Serial.println(_limb->joints[2]);
//   _moveJointAngle(_limb->joints[2], desired_position[2]);
//   _limb->angles[0] = desired_position[0];
//   Serial.print("_limb->angles[0]: ");
//   Serial.println(_limb->angles[0]);
//   _limb->angles[1] = desired_position[1];
//   Serial.print("_limb->angles[1]: ");
//   Serial.println(_limb->angles[1]);
//   _limb->angles[2] = desired_position[2];
//   Serial.print("_limb->angles[2]: ");
//   Serial.println(_limb->angles[2]);
// }
//
// void spiderbot::moveLimbPolar(limb* _limb, double r, double z, double theta)
// {
//   // r = radius
//   // z = height
//   // theta = rotation (at base)
//   float desired [3];
//   // limbPosition desired;
//   _inverseKinematic(r, z, &desired[1], &desired[2]);
//   desired[0] = theta;
//   _setLimbPosition(_limb, desired);
// }
//
// void spiderbot::setToInitialPosition()
// {
//   for(int j=0;j<=3;j++){
//     moveLimbPolar(&spider0.body0.limbs[j], 5, 4, 90);
//   }
// }
//
// void spiderbot::standUp()
// {
//   for(int i=0; i<=3; i++)
//   {
//     moveLimbPolar(&spider0.body0.limbs[i], 5, 5, 90);
//   }
//   delay(200);
//   for(int i=0; i<=3; i++)
//   {
//     moveLimbPolar(&spider0.body0.limbs[i], 5, 15, 90);
//   }
// }
//
// void spiderbot::riseSlowly()
// {
//   for(int i=0; i<=3; i++)
//   {
//     moveLimbPolar(&spider0.body0.limbs[i], 5, 4, 90);
//   }
//   delay(200);
//   double a;
//   for (int j = 400;j<1500;j=j+10){
//     for(int i=0; i<=3; i++)
//       {
//       a=double(j);
//     moveLimbPolar(&spider0.body0.limbs[i], 5, a/100 , 90);
//     }
//   }
// }
//
//
// void spiderbot::assumeStablePosition()
// {
//   for(int i=0; i<=3; i++)
//   {
//     moveLimbPolar(&spider0.body0.limbs[i], 6, 4, 90);
//   }
//   delay(200);
//   double a;
//   for (int j = 400;j<1200;j=j+10){
//     for(int i=0; i<=3; i++)
//       {
//       a=double(j);
//     moveLimbPolar(&spider0.body0.limbs[i], 6, a/100 , 90);
//     }
//   }
// }
//
//
// void spiderbot::rest()
// {
//   double a;
//   for (int j = 1000;j>350;j=j-10){
//     for(int i=0; i<=3; i++)
//       {
//       a=double(j);
//     moveLimbPolar(&spider0.body0.limbs[i], 8, a/100 , 90);
//     }
//   }
// }
//
// void spiderbot::goDoggy()
// {
//   for(int i=0; i<=3; i++)
//   {
//     moveLimbPolar(&spider0.body0.limbs[i], 5, 12, 135);
//   }
// }
//
//
// void spiderbot::begin()//inilicializo a placa dos servos
// {
//   pwm.begin();
//   pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz update
// }
//
//
// void spiderbot::creepingGait(int stepSize, int n_steps)
// {
//   int footMoving[4] = {0, 2, 1, 3};
//   assumeStablePosition();
//   int increment = 5;
//   delay(1000);
//   int stepsTaken = 0;
//   int stepFootIndex = 0;
//   while (stepsTaken < n_steps)
//   {
//     // move the forwarding foot
//     moveLimbPolar(&spider0.body0.limbs[footMoving[stepFootIndex]], 6, 9, 90);
//     // Serial.println(spider0.body0.limbs[footMoving[stepFootIndex]].angles[0]);
//     delay(100);
//     moveLimbPolar(&spider0.body0.limbs[footMoving[stepFootIndex]], 6, 9, 90+stepSize);
//     // Serial.println(spider0.body0.limbs[footMoving[stepFootIndex]].angles[0]);
//     delay(100);
//     moveLimbPolar(&spider0.body0.limbs[footMoving[stepFootIndex]], 6, 12, 90+stepSize);
//     delay(100);
//
//
//     // move the other feet
//     for (int k=0;k<stepSize;k++)
//     {
//       // temporary solution to move slowly
//       for (int j=0;j<=3;j++)
//       {
//         // Serial.println(spider0.body0.limbs[j].angles[0]);
//         moveLimbPolar(&spider0.body0.limbs[j], 6, 12, spider0.body0.limbs[j].angles[0]+increment);
//         // delay(5000);
//       }
//       // delay(5000);
//     }
//     // change the foot moving forward
//     stepsTaken ++;
//     stepFootIndex++;
//     if (stepFootIndex>3)
//     {
//       stepFootIndex = 0;
//     }
//
//   }
//   //
//   // for each leg
// }
