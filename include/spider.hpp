/*
A very beatiful description will go here in the nearby future
*/

//  - - - - - Including the necessary libraries - - - - - - - - - - - - - - - //
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - PWM limits for the Adafruit_PWMServoDriver, - - - - - - - - - - //
//  - - - - - Specifying the pulse lenght limit for the servos  - - - - - - - //
#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - Defining the number of servos in the overall robot- - - - - - - //
#define NUM_LEGS 4 //Number of legs to be considered
#define NUM_SERVOS_PER_LEG 3 //Number of servos per leg
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
//
// //  - - - - - Defining the relation robot_position -> slot in servo driver- - //
// //  - - - - - Where T stand for thighs, K for knees and F for feet -  - - - - //
// #define T_0 0
// #define T_1 1
// #define T_2 2
// #define T_3 3
//
// #define K_0 4
// #define K_1 5
// #define K_2 6
// #define K_3 7
//
// #define F_0 8
// #define F_1 9
// #define F_2 10
// #define F_3 11
// //  - - - - - -This defines the angular offset in each servo due to assembly- //
// //I dont know if this is correct, must check
// #define T_0_OFF -5
// #define T_1_OFF -5
// #define T_2_OFF -10
// #define T_3_OFF -3
//
// #define K_0_OFF -10 //OK
// #define K_1_OFF 5 //OK
// #define K_2_OFF -7// OK
// #define K_3_OFF 7 //OK
//
// #define F_0_OFF 5 // OK
// #define F_1_OFF -5
// #define F_2_OFF 5
// #define F_3_OFF 7 //OK
// //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
//
// //  - - - - - -Structures representing the robot configuration  - - - - - - - //
// // Defining the structures
// struct joint
// {
//   int servo_num;
//   int servo_offset;
//   bool bIsInverted;
// };
//
// struct limb
// {
//   joint joints[NUM_SERVOS_PER_LEG];
//   float angles[NUM_SERVOS_PER_LEG];
// };
//
// struct body
// {
//   limb limbs[NUM_LEGS];
// };
//
// // Assigning the structures to the servos and offsets
// static joint thigh0
// {
//   T_0,
//   T_0_OFF,
//   true
// };
// static joint thigh1
// {
//   T_1,
//   T_1_OFF,
//   false
// };
// static joint thigh2
// {
//   T_2,
//   T_2_OFF,
//   true
// };
// static joint thigh3
// {
//   T_3,
//   T_3_OFF,
//   false
// };
// static joint knee0
// {
//   K_0,
//   K_0_OFF,
//   true
// };
// static joint knee1
// {
//   K_1,
//   K_1_OFF,
//   false
// };
// static joint knee2
// {
//   K_2,
//   K_2_OFF,
//   true
// };
// static joint knee3
// {
//   K_3,
//   K_3_OFF,
//   false
// };
// static joint foot0
// {
//   F_0,
//   F_0_OFF,
//   true
// };
// static joint foot1
// {
//   F_1,
//   F_1_OFF,
//   false
// };
// static joint foot2
// {
//   F_2,
//   F_2_OFF,
//   true
// };
// static joint foot3
// {
//   F_3,
//   F_3_OFF,
//   false
// };
//
// static limb limb0
// {
//   {thigh0, knee0, foot0}
// };
// static limb limb1
// {
//   {thigh1, knee1, foot1}
// };
// static limb limb2
// {
//   {thigh2, knee2, foot2}
// };
// static limb limb3
// {
//   {thigh3, knee3, foot3}
// };
//
// static body robot0
// {
//   {limb0, limb1, limb2, limb3}
// };
// //  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //
//
// struct limbPosition
// {
//   int thighAngle;
//   int kneeAngle;
//   int footAngle;
// };
//
//
// // struct robotConfiguration
// // {
// //   limbPosition leg[NUM_LEGS];
// // };
// //
// //
// // static robotConfiguration configuration0;
//
//
// void command_by_serial_PolarPosition(struct spiderbot aranha1);
//
// class spiderbot
// {
// public:
//   void begin();
//   void moveLimbPolar(limb* _limb, double r, double z, double theta);
//   void setToInitialPosition();
//   void standUp();
//   void goDoggy();
//   void rest();
//   void riseSlowly();
//   void assumeStablePosition();
//   void moveAngle(int junta, int grau,int off);
//   void frente(int junta,int dist,int alt_atual,int amp,int vel);
//   void creepingGait(int stepSize, int n_steps);
//
//   body body0;
//
// // private:
//
//   void _moveJointAngle(joint _joint, int degree);
//   void _setLimbPosition(limb* _limb, float* desired);
//   void _inverseKinematic(double x,double y, float* a1, float* a2);
// };
//
// // static spiderbot spider0
// // {
// //   body body0 = robot0;
// // };









class RobotShape
{
public:
  // Size of links, from center of mass to ground, in mm, axis to axis


  static constexpr float offset_x = 51.05;
  static constexpr float offset_y = 50.91;
  static constexpr float offset_z = 53.5;
  static constexpr float d2 = 34;
  static constexpr float d3 = 80;
  static constexpr float d4 = 100;
};


class Point
{
public:
  Point();
  Point(float x, float y, float z);

  static float GetDistance(Point point1, Point point2);

  volatile float x, y, z;
};


class RobotLegsPoints
{
public:
  RobotLegsPoints();
  RobotLegsPoints(Point leg0, Point leg1, Point leg2, Point leg3);

  Point leg0, leg1, leg2, leg3;
};


class RobotJoint
{
public:
  RobotJoint();
  void Set(int servoNum, float offset, bool jointDir, float jointMinAngle, float jointMaxAngle);
  void SetOffset(float offset);
  void SetOffsetEnableState(bool state);
  void RotateToDirectly(float jointAngle);
  float GetJointAngle();
  bool CheckJointAngle(float jointAngle);

  volatile float jointAngleNow;
  volatile float servoAngleNow;

private:
  int servoNum;
  float offset;
  bool jointDir;
  float jointMinAngle;
  float jointMaxAngle;
  int offsetAddress;
};


class RobotLeg
{
public:
  RobotLeg();
  void Set(float xOrigin, float yOrigin, float zOrigin);

  void SetOffsetEnableState(bool state);

  void CalculatePoint(float alpha, float beta, float gamma, volatile float &x, volatile float &y, volatile float &z);
  void CalculatePoint(float alpha, float beta, float gamma, Point &point);
  void CalculateAngle(float x, float y, float z, float &alpha, float &beta, float &gamma);
  void CalculateAngle(Point point, float &alpha, float &beta, float &gamma);

  bool CheckPoint(Point point);
  bool CheckAngle(float alpha, float beta, float gamma);

  void MoveTo(Point point);
  void MoveToRelatively(Point point);
  void WaitUntilFree();

  void ServosRotateTo(float degreeA, float degreeB, float degreeC);

  void MoveToDirectly(Point point);
  void MoveToDirectlyRelatively(Point point);

  volatile bool isBusy = false;

  RobotJoint jointA, jointB, jointC;
  Point pointNow, pointGoal;

  static constexpr float negligibleDistance = 0.1;
  static constexpr float defaultStepDistance = 2;
  volatile float stepDistance = defaultStepDistance;

private:
  float xOrigin, yOrigin, zOrigin;
  volatile bool isFirstMove = true;

  void RotateToDirectly(float alpha, float beta, float gamma);
};


class Robot
{
public:
  Robot();
  void Start();
  enum State {Boot, Action };
  State state = State::Boot;
  void BootState();
  void MoveTo(RobotLegsPoints points);
  void MoveTo(RobotLegsPoints points, float speed);
  void MoveToRelatively(Point point);
  void MoveToRelatively(Point point, float speed);
  void WaitUntilFree();
  void SetSpeed(float speed);
  void SetSpeed(float speed0, float speed1, float speed2, float speed3);
  bool CheckPoints(RobotLegsPoints points);
  void GetPointsNow(RobotLegsPoints &points);
  void Update();
  RobotLeg leg0, leg1, leg2, leg3;
  const RobotLegsPoints bootPoints = RobotLegsPoints(
    Point(-90, 90, 0),
    Point(90, 90, 0),
    Point(90, -90, 0),
    Point(-90, -90, 0));


private:
  void UpdateAction();
  void UpdateLegAction(RobotLeg &leg);
  void MoveToDirectly(RobotLegsPoints points);
};


class RobotAction
{
public:
  RobotAction();
  void Start();

  void ActiveMode();
  void SleepMode();
  void SwitchMode();

  void CrawlForward();
  void CrawlBackward();
  void TurnLeft();
  void TurnRight();

  void MoveBody(float x, float y, float z);
  void RotateBody(float x, float y, float z);

  void TwistBody(Point move, Point rotate);

  void InitialState();

  void LegMoveToRelativelyDirectly(int leg, Point point);

  Robot robot;

private:
  void ActionState();

  enum Mode { Active, Sleep };
  Mode mode = Mode::Sleep;

  enum LegsState { Initial, Feet12Long, Feet34Long, Twist };
  LegsState legsState = LegsState::Initial;

  RobotLegsPoints initialPoints;

  const float legLift = -25;
  const float bodyLift = 15;

  const float crawlOffsetX = 15;
  const float crawlOffsetY = 15;

  const float crawlSpeedLeg = 7.5;
  const float crawlSpeedBody = crawlSpeedLeg * (crawlOffsetY / (7 * crawlOffsetY + 2 * legLift));

  void FirstStepForward();
  void FirstStepBackward();

  void LegStepTo(RobotLeg &leg, Point point, float speed);
  void LegStepToRelatively(RobotLeg &leg, Point point, float speed);
  void LegMoveToRelatively(RobotLeg &leg, Point point, float speed);
  void LegsMoveTo(RobotLegsPoints points);
  void LegsMoveTo(RobotLegsPoints points, float speed);
  void LegsMoveToRelatively(Point point, float speed);

  void GetCrawlPoints(RobotLegsPoints &points, Point point);
  void GetCrawlPoint(Point &point, Point direction);

  const float turnAngle = 22.5;

  const float turnOffset = 15;

  const float turnSpeedLeg = crawlSpeedLeg;
  const float turnSpeedBody = turnSpeedLeg / 6.5;

  void Turn(float degree);

  void GetTurnPoints(RobotLegsPoints &points, float angle);
  void GetTurnPoint(Point &point, float angle);

  const float speedTwistBody = 1.25;

  void TwistBody(Point move, Point rotateAxis, float rotateAngle);

  void GetMoveBodyPoints(RobotLegsPoints &points, Point point);
  void GetMoveBodyPoint(Point &point, Point direction);

  void GetRotateBodyPoints(RobotLegsPoints &points, Point rotateAxis, float rotateAngle);
  void GetRotateBodyPoint(Point &point, Point rotateAxis, float rotateAngle);
};
