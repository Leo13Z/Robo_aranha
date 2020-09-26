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

//  - - - - - Defining the relation robot_position -> slot in servo driver- - //
//  - - - - - Where T stand for thighs, K for knees and F for feet -  - - - - //
#define T_0 0
#define T_1 1
#define T_2 2
#define T_3 3

#define K_0 4
#define K_1 5
#define K_2 6
#define K_3 7

#define F_0 8
#define F_1 9
#define F_2 10
#define F_3 11
//  - - - - - -This defines the angular offset in each servo due to assembly- //
//I dont know if this is correct, must check
#define T_0_OFF -5
#define T_1_OFF -5
#define T_2_OFF -10
#define T_3_OFF -3

#define K_0_OFF -10 //OK
#define K_1_OFF 5 //OK
#define K_2_OFF -7// OK
#define K_3_OFF 7 //OK

#define F_0_OFF 5 // OK
#define F_1_OFF -5
#define F_2_OFF -5
#define F_3_OFF 7 //OK
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

//  - - - - - -Structures representing the robot configuration  - - - - - - - //
// Defining the structures
struct joint
{
  int servo_num;
  int servo_offset;
  bool bIsInverted;
};

struct limb
{
  joint joints[NUM_SERVOS_PER_LEG];
};

struct body
{
  limb limbs[NUM_LEGS];
};

// Assigning the structures to the servos and offsets
static joint thigh0
{
  T_0,
  T_0_OFF,
  true
};
static joint thigh1
{
  T_1,
  T_1_OFF,
  false
};
static joint thigh2
{
  T_2,
  T_2_OFF,
  true
};
static joint thigh3
{
  T_3,
  T_3_OFF,
  false
};
static joint knee0
{
  K_0,
  K_0_OFF,
  true
};
static joint knee1
{
  K_1,
  K_1_OFF,
  false
};
static joint knee2
{
  K_2,
  K_2_OFF,
  true
};
static joint knee3
{
  K_3,
  K_3_OFF,
  false
};
static joint foot0
{
  F_0,
  F_0_OFF,
  true
};
static joint foot1
{
  F_1,
  F_1_OFF,
  false
};
static joint foot2
{
  F_2,
  F_2_OFF,
  true
};
static joint foot3
{
  F_3,
  F_3_OFF,
  false
};

static limb limb0
{
  {thigh0, knee0, foot0}
};
static limb limb1
{
  {thigh1, knee1, foot1}
};
static limb limb2
{
  {thigh2, knee2, foot2}
};
static limb limb3
{
  {thigh3, knee3, foot3}
};

static body robot0
{
  {limb0, limb1, limb2, limb3}
};
//  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - //

struct limbPosition
{
  int thighAngle;
  int kneeAngle;
  int footAngle;
};
void command_by_serial_PolarPosition(struct spiderbot aranha1);

class spiderbot
{
public:
  void begin();
  void moveLimbPolar(limb _limb, double r, double z, double theta);
  void setToInitialPosition();
  void standUp();
  void goDoggy();
  void rest();
  void riseSlowly();
  void moveAngle(int junta, int grau,int off);
  void frente(int junta,int dist,int alt_atual,int amp,int vel);
  body body0 = robot0;

// private:

  void _moveJointAngle(joint _joint, int degree);
  void _setLimbPosition(limb _limb, limbPosition desired_position);
  void _inverseKinematic(double x,double y, int* a1, int* a2);
};
