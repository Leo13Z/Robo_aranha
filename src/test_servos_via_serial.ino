#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

#define C_1 0
#define C_2 1
#define C_3 2
#define C_4 3

#define J_1 4
#define J_2 5
#define J_3 6
#define J_4 7

#define P_1 8
#define P_2 9
#define P_3 10
#define P_4 11

int offset[12] = {0, 0, 0, 0, 20, 20, 10, 20, }

int perna1[3]={C_1,J_1,P_1};
int perna2[3]={C_2,J_2,P_2};
int perna3[3]={C_3,J_3,P_3};
int perna4[3]={C_4,J_4,P_4};

int coxas[4]={C_1,C_2,C_3,C_4};
int joelhos[4]={J_1,J_2,J_3,J_4};
int patas[4]={P_1,P_2,P_3,P_4};

String servo_to_move;
String angle_desired;

void set_walk_position()
{
  for(int i=0;i<=3;i++)
  {
    pwm.setPWM(coxas[i], 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
  }
  delay(500);


 pwm.setPWM(4, 0, (map(120, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(5, 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(6, 0, (map(120, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(7, 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
 delay(500);

 pwm.setPWM(8, 0, (map(60, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(9, 0, (map(140, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(10, 0, (map(50, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(11, 0, (map(140, 0,180,SERVOMIN,SERVOMAX)));
 delay(500);
}

void setup() {
  pwm.begin();
  Serial.begin(9600);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(50);
//  set_walk_position();


}

void command_by_serial()
{
  while (Serial.available())
  {
    char c = Serial.read();  //gets one byte from serial buffer
    servo_to_move += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }

  if (servo_to_move.length() >0)
  {
    Serial.print("Selected Servo: ");
    Serial.println(servo_to_move);  //so you can see the captured string
    Serial.print("Enter the angle:");

    int servo = servo_to_move.toInt();  //convert readString into a number

    while(!Serial.available());
    while (Serial.available())
    {
      char c = Serial.read();  //gets one byte from serial buffer
      angle_desired += c; //makes the string readString
      delay(2);  //slow looping to allow buffer to fill with next character
    }
    if (angle_desired.length() >0)
    {
      Serial.print("Selected Angle: ");
      Serial.println(angle_desired);  //so you can see the captured string
      int angle = angle_desired.toInt();  //convert readString into a number

      pwm.setPWM(servo, 0, (map(angle, 0,180,SERVOMIN,SERVOMAX)));
    }
  }
  servo_to_move=""; //empty for next input
  angle_desired=""; //empty for next input
}

void loop() {

  command_by_serial();

}
