#include<Arduino.h>
#include"spider.hpp"

// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void test_rise()
{
  for(int i=0;i<=3;i++)
  {
    pwm.setPWM(coxas[i], 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
  }
  delay(500);

  pwm.setPWM(4, 0, (map(30, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(5, 0, (map(150, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(6, 0, (map(40, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(7, 0, (map(150, 0,180,SERVOMIN,SERVOMAX)));
  delay(500);

  pwm.setPWM(8, 0, (map(30, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(9, 0, (map(150, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(10, 0, (map(30, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(11, 0, (map(150, 0,180,SERVOMIN,SERVOMAX)));
  delay(5000);

  //rise here
  pwm.setPWM(4, 0, (map(150, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(5, 0, (map(30, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(6, 0, (map(160, 0,180,SERVOMIN,SERVOMAX)));
  pwm.setPWM(7, 0, (map(30, 0,180,SERVOMIN,SERVOMAX)));
  delay(5000);
}


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


void command_by_serial()
{
  String servo_to_move;
  String angle_desired;
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


void test_stand_up() {
  for(int i=0;i<=3;i++)
  {
    pwm.setPWM(coxas[i], 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
  }
  delay(5000);


 pwm.setPWM(4, 0, (map(120, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(5, 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(6, 0, (map(120, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(7, 0, (map(90, 0,180,SERVOMIN,SERVOMAX)));
 delay(500);

 pwm.setPWM(8, 0, (map(60, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(9, 0, (map(140, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(10, 0, (map(50, 0,180,SERVOMIN,SERVOMAX)));
 pwm.setPWM(11, 0, (map(140, 0,180,SERVOMIN,SERVOMAX)));
 delay(5000);

}
