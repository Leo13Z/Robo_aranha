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


int perna1[3]={C_1,J_1,P_1};
int perna2[3]={C_2,J_2,P_2};
int perna3[3]={C_3,J_3,P_3};
int perna4[3]={C_4,J_4,P_4};

int coxas[4]={C_1,C_2,C_3,C_4};
int joelhos[4]={J_1,J_2,J_3,J_4};
int patas[4]={P_1,P_2,P_3,P_4};





void setup() {
  pwm.begin();
  Serial.begin(9600);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  delay(50);
//  set_walk_position();


}



void loop() {

  command_by_serial();

}
