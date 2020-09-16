#include"spider.hpp"

spiderbot aranha;

void setup() {
  Serial.begin(9600);
  aranha.begin();
  // aranha.setToInitialPosition();
}


void loop() {
  // for(float i=0;i<=15;i++)
  // {
  //   aranha.movePolar(0,i,7,30);
  //   delay(500);
  // }
  //
  // delay(2000);
  // for(float i=0;i<=18;i++)
  // {
    // aranha.movePolar(0,16,0,30);
    // delay(5000);
    // aranha.movePolar(0,17,0,30);
    // delay(5000);
    // aranha.movePolar(0,17.5,0,30);
    // delay(5000);
    // aranha.movePolar(0,18,0,30);
    // aranha.posInicial();
    // delay(500);
    for(int j=0;j<=3;j++){
    // for(int i=1;i<=2;i++){
      // aranha.moveLimbPolar(robot0.limbs[i],8, 5, 90);
      aranha._moveJointAngle(robot0.limbs[j].joints[0],90);
      aranha._moveJointAngle(robot0.limbs[j].joints[1],150);
      // delay(50);
      aranha._moveJointAngle(robot0.limbs[j].joints[2],180);
    }

    delay(5000);
  // }
    // delay(2000);
    for(int j=0;j<=3;j++)
    {
      // delay(100);
      aranha._moveJointAngle(robot0.limbs[j].joints[1],90);
      aranha._moveJointAngle(robot0.limbs[j].joints[2],180);
    }
    delay(2000);
    for(int j=0;j<=3;j++)
    {
      // delay(100);
      aranha._moveJointAngle(robot0.limbs[j].joints[1],45);
      aranha._moveJointAngle(robot0.limbs[j].joints[2],135);
    }
    delay(3000);
    while(1){
  //   aranha._moveJointAngle(robot0.limbs[0].joints[1],135);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[0].joints[1],45);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[1].joints[1],135);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[1].joints[1],45);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[2].joints[1],135);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[2].joints[1],45);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[3].joints[1],135);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[3].joints[1],45);
  //   delay(100);
  //   //
  //   //
  //   //
  //   //
  //   //
  //   aranha._moveJointAngle(robot0.limbs[0].joints[0],135);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[1].joints[0],45);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[2].joints[0],135);
  //   delay(100);
  //   aranha._moveJointAngle(robot0.limbs[3].joints[0],45);
  //   delay(100);
  for(int j=0;j<=3;j++)
  {
    aranha._moveJointAngle(robot0.limbs[j].joints[0],45);
    delay(100);
    aranha._moveJointAngle(robot0.limbs[j].joints[0],90);
    delay(100);
  }
  }

    delay(1000);
    for(int j=0;j<=3;j++)
    {
      aranha._moveJointAngle(robot0.limbs[j].joints[1],90);
      aranha._moveJointAngle(robot0.limbs[j].joints[2],180);
    }
    delay(5000);
  // }

  // delay(2000);
  //aranha.movePolar(1,15,3,30);
  //aranha.movePolar(2,5,12,-45);
  //aranha.movePolar(3,10,8,90);
  // delay(10000);
  //aranha.moveAngle(0,45);
  //delay(5000);
}
