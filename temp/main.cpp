#include"spider.hpp"

spiderbot aranha = spider0;

void setup() {
  // delay(3000);
  Serial.begin(9600);
  // delay(3000);
  aranha.begin();
  // delay(3000);
  Serial.println("0");
  aranha.moveLimbPolar(&aranha.body0.limbs[0], 5, 4, 90);
  // delay(10000);
  Serial.println("1");
  aranha.moveLimbPolar(&aranha.body0.limbs[1], 5, 4, 90);
  delay(20000);
  // aranha.setToInitialPosition();
  delay(3000);
}


void loop() {
  // delay(2000);
  // // aranha.standUp();
  // // delay(5000);
  // // aranha.rest();
  // aranha.goDoggy();
  // delay(500);
  // aranha.assumeStablePosition();
  // delay(5000);
  aranha.creepingGait(20, 10);
  delay(5000);
  // command_by_serial_PolarPosition(aranha);
  //                                 //  r z theta
  // aranha.moveLimbPolar(robot0.limbs[0], 4, 4, 90 );
  // delay(2000);
  // aranha.moveLimbPolar(robot0.limbs[0], 4, 8, 90 );
  // delay(2000);
  // aranha.moveLimbPolar(robot0.limbs[0], 4, 4, 90 );
  // delay(2000);
  // aranha.moveLimbPolar(robot0.limbs[0], 8, 4, 90 );
  // delay(2000);
  // aranha.moveLimbPolar(robot0.limbs[0], 4, 4, 90 );
  // delay(2000);
  // aranha.moveLimbPolar(robot0.limbs[0], 4, 4, 45 );
  // delay(2000);
  // aranha.moveLimbPolar(robot0.limbs[0], 16, 0, 90 );
  // delay(2000);
}
