#include"spider.hpp"

spiderbot aranha;

void setup() {
  Serial.begin(9600);
  aranha.begin();
  aranha.setToInitialPosition();
  delay(3000);
}


void loop() {
  // delay(2000);
  // // aranha.standUp();
  // // delay(5000);
  // // aranha.rest();
  // aranha.goDoggy();
  // delay(500);
  aranha.riseSlowly();
  delay(1000);
  aranha.rest();
  delay(1000);
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
