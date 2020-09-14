#include"aranha.h"
#include<Arduino.h>

#define NUMPATAS 4
#define NUMMOTORES 3
#define C_1 0
#define C_2 1
#define C_3 2
#define C_4 3

#define J_1 4
#define J_2 5
#define J_3 7
#define J_4 6

#define P_1 8
#define P_2 9
#define P_3 10
#define P_4 11

#define C_1_OFF -7
#define C_2_OFF 0
#define C_3_OFF 5
#define C_4_OFF 0

#define J_1_OFF -3
#define J_2_OFF 7
#define J_3_OFF 4
#define J_4_OFF 3

#define P_1_OFF 0
#define P_2_OFF 2
#define P_3_OFF 2
#define P_4_OFF 0

int ligacoes[NUMPATAS*NUMMOTORES]={C_1,C_2,C_3,C_4,J_1,J_2,J_3,J_4,P_1,P_2,P_3,P_4};
int offsets[NUMPATAS*NUMMOTORES]={C_1_OFF,C_2_OFF,C_3_OFF,C_4_OFF,J_1_OFF,J_2_OFF,J_3_OFF,J_4_OFF,P_1_OFF,P_2_OFF,P_3_OFF,P_4_OFF};
aranha aranha1(NUMPATAS,NUMMOTORES,ligacoes,offsets);

void setup() {
  Serial.begin(9600);
  aranha1.begin();


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
    for(int i=0;i<=3;i++)
    {
      aranha1.frente(i,4, 10,30,500);
      delay(200);
      // aranha.moveAngle(aranha.pata[i],125,aranha.pata_offs[i]);
      // delay(500);
    }
    // delay(2000);
    // for(int i=0;i<=3;i++)
    // {
    //   aranha.movePolar(i,7,15,90);
    //   // aranha.moveAngle(aranha.pata[i],125,aranha.pata_offs[i]);
    //   // delay(500);
    // }
    // delay(2000);
  // }

  // delay(2000);
  //aranha.movePolar(1,15,3,30);
  //aranha.movePolar(2,5,12,-45);
  //aranha.movePolar(3,10,8,90);
  // delay(10000);
  //aranha.moveAngle(0,45);
  //delay(5000);
}
