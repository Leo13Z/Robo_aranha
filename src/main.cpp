#include"aranha.hpp"

int ligacoes[NUMPATAS*NUMMOTORES]={C_1,C_2,C_3,C_4,J_1,J_2,J_3,J_4,P_1,P_2,P_3,P_4};
int offsets[NUMPATAS*NUMMOTORES]={C_1_OFF,C_2_OFF,C_3_OFF,C_4_OFF,J_1_OFF,J_2_OFF,J_3_OFF,J_4_OFF,P_1_OFF,P_2_OFF,P_3_OFF,P_4_OFF};
aranha aranha1;

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
