#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  140 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)

#define NUMPATAS 4
#define NUMMOTORES 3

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

//I dont know if this is correct, must check
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

 static int perna1[3]={C_1,J_1,P_1};
 static int perna2[3]={C_2,J_2,P_2};
 static int perna3[3]={C_3,J_3,P_3};
 static int perna4[3]={C_4,J_4,P_4};

 static int coxas[4]={C_1,C_2,C_3,C_4};
 static int joelhos[4]={J_1,J_2,J_3,J_4};
 static int patas[4]={P_1,P_2,P_3,P_4};

 static int offset[12] = {0, 0, 0, 0, 20, 20, 10, 20, };



class aranha
{
public:
  aranha();
  // int* coxa;
  // int* joelho;
  // int* pata;
  int coxa[4];
  int joelho[4];
  int pata[4];
  int coxa_offs[4];
  int joelho_offs[4];
  int pata_offs[4];
  //int offsets[12];
  void begin();
  void movePolar(int pata,double r,double z,double theta);
  void posInicial();
  void moveAngle(int junta, int grau,int off);
  void frente(int junta,int dist,int alt_atual,int amp,int vel);

private:
  void angulos(double x,double y, double& a1, double& a2);
};
