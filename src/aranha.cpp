#include"aranha.hpp"



//Tamanho dos elos
#define L1 8
#define L2 10
Adafruit_PWMServoDriver pwm;

//Define a estrutura geral do robo, passando o numero de pernas e o numero de motores por perna
//Configura as juntas e offsets, deve ser passado um vetor com uma sequencia de ligamentos indo da base até o chao, perna por perna
//Exemplo : C1,C2,C3,J1,J2,J3,P1,P2,P3 para tres pata, cada uma contendo e ligacoes (C,J e P);
aranha::aranha()
{}


//FAz a cinematica inversa pra determinar a distancia e altura da ponta do pé em relacao ao servo do joelho
void aranha::angulos(double x,double y, double& a1, double& a2)
{
  double B=sqrt((x*x)+(y*y));
  double q1=atan2((double)y,(double)x);
  q1=q1*57.2958;
  double q2= acos(((L1*L1)+(B*B)-(L2*L2))/(2*B*L1));
  q2=q2*57.2958;
  a1=q1+q2;
  a1=180-a1;
  a2=acos(((L1*L1)-(B*B)+(L2*L2))/(2*L2*L1));
  a2=a2*57.2958;
  a2=a2-55;//125 é  180 devido a  montagem
}

void aranha::moveAngle(int junta, int grau, int off)
{
  grau=grau+off;
  if((junta==coxa[0])||(junta==coxa[2])||(junta==joelho[0])||(junta==joelho[2])||(junta==pata[0])||(junta==pata[2]))
  {
    grau= 180 - grau;
  }

    pwm.setPWM(junta,0,(map(grau, 0,180,SERVOMIN,SERVOMAX)));
}

void aranha::movePolar(int membro,double r,double z,double theta)
{
  double a1,a2,a3;
  //int c= *(&coxa+membro),j=*(&joelho+membro),p=*(&pata+membro);
  angulos(z,r,a1,a2);
  a3=theta;
  moveAngle(coxa[membro],(int) a3,coxa_offs[membro]);
  moveAngle(joelho[membro],(int) a1,joelho_offs[membro]);
  moveAngle(pata[membro],(int) a2,pata_offs[membro]);
}

void aranha::posInicial()
{
  for(int i=0;i<=3;i++)
  {
    movePolar(i,5,12,90);
  }
}
void aranha::begin()//inilicializo a placa dos servos
{
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  pwm.setPWM(0,0,map(90,0 ,180 ,SERVOMIN ,SERVOMAX));
}

void aranha::frente(int junta,int dist,int alt_atual,int amp,int vel)
{
  movePolar(junta,dist,alt_atual-3,90-amp);
  delay(vel);
  movePolar(junta,dist,alt_atual,90-amp);
  delay(vel);
  movePolar(junta,dist,alt_atual,90+amp);
  delay(vel);
}
