class aranha
{
public:
  aranha(int numpata,int nummotores,int *lig,int*offs);
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
