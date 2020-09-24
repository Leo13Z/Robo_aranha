void setup(){
  Serial.begin(9600); //INICIALIZAÇÃO DA SERIAL
  pinMode(8, INPUT); //DEFINE O PINO COMO ENTRADA
}
 
void loop(){
  Serial.println(digitalRead(8));
    
  
 }
