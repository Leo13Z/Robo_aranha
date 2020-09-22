const int pinoFototransistor = A7; //PINO ANALÓGICO UTILIZADO PELO FOTOTRANSISTOR
const float alpha = 0.1;
double data;
double data_filtered[] = {0, 0};
const int n = 1;

               
void setup(){
  Serial.begin(9600); //INICIALIZAÇÃO DA SERIAL
   pinMode(pinoFototransistor, INPUT); //DEFINE O PINO COMO ENTRADA
}
 
void loop(){
  data = analogRead(pinoFototransistor);
  // Low Pass Filter
    data_filtered[n] = alpha * data + (1 - alpha) * data_filtered[n-1];

    // Store the last filtered data in data_filtered[n-1]
    data_filtered[n-1] = data_filtered[n];
    // Print Data
    Serial.println(data_filtered[n]);

    delay(10);

    
//  double valorLido = 0;
//  for (int i=0;i<5000;i++)
//  {
//    valorLido += analogRead(pinoFototransistor);
//  }
//  valorLido = valorLido/5000;
////  Serial.println(analogRead(pinoFototransistor));
//  Serial.println(valorLido);
    
  
 }
