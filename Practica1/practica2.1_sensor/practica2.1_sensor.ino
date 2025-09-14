#include <NewPing.h>
#define PINTRG 6
#define PINECHO 7
#define MAXDISTANCE 200

float tiempoInicio = 0;
float tiempoFin = 0;
float velocidadSonido = 29.287;

NewPing sonar (PINTRG, PINECHO, MAXDISTANCE);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PINTRG, OUTPUT);
  pinMode(PINECHO, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  tiempoInicio = micros();  // Marca de tiempo inicial
  float tiempo = sonar.ping(35) ;
  float distancia = tiempo / (velocidadSonido*2);
  Serial.println(distancia);
  tiempoFin = micros();  // Marca de tiempo final
  Serial.println(tiempoFin - tiempoInicio);
  delayMicroseconds(15000);
  delayMicroseconds(5000 - (tiempoFin - tiempoInicio));


  
}
