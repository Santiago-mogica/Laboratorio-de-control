#define PINPOTE A0
#define CTEPROP 3.76666666666666


float tiempoInicio = 0;
float tiempoFin = 0;
float angulo = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PINPOTE, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  tiempoInicio = micros();  // Marca de tiempo inicial

  int pote= analogRead(A0);
  angulo = pote/CTEPROP;
  tiempoFin = micros();  // Marca de tiempo final
  Serial.println((tiempoFin - tiempoInicio));
  Serial.println(angulo);
  delayMicroseconds(15000);
  delayMicroseconds(5000 - (tiempoFin - tiempoInicio));


  
}
