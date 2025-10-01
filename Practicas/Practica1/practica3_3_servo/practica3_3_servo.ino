#include <Servo.h>
#define PINPOTE A0
#define CTEPROP 3.76666666666666
#define PINSERVO 9
float angulo = 0;

float tiempoInicio = 0;
float tiempoFin = 0;
int t_us = 0;
Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(PINSERVO);
  pinMode(PINPOTE, INPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  tiempoInicio = micros();  // Marca de tiempo inicial

 int pote= analogRead(A0);
  angulo = pote/CTEPROP;
  // Mapeo   0º---> 540us
  //       180º---> 2400us
  t_us = 540 + (long)angulo * (2400 - 540) / 180;

  servo.writeMicroseconds(t_us);  // usar writeMicroseconds para mas precision
  tiempoFin = micros();  // Marca de tiempo final
  //50 Hz
 // delayMicroseconds(15000);
 // delayMicroseconds(5000 - (tiempoFin - tiempoInicio));

//10 Hz /100m
 // delay(80);
  //delayMicroseconds(15000);
  //delayMicroseconds(5000 - (tiempoFin - tiempoInicio));

//1 Hz /1
  delay(980);
  delayMicroseconds(15000);
  delayMicroseconds(5000 - (tiempoFin - tiempoInicio));

}
