#include <Servo.h>

#define PINSERVO 9

Servo servo;

void setup() {
  // put your setup code here, to run once:
  servo.attach(PINSERVO);


}

void loop() {
  // put your main code here, to run repeatedly:
  // Pulso de 1 ms
  servo.writeMicroseconds(1100);
  delay(1000);

  // Pulso de 1.5 ms
  servo.writeMicroseconds(1500);
  delay(1000);

  // Pulso de 2 ms
  servo.writeMicroseconds(1900);
  delay(1000);

  // Pulso min
  //servo.writeMicroseconds(540);
  //delay(1000);

  //servo.writeMicroseconds(1470);
  //delay(1000);


  // Pulso max
  //servo.writeMicroseconds(2400);
  //delay(3000);

//---------------------------------------------
//    3.2

//  servo.write(0); // angulos
//  delay(1000);

//  servo.write(90);
  //delay(1000);
  //servo.write(180);
  //delay(3000);
  }
