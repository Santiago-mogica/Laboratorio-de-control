#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Math.h>
#include <Servo.h>

// init mpu
Adafruit_MPU6050 mpu;
#define PINSCL SCL 
#define PINSDA SDA
#define PI 3.1415926536
#define TS 0.02
#define alfa 0.5

// init servo
#define CTEPROP 3.76666666666666
#define PINSERVO 9
float t_us = 0;
float angulo = 90; float sesgo = 0;
Servo servo;

int i = 0; int j = 0; int indice = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float datos[] = {0,0}; // x, y
float titas[] = {0,0,0,0,0};
float titasSesgo[] = {0,0,0,0,0};
float angulosEscalon[] = {120, 90, 60}; // en grados

void setup(void) {
	Serial.begin(115200);
	delay(100);

  // inicializo el servo en 0°
  servo.attach(PINSERVO);
  // Mapeo   0º---> 540us
  //       180º---> 2400us
  t_us = 540 + (long)angulo * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);

  //inicializo la IMU, tendía que leer 0°
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  
}


void loop() {

// servo write angulosEscalon - lectura del angulo con el mpu - lo pasamos a matlab, leemos la rta transitoria.


  // Matlab send
  tiempoInicio = micros();

  //servo write cada 150 iteraciones del ciclo osea 3 segundos


  indice++;
  t_us = 540 + (long)angulosEscalon[indice%3] * (2400 - 540) / 180;
  tiempoInicio = micros();
  servo.writeMicroseconds(t_us);
  tiempoFin = micros();

   unsigned long tiempoTranscurrido = tiempoFin - tiempoInicio;
  Serial.println(tiempoTranscurrido);

  if(tiempoTranscurrido < 20000) {  // 20ms in microseconds
    delayMicroseconds((5000 - tiempoTranscurrido));  
    delay(15);
  }
  
}


