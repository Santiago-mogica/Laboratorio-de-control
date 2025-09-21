#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include<Math.h>
Adafruit_MPU6050 mpu;
#define PINSCL SCL 
#define PINSDA SDA
#define PI 3.1415926536
#define TS 0.02
#define alfa 0.5

int i = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float titas[] = {0,0,0,0}; // titas[0] = tita_g, tita[1] = tita_a, tita[2] = tita_estimado

void setup(void) {
	Serial.begin(115200);
	delay(100);

  //inicializo la IMU
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
//tenemos que transmitir 16 bits cada un mili, tiene que ser compatible con el I2C

void loop() {
  // Matlab send
  tiempoInicio = micros();
  sensors_event_t a, g, temp;
  //tiempoPrueba = micros();
  mpu.getEvent(&a, &g, &temp);

  titas[0] += g.gyro.x *(180/PI) *TS;                               //tita_g
  titas[1] = atan2(a.acceleration.y, a.acceleration.z ) *(180/PI);  //tita_a
  titas[2] = titas[3] + g.gyro.x *(180/PI) *TS; 
  titas[3] = (alfa * titas[1]) + ((1-alfa) *titas[2]);
  
  if(i%3 == 0){
    matlab_send(titas, 6);  
  }else{
    
  }

  tiempoFin = micros();
  unsigned long tiempoTranscurrido = tiempoFin - tiempoInicio;
  if(tiempoTranscurrido < 20000) {  // 20ms in microseconds
    delayMicroseconds((50000 - tiempoTranscurrido));  
    delay(15);
  }
  i++;
  
}

void matlab_send(float* vector, int size) {
    Serial.write("abcd"); // header or marker
    for (int j = 0; j < size; j++) {
        byte* b = (byte*)&vector[j];
        Serial.write(b, sizeof(float));  // More explicit and efficient
    }
}
