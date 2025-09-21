#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;
#define PINSCL SCL 
#define PINSDA SDA

int i = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueba;
float vector[]={1,2,3,4,5,6};

void setup(void) {
	Serial.begin(115200);
	delay(100);

  //inicializo la IMU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
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
  vector[0] = a.acceleration.x;
  vector[1] = a.acceleration.y;
  vector[2] = a.acceleration.z;
  vector[3] = g.gyro.x;
  vector[4] = g.gyro.y;
  vector[5] = g.gyro.z;

  if(i%3 == 0){
    matlab_send(vector, 6);  // Fixed size, no calculation needed
  }else{
    //espacio por si queremos que haga otra cosa mientras
  }

  tiempoFin = micros();
  unsigned long tiempoTranscurrido = tiempoFin - tiempoInicio;
  if(tiempoTranscurrido < 20000) {  // 20ms in microseconds
    delayMicroseconds((50000 - tiempoTranscurrido));  
    delay(15);
  }
  i++;
  //tiempoPrueba -= tiempoInicio;
  //Serial.println("Tiempo del get datos :");
  //Serial.println(tiempoPrueba);
  //Serial.println("\n");
}

void matlab_send(float* vector, int size) {
    Serial.write("abcd"); // header or marker
    for (int j = 0; j < size; j++) {
        byte* b = (byte*)&vector[j];
        Serial.write(b, sizeof(float));  // More explicit and efficient
    }
}
