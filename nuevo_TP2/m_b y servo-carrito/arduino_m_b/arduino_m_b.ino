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
#define alfa 0.4

// init servo
#define CTEPROP 3.76666666666666
#define PINSERVO 9
float t_us = 0;
float angulo = 90; 
Servo servo;

const int nbias = 200; // Cantidad de iteraciones para estimar el bias
float bias_gyroX = 0; // Bias del giroscopio en X
float bias_accY = 0; // Bias del Acelerometro en Y

// init sensor
#include <NewPing.h>
#define PINTRG 6
#define PINECHO 7
#define MAXDISTANCE 200

int i = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float datos[] = {0,0}; // {posicion, tita}
float titas[] = {0,0,0,0,0};
float tita_barra = 0;
float referencia = 15.85; //en cm 
float Ts = 0.02;

// init sensor
float posicion = 0, tiempo_ping = 0; 
float velocidadSonido = 29.287;


NewPing sonar (PINTRG, PINECHO, MAXDISTANCE);
void setup(void) {
	Serial.begin(115200);
	delay(100);

  // inicializo el servo en 0°
  servo.attach(PINSERVO);
  // Mapeo   0º---> 540us
  //       180º---> 2400us
  t_us = 540 + (long)angulo * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);
  delay(100);

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
  delay(100);

  // inicializo el sesor de distancia
  pinMode(PINTRG, OUTPUT);
  pinMode(PINECHO, INPUT);

  //obtengo el sesgo
  for (int i = 0; i < nbias ; i++) {
    // Get new sensor events with the readings 
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Acumula las lecturas
    bias_gyroX += g.gyro.x;
    bias_accY += a.acceleration.y;

    delay(10); // Espera 10 ms entre lecturas
  }  
  // Calculo el sesgo promedio
  bias_gyroX = bias_gyroX/nbias; 
  bias_accY = bias_accY/nbias;

  delay(100);

  // inicio la barra en 20°
  t_us = 540 + (long)110 * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);
  delay(100);

}


void loop() {

  tiempoInicio = micros();

  //obtengo tita barra
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  titas[0] += (g.gyro.x - bias_gyroX) *(180/PI) *TS;                               //tita_g
  titas[1] =  atan2((a.acceleration.y - bias_accY), (a.acceleration.z))*(180/PI); 
  titas[2] = titas[3] + (g.gyro.x - bias_gyroX) *(180/PI) *TS; 
  titas[3] = ((alfa * titas[1]) + ((1-alfa) *titas[2]));
  tita_barra =(-1)* (titas[3]);
  
  //posicion, 
  tiempo_ping = sonar.ping(35) ;
  posicion = tiempo_ping / (velocidadSonido*2); //en cm

  //envío de datos
  datos[0] =  posicion; // posicion
  datos[1] = tita_barra; // tita medida de la barra

  if(i%1 == 0){
    matlab_send(datos,2);  
  }

  tiempoFin = micros();
  unsigned long tiempoTranscurrido = tiempoFin - tiempoInicio;
  if(tiempoTranscurrido < 20000) {  // 20ms in microseconds
    delayMicroseconds((5000 - tiempoTranscurrido));  
    delay(15);
  }
  i++;
  
}

void matlab_send(float* vector, int size) {
    Serial.write("abcd"); 
    for (int j = 0; j < size; j++) {
        byte* b = (byte*)&vector[j];
        Serial.write(b, sizeof(float));  // More explicit and efficient
    }
}