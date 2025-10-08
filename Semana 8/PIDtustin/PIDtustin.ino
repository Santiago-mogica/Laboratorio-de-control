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
float angulo = 90; float sesgo = 0;
Servo servo;

// init sensor
#include <NewPing.h>
#define PINTRG 6
#define PINECHO 7
#define MAXDISTANCE 200

int i = 0; int j = 0; int indice = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
//float datos[] = {0,0}; // x, y
float titas[] = {0,0,0,0,0};
float titasSesgo[] = {0,0,0,0,0};
float entradaEscalon[] = {20, 15, 10}; // en grados
float tita_barra = 0, tita_servo_prev = 0, tita_servo = 0;
float referencia = 15.85; //en cm
float error = 0, error_acum =0, error_ant = 0;
float kp = 0, ki =0, kd =0, k0, T0;
float uk = 0, Ts = 0.02;
float ik=0, ik_ant =0, dk=0, dk_ant =0;
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

  // inicializo el sesor de distancia
  pinMode(PINTRG, OUTPUT);
  pinMode(PINECHO, INPUT);
}


void loop() {

// lectura de las variables y filtro complementario
  tiempoInicio = micros();

  //Send a ping and get the echo time (in microseconds) as a result
  // le mandamos maximo cuartenta que es lo maximo de la barra
  tiempo_ping = sonar.ping(35) ;
  posicion = tiempo_ping / (velocidadSonido*2); //en cm
  error = referencia - posicion;
  Serial.println(error);
  // ==== control backwards
  // con un proporcional puro: 
  // con PD puro: 
  // PID 
  kp = 3; ki =0; kd = 0.001;

  ik = ik_ant + (Ts*error/2) + (Ts*error_ant/2);
  dk = 2*(error - error_ant)/Ts - dk_ant;
  uk = error * kp + ki * ik + kd * dk;
  error_ant = error;
  ik_ant = ik;
  dk_ant = dk;

  if (uk > 30)  uk = 30;
  if (uk < -30) uk = -30;
  //  ====

  //t_us ahora lo medimos desde la distacia de la imu
  t_us = 540 + (uk + 90)  * (2400 - 540) / 180;

  servo.writeMicroseconds(t_us);
  
  if(i%100 == 0){
    j++;  
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