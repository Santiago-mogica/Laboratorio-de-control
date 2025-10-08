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
  error = entradaEscalon[j%3] - posicion;
  Serial.println(error);
  // ==== control backwards
  // con kp = 9.2 oscila, hace 5 oscilaciones en 8 segundos: T0 = 1.6 seg - pero supera la saturacion del sistema entonces no lo podemos usar
  k0 = 9.2; T0 = 1.6;
  // kp = k0 *0.6; ki =0; kd = 3 * k0 * T0 / 40;
  // con un proporcional puro: 4.3
  // con PD puro: kp = 2.6; ki =0; kd = 0.5;
  // PID kp = 2.8; ki =3.9; kd = 0.35;
  kp = 2.8; ki =2.2; kd = 0.15;
  error_acum += Ts*error;
  uk = (kp * error) + (ki * error_acum) + (kd * (error - error_ant)/Ts);
  error_ant = error;

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