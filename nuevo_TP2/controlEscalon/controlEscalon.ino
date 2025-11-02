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

int i = 0; int j = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float datos[] = {0,0}; // x, y
float titas[] = {0,0,0,0,0};
float titasSesgo[] = {0,0,0,0,0};
float entradaEscalon[] = {10, 20}; // en cm
float tita_barra = 0, tita_servo_prev = 0, tita_servo = 0;
//float referencia = 16; //en cm
float referencia = 10; //para que no me pase valor basura en la medicion del escalon
float error = 0, error_acum =0, error_ant = 0;
float kp = 0, ki =0, kd =0, k0, T0;
float uk = 0, Ts = 0.02;
float ik=0, ik_ant =0, dk=0, dk_ant =0;
// init sensor
float posicion = 0, posicion_ant = 15.85, tiempo_ping = 0; 
float velocidadSonido = 29.287;
unsigned long tiempoUltimoCambio = 0;
int indiceEscalon = 0;
float intervaloEscalon = 10000; // 5 segundos en ms

NewPing sonar (PINTRG, PINECHO, MAXDISTANCE);
void setup(void) {
	Serial.begin(115200);
	delay(100);

  // inicializo el servo en 0°
  servo.attach(PINSERVO);
  delay(1000);
  // Mapeo   90º---> 1500us
  //         45º---> 1000us
  t_us = 11.11 * (90) + 500;
  //servo.writeMicroseconds(t_us);
  Serial.println("Initializing servo at 90°");
  delay(1000);

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
  delay(100);

  //obtengo el sesgo
  for(int k = 0; k < 50; k++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); 

    titasSesgo[0] += g.gyro.x *(180/PI) *TS;                               //tita_g
    titasSesgo[1] = atan2(a.acceleration.y, a.acceleration.z ) *(180/PI);  //tita_a
    titasSesgo[2] = titas[3] + g.gyro.x *(180/PI) *TS; 
    titasSesgo[3] = (alfa * titas[1]) + ((1-alfa) *titas[2]); 
    sesgo += titasSesgo[3];
  }
  sesgo = sesgo/50;
  delay(100);
}


void loop() {

  // lectura de las variables y filtro complementario
  tiempoInicio = micros();

  // Cambiar referencia cada 5 segundos
  unsigned long tiempoActual = millis();
  if((tiempoActual - tiempoUltimoCambio) >= intervaloEscalon){
    indiceEscalon = (indiceEscalon + 1) % 2;  // Alterna entre 0 y 1
    referencia = entradaEscalon[indiceEscalon];
    tiempoUltimoCambio = tiempoActual;
  }

  // --- resto del loop ---
  tiempoInicio = micros();

  // lectura del sensor de distancia
  tiempo_ping = sonar.ping(35) ;
  posicion = tiempo_ping / (velocidadSonido*2); //en cm
  error = referencia - posicion;
  datos[0] = posicion;

  // ==== control backwards
  // kp = k0 *0.6; ki =0; kd = 3 * k0 * T0 / 40;
  // con P: 4.3
  // con PI : kp = ; ki =; kd = ;
  // con PD = ; ki = ; kd = ;
  //kp = 0.115; ki =0; kd = 0;
  //error_acum += Ts*error;
  //uk = (kp * error) + (ki * error_acum) + (kd * (error - error_ant)/Ts);
  //error_ant = error;

  //if (uk > 30)  uk = 30;
  //if (uk < -30) uk = -30;
  //  ====

  // === control bilineal

  kp = 1.8; ki =0; kd = 0.002;

  ik = ik_ant + (Ts*error/2) + (Ts*error_ant/2);
  dk = 2*(error - error_ant)/Ts - dk_ant;
  uk = error * kp + ki * ik + kd * dk;
  error_ant = error;
  ik_ant = ik;
  dk_ant = dk;

  if (uk > 30)  uk = 30;
  if (uk < -30) uk = -30;

  // ===
  
  //Serial.println(posicion);
  // Mapeo   90º---> 1500us
  //         45º---> 1000us
  t_us = 11.11 * (uk+90) + 500;
  servo.writeMicroseconds(t_us);


  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);

  //titas[0] += g.gyro.x *(180/PI) *TS;                               //tita_g
  //titas[1] = atan2(a.acceleration.y, a.acceleration.z ) *(180/PI);  //tita_a
  //titas[2] = titas[3] + g.gyro.x *(180/PI) *TS; 
  //titas[3] = (-1)*(alfa * titas[1]) + ((1-alfa) *titas[2]);
  
  //tita_barra = titas[3] - sesgo;
  datos[1] = referencia;
  matlab_send(datos, 2);

  tiempoFin = micros();
  unsigned long tiempoTranscurrido = tiempoFin - tiempoInicio;
  if(tiempoTranscurrido < 20000) {  // 20ms in microseconds
    delayMicroseconds((5000 - tiempoTranscurrido));  //5ms - t_transc
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