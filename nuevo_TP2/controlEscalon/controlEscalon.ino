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

int i = 0; int j = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float datos[] = {0,0}; // x, y
float titas[] = {0,0,0,0,0};
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
  //t_us = 11.11 * (90) + 500;
  t_us = 540 + (long)angulo * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);

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


  // === control bilineal

  kp = 1.2; ki =0; kd = 0.0003;

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
  //t_us = 11.11 * (uk+90) + 500;
  t_us = 540 + ((long)(uk + 90)) * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);


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