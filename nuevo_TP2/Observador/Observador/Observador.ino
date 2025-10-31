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

int i = 0; int j = 0; int indice = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float datos[] = {0,0,0,0};
float titas[] = {0,0,0,0,0};
float entradaEscalon[] = {-15, 15}; // en grados
float tita_barra = 0, tita_servo_prev = 0, tita_servo = 0;
float referencia = 15.85; //en cm 
float error = 0, error_acum =0, error_ant = 0;
float kp = 0, ki =0, kd =0, k0, T0;
float uk = 0, Ts = 0.02;
// init sensor
float posicion = 0, tiempo_ping = 0; 
float velocidadSonido = 29.287;

//semana 9
// Matriz A (2x2)

float A_d[2][2] = {
  {1, 0.02},
  {-10.7660,  0.3392}
};

// Matriz B (2x1)

float B_d[2][1] = {
  {0},
  {3.6960}
};
// Matriz C (1x2)
float C_d[1][2] = {
  {1, 0}
};

// Matriz D (1x2)
float D_d[1][2] = {
  {0, 0}
};

// Matriz L (observer gain as column vector 2x1)
float L[2] = {0.9242, -5.1651};

//estados
float estados[] = {0,0}; // {tita , tita_punto}
float estados_sig[] = {0,0}; // {tita_punto ; tita_puntopunto}
float u = 0, y = 0, y_estimado = 0; 

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
  delay(200);
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

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  titas[0] += (g.gyro.x - bias_gyroX) *(180/PI) *TS;                               //tita_g
  titas[1] =  atan2((a.acceleration.y - bias_accY), (a.acceleration.z))*(180/PI); 
  titas[2] = titas[3] + (g.gyro.x - bias_gyroX) *(180/PI) *TS; 
  titas[3] = ((alfa * titas[1]) + ((1-alfa) *titas[2]));
  tita_barra =(-1)* (titas[3]);
  
  y = tita_barra;
  

  // u es lo que le mandamos al servo
  u = entradaEscalon[j%2] ;
  t_us = 540 + (u+90)  * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);

// ----- Calcular ŷk = Cd * x̂k + Dd * uk -----
  y_estimado = C_d[0][0] * estados[0] + C_d[0][1] * estados[1] ;

// ----- Calcular el error de observación -----
  float e = y - y_estimado;

//----- Calcular x̂(k+1) = Ad*x̂ + L*e + Bd*u -----
  estados_sig[0] = A_d[0][0]*estados[0] + A_d[0][1]*estados[1] + B_d[0][0]*(u) + L[0]*e;
  estados_sig[1] = A_d[1][0]*estados[0] + A_d[1][1]*estados[1] + B_d[1][0]*(u) + L[1]*e;

// ----- Actualizar estado -----
  estados[0] = estados_sig[0];
  estados[1] = estados_sig[1];


  datos[0] = estados[0];  //tita estimado
  datos[1] =  estados[1]; // tita_punto estimado
  datos[2] = tita_barra; // tita medida
  datos[3] = (g.gyro.x - bias_gyroX)*(-180/PI) ; //tita_punto medido

  
  if(i%150 == 0){
    j++;  
  }

  if(i%2 == 0){
    matlab_send(datos,4);  
  }

  tiempoFin = micros();
  unsigned long tiempoTranscurrido = tiempoFin - tiempoInicio;
  if(tiempoTranscurrido < 20000) {  // 20ms in microseconds
    delayMicroseconds((5000 - tiempoTranscurrido));  
    delay(15);
  }
  i++;

  //float tiempoFin2 = micros();
  //unsigned long tiempoTranscurrido2 = tiempoFin2 - tiempoInicio;
  //Serial.println(tiempoTranscurrido2/1000.0);
  
}

void matlab_send(float* vector, int size) {
    Serial.write("abcd"); 
    for (int j = 0; j < size; j++) {
        byte* b = (byte*)&vector[j];
        Serial.write(b, sizeof(float));  // More explicit and efficient
    }
}