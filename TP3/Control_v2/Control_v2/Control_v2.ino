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
float datos[] = {0,0,0,0,0,0,0,0};
float titas[] = {0,0,0,0,0};
float entradaEscalon[] = {-5, 5}; // en grados
float tita_barra = 0, tita_servo_prev = 0, tita_servo = 0;
float referencia_fija = 15.55; //en cm 
float error = 0, error_acum =0, error_ant = 0;
float kp = 0, ki =0, kd =0, k0, T0;
float uk = 0, Ts = 0.02;
// init sensor
float posicion = 0, posicion_anterior = 0,velocidad_carrito =0, tiempo_ping = 0; 
float velocidadSonido = 29.287;
float y1 = 0, y2 = 0; 

// Matriz A_d (4x4)
float A_d[4][4] = {
  { 1.0000,   0.02,     0.0,       0.0     },
  { 0.0,     0.9453,   0.196,      0.0     },
  { 0.0,      0.0,      1.0,       0.02    },
  { 0.0,      0.0,    -10.7660,   0.3392   }
};

// Matriz B_d (4x1)
float B_d[4][1] = {
  { 0.0      },
  { 0.0      },
  { 0.0      },
  { 3.696 }
};

// Matriz C_d (2x4)
float C_d[2][4] = {
  { 1.0, 0.0, 0.0, 0.0 },
  { 0.0, 0.0, 1.0, 0.0 }
};

// Matriz D_d (2x1)
float D_d[2][1] = {
  { 0.0 },
  { 0.0 }
};

// Matriz L (ganancia del observador, 4x2)
float L[4][2] = { 
  {   1.0560,   -0.5249 },
  {  4.9376,   -2.8050 },
  {  0.0672 ,   0.5643 },
  {  -2.6331 , -17.6447 }
};


//float K[4] = { 1.327624, 0.71637, -0.174823, 0.036391 }; 
float K[4] = { 1.327624, 0.71637, -0.174823, 0.046391 }; 

float F = 1.3276;


//estados
float estados[] = {0,0,0,0}; // {posicion, posicion_punto ,tita , tita_punto}
float estados_sig[] = {0,0,0,0}; // {posicionpunto, posicion_puntopunto ,titapunto , tita_puntopunto}
float u = 0, y = 0, y_estimado = 0; 

NewPing sonar (PINTRG, PINECHO, MAXDISTANCE);

void setup(void) {
	Serial.begin(115200);
	delay(100);

  // inicializo el servo en 0°
  servo.attach(PINSERVO);
  // Mapeo   0º---> 540us
  //       180º---> 2400us
  t_us = 570 + (long)angulo * (2400 - 540) / 180;
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

  tiempo_ping = sonar.ping(35) ;
  posicion = (tiempo_ping / (velocidadSonido*2)) - referencia_fija; //en cm  
  y1 = posicion;
  velocidad_carrito = (posicion-posicion_anterior)/TS;
  posicion_anterior = posicion;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  titas[0] += (g.gyro.x - bias_gyroX) *(180/PI) *TS;                               //tita_g
  titas[1] =  atan2((a.acceleration.y - bias_accY), (a.acceleration.z))*(180/PI); 
  titas[2] = titas[3] + (g.gyro.x - bias_gyroX) *(180/PI) *TS; 
  titas[3] = ((alfa * titas[1]) + ((1-alfa) *titas[2]));
  tita_barra =(-1)* (titas[3]);
  
  y2 = tita_barra;
  

// ========= CONTROLADOR ==========
float u_calc = 0;

// K es un vector de 4 elementos, estados tambien.
// Esto produce un solo numero u.
for (int idx = 0; idx < 4; idx++) {
    u_calc += K[idx] * estados[idx]; // 
}

float ref = 8;
u = -u_calc + F*ref;  
//Serial.println(u);

// Saturación
float umax = 30.0;
if (u > umax) u = umax;
if (u < -umax) u = -umax;

t_us = 570 + (u+90)  * (2400 - 540) / 180;
servo.writeMicroseconds(t_us);


/// ----- Calcular ŷ(k) = C_d * x̂(k) -----
float y1_est = C_d[0][0]*estados[0] + C_d[0][1]*estados[1] + C_d[0][2]*estados[2] + C_d[0][3]*estados[3];
float y2_est = C_d[1][0]*estados[0] + C_d[1][1]*estados[1] + C_d[1][2]*estados[2] + C_d[1][3]*estados[3];

// ----- Calcular errores de observación -----
float e1 = y1 - y1_est;
float e2 = y2 - y2_est;

// ----- Calcular x̂(k+1) = A_d*x̂ + B_d*u + L*e -----
float estados_sig[4];

// Posicion
estados_sig[0] = A_d[0][0]*estados[0] + A_d[0][1]*estados[1] + A_d[0][2]*estados[2] + A_d[0][3]*estados[3]
               + B_d[0][0]*u
               + L[0][0]*e1 + L[0][1]*e2;

// Posicion_punto
estados_sig[1] = A_d[1][0]*estados[0] + A_d[1][1]*estados[1] + A_d[1][2]*estados[2] + A_d[1][3]*estados[3]
               + B_d[1][0]*u
               + L[1][0]*e1 + L[1][1]*e2;

// Tita
estados_sig[2] = A_d[2][0]*estados[0] + A_d[2][1]*estados[1] + A_d[2][2]*estados[2] + A_d[2][3]*estados[3]
               + B_d[2][0]*u
               + L[2][0]*e1 + L[2][1]*e2;

// Tita_punto
estados_sig[3] = A_d[3][0]*estados[0] + A_d[3][1]*estados[1] + A_d[3][2]*estados[2] + A_d[3][3]*estados[3]
               + B_d[3][0]*u
               + L[3][0]*e1 + L[3][1]*e2;

// ----- Actualizar estado -----
estados[0] = estados_sig[0];
estados[1] = estados_sig[1];
estados[2] = estados_sig[2];
estados[3] = estados_sig[3];


datos[0] = estados[0]; //Posicion
datos[1] = estados[1]; //Posicion_punto
datos[2] = estados[2]; //tita 
datos[3] = estados[3]; //tita_punto 
datos[4] = posicion;                //Posicion medido
datos[5] = velocidad_carrito; //Posicion_punto medido
datos[6] = y2;                //tita medido
datos[7] = (g.gyro.x - bias_gyroX)*(-180/PI); //tita_punto medido

  
  if(i%15 == 0){
    j++;  
  }

  if(i%2 == 0){
    matlab_send(datos,8);  
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