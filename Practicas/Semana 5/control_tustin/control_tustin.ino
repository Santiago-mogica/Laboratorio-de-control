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


int i = 0; int j = 0; int indice = 0;
unsigned long tiempoInicio, tiempoFin, tiempoPrueban;
float datos[] = {0,0}; // x, y
float titas[] = {0,0,0,0,0};
float titasSesgo[] = {0,0,0,0,0};
float tita_barra = 0;
float tita_servo_prev = 0;
float tita_servo = 0;
float referencia = 0;
float error = 0;
float error_prev = 0;


// Parámetros del PI
float Kp = 2.9087;
float Ki = 18.3554;


void setup(void) {
  Serial.begin(115200);
  delay(100);


  // inicializo el servo en 0°
  servo.attach(PINSERVO);
  // Mapeo   0º---> 540us
  //       180º---> 2400us
  t_us = 540 + (long)angulo * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);

  delay(300);  //Le doy tiempo para que vaya a cero

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
 
  //obtengo el sesgo
  for(int k = 0; k < 50; k++){
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);


    titasSesgo[0] += g.gyro.x *(180/PI) *TS;                               //tita_g
    titasSesgo[1] = atan2(a.acceleration.y, a.acceleration.z ) *(180/PI);  //tita_a
    titasSesgo[2] = titasSesgo[3] + g.gyro.x *(180/PI) *TS;
    titasSesgo[3] = (alfa * titasSesgo[1]) + ((1-alfa) *titasSesgo[2]);
    sesgo += titasSesgo[3];
  }
  sesgo = sesgo/50;
}

void loop() {

  tiempoInicio = micros();
 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  titas[0] += g.gyro.x *(180/PI) *TS;                               //tita_g
  titas[1] = atan2(a.acceleration.y, a.acceleration.z ) *(180/PI);  //tita_a
  titas[2] = titas[3] + g.gyro.x *(180/PI) *TS;
  titas[3] = (alfa * titas[1]) + ((1-alfa) *titas[2]);
 
  tita_barra = titas[3] - sesgo;
  error =  tita_barra -referencia ;
  //if (abs(error) < 0.05) error = 0; // ignorar errores < 0.5°

  tita_servo = tita_servo_prev + Kp*(error -error_prev) + (Ki*TS/2)*(error + error_prev);
  
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" | tita_servo: ");
  Serial.println(tita_servo);



  if (tita_servo > 90)  tita_servo = 90;
  if (tita_servo < -90) tita_servo = -90;


  t_us = 540 + (tita_servo + 90)  * (2400 - 540) / 180;
  servo.writeMicroseconds(t_us);
  //Actualizo el error y la accion de control
  tita_servo_prev = tita_servo;
  error_prev = error;


  datos [0] = error;
  datos [1] = tita_servo ;


  if(i%3 == 0){
    //matlab_send(datos, 2);  
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



