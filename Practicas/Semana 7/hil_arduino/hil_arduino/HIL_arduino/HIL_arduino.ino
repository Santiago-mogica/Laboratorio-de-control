#include "TimerOne.h"

typedef union{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

// Parámetros del PI
float a = 5.8619;
float b = -5.9828;
float error = 0;
float error_prev = 0;
float u_prev = 0;
static float u0=0.5, h_ref=0.45, h=0.45, u;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  // Ajustar condiciones iniciales de trabajo

  static float Ts=1;  //segundos
  FLOATUNION_t aux;
  static float sampling_period_ms = 1000*Ts;
  //=========================
  // Definir parametros y variables del control

  //=========================

  if (Serial.available() >= 8) {
 
    aux.number = getFloat();
    h = aux.number;
    aux.number = getFloat();
    h_ref = aux.number;   /// h - href = error? 
  }
  //=========================
  //CONTROL
  error = h_ref - h ;
  //tustin de la clase anterior   
  //u = u_prev + Kp* (error -error_prev) + (Ki*Ts/2)*(error + error_prev);
  u = u_prev + (b*error) +(a*error_prev);
  u_prev = u;
  error_prev = error;
  //=========================
    
  matlab_send(u+u0,h_ref,u0);
  delay(sampling_period_ms);
}

void matlab_send(float u, float h, float u0){
  Serial.write("abcd");
  byte * b = (byte *) &u;
  Serial.write(b,4);
  b = (byte *) &h;
  Serial.write(b,4);
  b = (byte *) &u0;
  Serial.write(b,4);
}

float getFloat(){
    int cont = 0;
    FLOATUNION_t f;
    while (cont < 4 ){
        f.bytes[cont] = Serial.read() ;
        cont = cont +1;
    }
    return f.number;
}

