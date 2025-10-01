float tiempoInicio = 0;
float tiempoFin = 0;
float tiempoProcesamiento = 0; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

  tiempoInicio = millis();  // Marca de tiempo inicial

  int pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
  pote= analogRead(A0);
 
  tiempoFin = millis();  // Marca de tiempo final
  tiempoProcesamiento = (tiempoFin - tiempoInicio)/10; 
  Serial.println(tiempoProcesamiento);


  
}
