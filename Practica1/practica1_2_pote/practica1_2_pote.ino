#define PINPOTE A0
#define CTEPROP 3.76666666666666


float angulo = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(PINPOTE, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pote= analogRead(A0);
  angulo = pote/CTEPROP;
  Serial.println(angulo);
  delay(200);


  
}
