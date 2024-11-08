#include <TimerOne.h>
#include <Wire.h> 

#define DELAY 2000

/*** Motor Setup ***/
// MOTOR A = RIGHT WHEEL
const int enA = 8;
const int in1 = 28;
const int in2 = 30;  //Motor A aprox 3060 pulsos en 2 seg

// MOTOR B = LEFT WHEEL
const int enB = 9;
const int in3 = 29;
const int in4 = 31; //Motor B aprox 2370 pulsos en 2 seg (varia bastante :C)

/*** Sensor Setup ***/
const int sens1= 47;
const int sens2= 23;
const int sens3= 52;
const int sens4= 50;
const int sens5= 53; 
const int sens6= 43;
const int sens7= 51; 
const int sens8= 45;

/*** Encoder Setup ***/
const int encoderA= 2;
const int encoderB= 3;

/*** Encoder Setup***/
unsigned long previousMillis = 0; // Tiempo de la última medición de velocidad
const int encoderPin = 13; // Pin para el Canal A del encoder
const int interval = 1000; // Intervalo de tiempo para calcular la velocidad (ms)

volatile int pulseCountA = 0; // Contador de pulsos del encoder A
volatile int pulseCountB = 0; // Contador de pulsos del encoder B

/*** Remote Control Setup ***/
const int  = 34;
const int  = 36;
const int  =  

/*** State Machine Setup ***/
volatile int state = 0;
enum STATE_MACHINE = {AUTO, RIGHT_T, LEFT_T, FORWARD_M, BACKWARD_M, STOP};

/*** Other ***/
enum WHEEL_DIRECTIONS = {FORWARD, BACKWARD};

/****************************************************************/
/****************************************************************/

/*** MAIN ***/
void setup() {

  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  Serial.println("Serial Open");

   // Motor Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Encoder Setup
  interrupts();
  attachInterrupt(digitalPinToInterrupt(encoderA), countPulseA, RISING); // Interrupción en flanco ascendente
  attachInterrupt(digitalPinToInterrupt(encoderB), countPulseB, RISING); // Interrupción en flanco ascendente

}

void loop() {
  // State Machine
  switch (state) {
    case AUTO:
      autoPilot();
      break;
    case RIGHT_T:
      turn_right();
      break;
    case LEFT_T:
      turn_left();
      break;
    case FORWARD_M:
      forward();
      break;
    case BACKWARD_M:
      backward();
      break;
    case STOP:
      stop();
      break;
    default:
      break;
  }

}

/****************************************************************/
/****************************************************************/

/*** ISRs ***/
void countPulseA() {
    pulseCountA = pulseCountA+1; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}

void countPulseB() {
    pulseCountB = pulseCountB+1; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}

/****************************************************************/
/****************************************************************/

/*** Funciones globales ***/
void stop(){
  stopRightWheel();
  stopLeftWheel();
}

void turn_right(){
  turnRightWheel(BACKWARD);
  turnLeftWheel(FORWARD);
}

void turn_left(){
  turnRightWheel(FORWARD);
  turnLeftWheel(BACKWARD);
}

void forward(){
  turnRightWheel(FORWARD);
  turnLeftWheel(FORWARD);
}

void backward(){
  turnRightWheel(BACKWARD);
  turnLeftWheel(BACKWARD);
}

void autoPilot(){
  //ALGORITMO FACHERO

}

/****************************************************************/
/****************************************************************/

/*** Funciones privadas ***/

void stopRightWheel(){
  digitalWrite(in1, LOW);
  digitalWrite(in2,LOW);
  digitalWrite(enA,0);
}

void stopLeftWheel(){
  digitalWrite(in3, LOW);
  digitalWrite(in4,LOW);
  digitalWrite(enB,0);
}

void turn_rightWheel(int dir){
  switch (dir) {
    case FORWARD:
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
      break;
    case BACKWARD:
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
      break;
    default:
      break;
  }
}

void turnleftWheel(int dir){
  switch (dir) {
    case FORWARD:
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
      break;
    case BACKWARD:
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
      break;
    default:
      break;
  }
}

/****************************************************************/
/****************************************************************/