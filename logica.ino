#include <TimerOne.h>
#include <Wire.h> 
#include <SPI.h>
#include <RF24.h>
#include "DualVNH5019MotorShield.h"

#define DELAY 2000
#define VEL 75

DualVNH5019MotorShield md;

/*** Declaracion de prototipos ***/
void countPulseA(); 
void countPulseB(); 
void handleRadioInterrupt();
void forward_till_edge();
void turn_left_deg(int deg);
void turn_right_deg(int deg);
void set_motor_speed(const int ina, const int inb, const int en, int speed);
void move_forward(int time);
void move_backward(int time);
void forward_detectObstacles();

/*** Drivers Setup ***/
const int csA = A9;
const int csB = A8;
/*** Motor Setup ***/

// MOTOR A = RIGHT WHEEL
const int enA = 9; //PWM
const int in1 = 30; //INA
const int in2 = 28;  // INBMotor A aprox 3060 pulsos en 2 seg

// MOTOR B = LEFT WHEEL
const int enB = 8; //PWM
const int in3 = 29; //INA
const int in4 = 31; //INB Motor B aprox 2370 pulsos en 2 seg (varia bastante :C)

/*** Sensor Setup ***/
const int sens1= 47;
const int sens2= 23;
const int sens3= 49;
const int sens4= 48;
const int sens5= 44; 
const int sens6= 39;
const int sens7= 33; 
const int sens8= 45;

// Definir la estructura para los sensores
struct IR_Sensors {
    int IL;
    int IF;
    int DF;
    int DL;    // Hasta aca apuntan a la mesa
    int IA;
    int FIA;
    int FDA;
    int DA;    // El resto apuntan a obstaculos
};

IR_Sensors sens = {sens3, sens5, sens6, sens8, sens2, sens4, sens7, sens1};

/*** Encoder Setup ***/
const int encoderA= 2;
const int encoderB= 3;

unsigned long previousMillis = 0; // Tiempo de la última medición de velocidad
const int interval = 1000; // Intervalo de tiempo para calcular la velocidad (ms)

volatile int pulseCountA = 0; // Contador de pulsos del encoder A
volatile int pulseCountB = 0; // Contador de pulsos del encoder B

/*** Remote Control Setup ***/
const int  RC_CE = 34;
const int  RC_CSN = 36;
const int  RC_IRQ = 21;

volatile int receivedCommand;

RF24 radio(RC_CE, RC_CSN); // Inicialización del módulo RF
const byte address[6] = "00001"; // Dirección del canal de comunicación
unsigned long lastReceived = millis();

unsigned long prev_time = millis();

/*** State Machine Setup ***/
volatile int state = 0;
int prevState = 0;
enum STATE_MACHINE {
  MANUAL, 
  AUTO, 
  FORWARD_M,
  BACKWARD_M,
  LEFT_T,
  RIGHT_T  
  };

/*** Other ***/
enum WHEEL_DIRECTIONS {
  FORWARD, 
  BACKWARD
  };

bool manual = true;
const int buzzer = 22;

/*******************************************************/
/*******************************************************/
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  Serial.println("Serial Open");

   // Buzzer Setup
  pinMode(buzzer, OUTPUT);

   // Motor Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
   // Inicialización de los controladores de motor

  // Encoder Setup
  interrupts();
  attachInterrupt(digitalPinToInterrupt(encoderA), countPulseA, RISING); // Interrupción en flanco ascendente
  attachInterrupt(digitalPinToInterrupt(encoderB), countPulseB, RISING); // Interrupción en flanco ascendente

  // IR sensors setup
  pinMode(sens1, INPUT);
  pinMode(sens2, INPUT);
  pinMode(sens3, INPUT);
  pinMode(sens4, INPUT);
  pinMode(sens5, INPUT);
  pinMode(sens6, INPUT);
  pinMode(sens7, INPUT);
  pinMode(sens8, INPUT);

  // Radio Setup
  if (!radio.begin()) {
    Serial.println("Error al inicializar el radio.");
    while (1); // Detener si falla la inicialización
  }
  
  radio.openReadingPipe(0, address); // Abre el canal de lectura
  radio.setPALevel(RF24_PA_LOW);     // Nivel de potencia bajo para pruebas cercanas
  radio.startListening();            // Configura el módulo en modo escucha

  pinMode(RC_IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_IRQ), handleRadioInterrupt, FALLING); // Interrupción

  Serial.println("Receptor listo para recibir señales.");

  // delay(1000);

}

void loop() {
  //Reinicia el radio si no ha recibido nada por más de 5 segundos
  if (millis() - lastReceived > 5000) {
    Serial.println("Reiniciando radio por inactividad.");
    radio.stopListening();
    radio.startListening();
    lastReceived = millis();
  }

//forward();
 state = receivedCommand;
 //state = AUTO;
  // State Machine
  switch (state) {
    case AUTO:
      //Serial.println("Caso automatico");
      manual = false;
      autoPilot();
      break;
    case MANUAL:
    //Serial.println("Caso manual");
      manual = true;
      stop();
      break;
    case RIGHT_T:
      if (manual)
        turn_right();
      break;
    case LEFT_T:
      if (manual)
        turn_left();
      break;
    case FORWARD_M:
      if(!digitalRead(sens.IF) && !digitalRead(sens.DF) && (manual))
      {
        forward();       
      }
      else
      { 
        stop();
        digitalWrite(buzzer,HIGH);
      }
      break;
    case BACKWARD_M:
      if (manual)
      {
        digitalWrite(buzzer,LOW);
        backward();
      }
        
      break;
    // case STOP:
    //   if (manual)
    //     stop();
    //   break;
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

void handleRadioInterrupt() {
  radio.read(&receivedCommand, sizeof(receivedCommand));
  lastReceived = millis(); // Actualiza el tiempo de recepción
  Serial.print("Comando recibido: ");
  Serial.println(receivedCommand);
}

/****************************************************************/
/****************************************************************/


/*** Funciones globales ***/
void stop(){
  stopRightWheel();
  stopLeftWheel();
}

void stop_stuck(){
  while(receivedCommand == AUTO){
    stop();
    digitalWrite(buzzer,HIGH);
  }
  digitalWrite(buzzer,LOW);
}

void turn_left(){
  turnRightWheel(FORWARD);
  turnLeftWheel(BACKWARD);
}

void turn_right(){
  turnRightWheel(BACKWARD);
  turnLeftWheel(FORWARD);
}

void forward(){
  turnRightWheel(FORWARD);
  turnLeftWheel(FORWARD);
  if (millis() - prev_time > 1000)
    {
      if ((analogRead(csA)>24) || (analogRead(csB)>24))
      {
        Serial.println("me frene, ayuda");
        stop_stuck(); 
      }
      prev_time = millis();
    }
}

void backward(){
  turnRightWheel(BACKWARD);
  turnLeftWheel(BACKWARD);
}


void autoPilot(){
  if(!digitalRead(sens.IF)&&!digitalRead(sens.DF) && (receivedCommand == AUTO))
  {
    Serial.println("Avanzo");
    forward_till_edge();
    if(receivedCommand == AUTO)
    {
      if(!digitalRead(sens.IL))
      {
        Serial.println("giro izq");
        turn_left_deg(90);
      }

      else if(!digitalRead(sens.DL))
      {
        Serial.println("giro der");
        turn_right_deg(90);
      }
    }
    else if (receivedCommand == MANUAL)
    {
      stop();
    }
    
  }
}

/****************************************************************/
/****************************************************************/
/*** Funciones privadas ***/

void stopRightWheel(){
  set_motor_speed(in1, in2, enA, 0);
}

void stopLeftWheel(){
  set_motor_speed(in3, in4, enB, 0);
}

void turnRightWheel(int dir){
  switch (dir) {
    case BACKWARD:
      set_motor_speed(in1, in2, enA, -VEL);
      break;
    case FORWARD:
      set_motor_speed(in1, in2, enA, VEL);
      break;
    default:
      break;
  }
}

void turnLeftWheel(int dir){
  switch (dir) {
      case BACKWARD:
        set_motor_speed(in3, in4, enB, -VEL);
        break;
      case FORWARD:
        set_motor_speed(in3, in4, enB, VEL);
        break;
      default:
        break;
    }
}

void turn_right_deg(int deg){
  pulseCountA = 0;
  while(pulseCountA <= (deg * (0.6/90) * 400)) //417
  {
      turn_right();
  }
  stop();
}

void turn_left_deg(int deg){
  pulseCountB = 0;
  while(pulseCountB <= (deg * (0.6/90) * 400))
  {
      turn_left();
  }
  stop();
}

void move_forward(int time){
  forward();
  delay(time);
  stop();
}

void move_backward(int time){
  backward();
  delay(time);
  stop();
}

void forward_till_edge(){
  while((!digitalRead(sens.IF)) && (!digitalRead(sens.DF)) && (receivedCommand == AUTO)){
    forward_detectObstacles();
    //forward();
    //Serial.println("forward till edge");
  }
  if(receivedCommand == AUTO){
    move_backward(500);
    stop();
  }
}

void forward_detectObstacles(){ 

  int counter = 0;

  forward();
  if(!digitalRead(sens.FIA) && !digitalRead(sens.FDA)){ //Objeto al frente
    Serial.println("Encontre un objeto");
    if(!digitalRead(sens.IL) && !digitalRead(sens.DL)){
      Serial.println("1");
      move_backward(100);
      turn_right_deg(40);
      
    }
    else if(!digitalRead(sens.IL) && digitalRead(sens.DL)){
      Serial.println("2");
      move_backward(100);
      turn_left_deg(40);
      
    }
    else if(digitalRead(sens.IL) && !digitalRead(sens.DL)){
      Serial.println("3");
      move_backward(100);
      turn_right_deg(40);
      
    }
    else{
      Serial.println("4");
      stop();
    }  
  }
  else if(!digitalRead(sens.FIA) && digitalRead(sens.FDA)){ //Objeto al frente
    Serial.println("Encontre un objeto a la izquierda");
    if(!digitalRead(sens.IL) && !digitalRead(sens.DL)){
      Serial.println("1");
      move_backward(100);
      turn_right_deg(40);
      
    }
    else if(!digitalRead(sens.IL) && digitalRead(sens.DL)){
      Serial.println("2");
      move_backward(100);
      turn_left_deg(40);
      
    }
    else if(digitalRead(sens.IL) && !digitalRead(sens.DL)){
      Serial.println("3");
      move_backward(100);
      turn_right_deg(40);
      
    }
    else{
      Serial.println("4");
      stop();
    }  
  }
  else if(digitalRead(sens.FIA) && !digitalRead(sens.FDA)){ //Objeto al frente
    Serial.println("Encontre un objeto a la derecha");
    if(!digitalRead(sens.IL) && !digitalRead(sens.DL)){
      Serial.println("1");
      move_backward(100);
      turn_left_deg(40);
      
    }
    else if(!digitalRead(sens.IL) && digitalRead(sens.DL)){
      Serial.println("2");
      move_backward(100);
      turn_left_deg(40);
      
    }
    else if(digitalRead(sens.IL) && !digitalRead(sens.DL)){
      Serial.println("3");
      move_backward(100);
      turn_right_deg(40);
      
    }
    else{
      Serial.println("4");
      stop();
    }  
  }
}


/****************************************************************/
/****************************************************************/
//Seteo velocidad de motores con Drivers
void set_motor_speed(const int ina, const int inb, const int en, int speed){
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
    analogWrite(en,speed * 51 / 80); // map 400 to 255
    

  if (speed == 0)
  {
    digitalWrite(ina,LOW);   // Make the motor coast no
    digitalWrite(inb,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(ina,LOW);
    digitalWrite(inb,HIGH);
  }
  else
  {
    digitalWrite(ina,HIGH);
    digitalWrite(inb,LOW);
  }
}