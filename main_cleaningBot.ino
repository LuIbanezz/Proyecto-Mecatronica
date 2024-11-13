/***
Cosas:
Esto esta basado en lo que pensamos con Lu: incheckeable todavia
La logica autoPilot es bastante bloqueante y solo puse que se consideren obstaculos en forward_till_edge
Fijarnos como hacer que no sea bloqueante el modo AUTO
CAMBIAR LO DE LOS SENSORES (CUANDO TENGA LOS NROS CORRECTOS PARA QUE SIEMPRE HAGA EL READ, fleta readSensors, no sirve)
***/

#include <TimerOne.h>
#include <Wire.h> 
#include <SPI.h>
#include <RF24.h>

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

IR_Sensors sens;

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

/*** State Machine Setup ***/
volatile int state = 0;
int prevState = 0;
enum STATE_MACHINE = {AUTO, MANUAL, RIGHT_T, LEFT_T, FORWARD_M, BACKWARD_M, STOP};

/*** Other ***/
enum WHEEL_DIRECTIONS = {FORWARD, BACKWARD};

bool manual = true;


/****************************************************************/
/****************************************************************/

/*** MAIN ***/
void setup() {

  Serial.begin(9600);
  Wire.begin();
  delay(2000);
  Serial.println("Serial Open");

   // Motor Setup
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

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

  delay(1000);

}

/****************************************************************/
/****************************************************************/

void loop() {

   // Reinicia el radio si no ha recibido nada por más de 5 segundos
  if (millis() - lastReceived > 5000) {
    Serial.println("Reiniciando radio por inactividad.");
    radio.stopListening();
    radio.startListening();
    lastReceived = millis();
  }

  state = receivedCommand;
  // State Machine
  switch (state) {
    case AUTO:
      manual = false;
      if(prevState!=AUTO){
        autoPilotStart();
      }
      autoPilot();
      break;
    case MANUAL:
      manual = true;
    case RIGHT_T:
      if (manual)
        turn_right();
      break;
    case LEFT_T:
      if (manual)
        turn_left();
      break;
    case FORWARD_M:
      if (manual)
        forward();
      break;
    case BACKWARD_M:
      if (manual)
        backward();
      break;
    case STOP:
      if (manual)
        stop();
      break;
    default:
      break;
  }
  prevState = state;
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
  //Serial.print("Comando recibido: ");
  //Serial.println(receivedCommand);
}

/****************************************************************/
/****************************************************************/

/*** Funciones globales ***/
void stop(){
  stopRightWheel();
  stopLeftWheel();
}

void turn_left(){
  turnRightWheel(BACKWARD);
  turnLeftWheel(FORWARD);
}

void turn_right(){
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

void autoPilotStart(){
  // Actualizo sensores : 1 si no detecta, 0 si detecta
  readIRSensors();

  forward();
  if(!sens.IF && sens.DF){ //caso inclinado a la derecha
    stop();
    while(!(sens.IF && sens.DF)){
      turn_left_deg(5);
      if(sens.IF && sens.DF){
        move_forward(5);
        if(sens.IF && sens.DF){
          break;
        }
      }  
    }
    turn_left_deg(90);
    forward_till_edge();
    turn_left_deg(90);
  }
  else if(!sens.IF && !sens.DF){ //caso recto
    stop();
    turn_left_deg(90);
    forward_till_edge();
    turn_left_deg(90);
  }
  else if(sens.IF && !sens.DF){ //caso inclinado a la izquierda
    stop();
    while(!(sens.IF && sens.DF)){
      turn_right_deg(5);
      if(sens.IF && sens.DF){
        move_forward(5);
        if(sens.IF && sens.DF){
          break;
        }
      }  
    }
    forward_till_edge();
    turn_right_deg(180);
    forward_till_edge();
    turn_left_deg(90);
    forward_till_edge();
    turn_left_deg(90);
  }
}

void autoPilot(){ //TODO por ahora esto es muy secuencial, no permitiria que nada lo interrumpa no?
  readIRSensors();
  forward_till_edge();
  turn_left_deg(180);
  forward_till_edge();
  turn_right_deg(180);
  forward_till_edge();
  if((sens.IF && sens.DF) && (sens.IL || sens.DL)){
    if(sens.IL && !sens.DF && !sens.IL && !sens.DL){
      turn_left_deg(180);
    }
    else if(!sens.IL && !sens.DF && !sens.IL && sens.DL){
      turn_right_deg(180);
      forward_till_edge();
      turn_left_deg(180);
    }
  }
  else{
    turn_left_deg(180);
    forward_till_edge();
    if((sens.IF && sens.DF) && (sens.IL || sens.DL)){
      turn_right_deg(180);
    }
  }
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
      digitalWrite(enA,120);
      break;
    case BACKWARD:
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
      digitalWrite(enA,120);
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
      digitalWrite(enB,120);
      break;
    case BACKWARD:
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
      digitalWrite(enB,120);
      break;
    default:
      break;
  }
}

void turn_right_deg(int deg){
  pulseCountA = 0;
  while(pulseCountA <= (deg * (0.6/90) * 417))
  {
      turn_right();
  }
  stop();
}

void turn_left_deg(int deg){
  pulseCountB = 0;
  while(pulseCountB <= (deg * (0.6/90) * 417))
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

void forward_till_edge(){
  while(!sens.IF && !sens.DF){
    forward_detectObstacles();
  }
  stop();
}

void forward_detectObstacles(){ //revisar, no creo que ande esto

  int counter = 0;

  forward();
  if(!sens.FIA || !sens.FDA){ //Objeto al frente
    if(sens.IL && sens.DL){
      turn_right_deg(90);
      while(sens.IA){
        move_forward(5);
        counter++;
      }
      turn_left_deg(90);
      if(sens.IF || sens.DF){
        while(counter--){
          move_forward(5);
        }
        turn_right_deg(90);
        break;
      }
      else{
        break;
      }
    }
    else if(sens.IL && !sens.DL){
      if(sens.IL && sens.DL){
        turn_left_deg(90);
        while(sens.DA){
          move_forward(5);
          counter++;
        }
        turn_right_deg(90);
        if(sens.IF || sens.DF){
          while(counter--){
            move_forward(5);
          }
          turn_left_deg(90);
          break;
        }
        else{
          break;
        }
    }
    else if(!sens.IL && sens.DL){
      turn_right_deg(90);
      while(sens.IA){
        move_forward(5);
        counter++;
      }
      turn_left_deg(90);
      if(sens.IF || sens.DF){
        while(counter--){
          move_forward(5);
        }
        turn_right_deg(90);
        break;
      }
      else{
        break;
      }  
    }
  }
}

void readIRSensors(){
  sens.IL = digitalRead(sens1); //TODO : Verificar numeros
  sens.IF = digitalRead(sens2);
  sens.DF = digitalRead(sens3);
  sens.DL = digitalRead(sens4);
  sens.IA = digitalRead(sens5);
  sens.FIA = digitalRead(sens6);
  sens.FDA = digitalRead(sens7);
  sens.DA = digitalRead(sens8);
}

/****************************************************************/
/****************************************************************/