/*** Timer 1 setup for Sample ***/
#include <TimerOne.h>
#include <Wire.h> 


//para girar a la izquierda es 300. Para la derecha es cerca de 240
#define DELAY 2000
#define PULSES_A 240
#define PULSES_B 320
 
/*** Motor Setup ***/
const int enA = 8;
const int in1 = 28;
const int in2 = 30;  //Motor A aprox 3060 pulsos en 2 seg

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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

}

void loop() {
    // Mostrar el número total de pulsos en el monitor serie

    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
    // analogWrite(enA, 120);

    // digitalWrite(in3, LOW);
    // digitalWrite(in4, HIGH);
    // analogWrite(enB, 120);

    // delay(2000);

    // digitalWrite(in1, LOW);
    // digitalWrite(in2, LOW);
    // analogWrite(enA, 120);

    // digitalWrite(in3, LOW);
    // digitalWrite(in4, LOW);
    // analogWrite(enB, 120);

    Serial.println("Pulsos A: ");
    Serial.println(pulseCountA);
    Serial.println("Pulsos B: ");
    Serial.println(pulseCountB);


    // if (pulseCountA < PULSES_A && pulseCountB < PULSES_B)
    // {
    //   turn();
    // }
    //  else if (pulseCountA < PULSES_A && pulseCountB >= PULSES_B)
    //  {
    //    turn_right_wheel();
    //  }
    //  else if (pulseCountB < PULSES_B && pulseCountA >= PULSES_A)
    //  {
    //    turn_left_wheel();
    //  }
    if (pulseCountA < PULSES_A)
    {
      turn_right();
    }

    else
    {
      stop();
    }
}

void countPulseA() {
    pulseCountA = pulseCountA+1; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}

void countPulseB() {
    pulseCountB = pulseCountB+1; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}

void turn_left()
{
  Serial.println("girando izq");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 120);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 120);
}

void turn_right()
{
  Serial.println("girando der");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 120);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, 120);
}
void turn_right_wheel()
{
  Serial.println("falta derecha");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH); 
  analogWrite(enA, 120);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
}

void turn_left_wheel()
{
  Serial.println("falta izquierda");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, 0);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, 120);
}


void stop()
{
  Serial.println("Terminó de girar");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);

  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 0);
  pulseCountA = 0;
  pulseCountB = 0;
  delay(5000);
}