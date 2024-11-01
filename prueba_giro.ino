/*** Timer 1 setup for Sample ***/
#include <TimerOne.h>
#include <MPU9250.h>
#include <Wire.h> 

#define DELAY 2000

/*** Motor Setup ***/
const int enA = 6;
const int in1 = 4;
const int in2 = 2;

const int enB = 5;
const int in3 = 7;
const int in4 = 8;

/*** Encoder Setup***/
//static int pulseCount = 0; // Contador de pulsos
unsigned long previousMillis = 0; // Tiempo de la última medición de velocidad
const int encoderPin_a = 18; // Pin para el Canal A del encoder
const int encoderPin_b = 13; // Pin para el Canal A del encoder
const int interval = 1000; // Intervalo de tiempo para calcular la velocidad (ms)


int pulseCount_a = 0;
int pulseCount_b = 0;
void setup() {
  // put your setup code here, to run once:
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
  pinMode(encoderPin_a, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin_a), countPulse_a, RISING); // Interrupción en flanco ascendente
  // Encoder Setup
  pinMode(encoderPin_b, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin_b), countPulse_b, RISING); // Interrupción en flanco ascendente

}

void loop() {
    Serial.print("Pulsos a: ");
    Serial.println(pulseCount_a);

    Serial.print("Pulsos b: ");
    Serial.println(pulseCount_b);

  digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 255);
  if(pulseCount_a<=0.6*417)
  {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 255);
  }
  else
  {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
  }

}

void countPulse_a() {
    pulseCount_a++; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}

void countPulse_b() {
    pulseCount_b++; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}
