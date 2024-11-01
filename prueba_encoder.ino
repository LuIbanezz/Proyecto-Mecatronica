/*** Timer 1 setup for Sample ***/
#include <TimerOne.h>
#include <MPU9250.h>
#include <Wire.h> 

#define DELAY 2000

MPU9250 mpu;



/*** Motor Setup ***/
const int enA = 6;
const int in1 = 4;
const int in2 = 2;

/*** Encoder Setup***/
//static int pulseCount = 0; // Contador de pulsos
unsigned long previousMillis = 0; // Tiempo de la última medición de velocidad
const int encoderPin = 13; // Pin para el Canal A del encoder
const int interval = 1000; // Intervalo de tiempo para calcular la velocidad (ms)



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

  // Encoder Setup
  pinMode(encoderPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING); // Interrupción en flanco ascendente

}

void loop() {
    // Mostrar el número total de pulsos en el monitor serie
    Serial.println("Pulsos: ");
    Serial.println(countPulse());
    //delay(500); // Actualización cada medio segundo

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 120);

}

int countPulse() {
  static int pulseCount = 0; 
   
    return pulseCount++; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}
