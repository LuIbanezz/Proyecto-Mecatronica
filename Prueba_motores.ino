/*** Timer 1 setup for Sample ***/
#include <TimerOne.h>
#include <Wire.h> 

const unsigned long interval = 10000; // intervalo en us

/*** Motor Setup ***/
const int enA = 8;
const int in1 = 28;
const int in2 = 30;

const int enB = 9;
const int in3 = 29;
const int in4 = 31;

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
const int encoderA= 3;
const int encoderB= 2;


#define DELAY 2000

void setup() {
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

  // Sensor setup
  pinMode(sens1, INPUT);
  pinMode(sens2, INPUT);
  pinMode(sens3, INPUT);
  pinMode(sens4, INPUT);
  pinMode(sens5, INPUT);
  pinMode(sens6, INPUT);
  pinMode(sens7, INPUT);
  pinMode(sens8, INPUT);


  delay(1000);

}

void loop() {

  if(!digitalRead(sens8)) //si detecta mesa
  {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 255);

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, 255);

      Serial.println("detecta mesa");
  }

  else
  {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);

      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enB, 0);

      Serial.println("no detecta mesa");
  }
}

