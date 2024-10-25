/*** Timer 1 setup for Sample ***/
#include <TimerOne.h>

const unsigned long interval = 10000; // intervalo en us

/*** Motor Setup ***/
const int enA = 6;
const int in1 = 4;
const int in2 = 2;

const int enB = 5;
const int in3 = 7;
const int in4 = 8;

/*** Sensor Setup ***/
const int sens1= 14;
const int sens2= 15;
const int sens3= 16;
const int sens4= 17;
const int sens5= 3; 
const int sens6= 12;
const int sens7= 10; 
const int sens8= 9; 


#define DELAY 2000

void setup() {
  Serial.begin(115200);
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

  //Timer1.initialize(interval);   //  conversión a us
  //Timer1.attachInterrupt(timerCallback);

  delay(1000);

}

void loop() {

 // int v = analogRead(sens5);

  //Serial.println(v);

  if(!digitalRead(sens8)) //si detecta mesa
  {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 255);
      Serial.println("detecta mesa");

      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
      analogWrite(enB, 255);
  }

  else
  {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
      Serial.println("no detecta mesa");

      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enB, 0);
  }
  

}
