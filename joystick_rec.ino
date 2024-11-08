#include <SPI.h>
#include <RF24.h>

// Definición de pines para el NRF24L01
#define CE_PIN 34
#define CSN_PIN 36
#define IRQ_PIN 21


/*** Motor Setup ***/
const int enA = 8;
const int in1 = 28;
const int in2 = 30;  //Motor A aprox 3060 pulsos en 2 seg

const int enB = 9;
const int in3 = 29;
const int in4 = 31; //Motor B aprox 2370 pulsos en 2 seg (varia bastante :C)

/*** Encoder Setup ***/
const int encoderA= 2;
const int encoderB= 3;

/*** Encoder Setup***/
unsigned long previousMillis = 0; // Tiempo de la última medición de velocidad
const int encoderPin = 13; // Pin para el Canal A del encoder
const int interval = 1000; // Intervalo de tiempo para calcular la velocidad (ms)

volatile int pulseCountA = 0; // Contador de pulsos del encoder A
volatile int pulseCountB = 0; // Contador de pulsos del encoder B

volatile int receivedCommand;

RF24 radio(CE_PIN, CSN_PIN); // Inicialización del módulo RF
const byte address[6] = "00001"; // Dirección del canal de comunicación
unsigned long lastReceived = millis();



enum ButtonState {
  DISABLE = 0,
  ENABLE = 1,
  UP = 2,
  DOWN = 3,
  LEFT = 4,
  RIGHT = 5,
};

void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Configuración del módulo RF
  if (!radio.begin()) {
    Serial.println("Error al inicializar el radio.");
    while (1); // Detener si falla la inicialización
  }
  
  radio.openReadingPipe(0, address); // Abre el canal de lectura
  radio.setPALevel(RF24_PA_LOW);     // Nivel de potencia bajo para pruebas cercanas
  radio.startListening();            // Configura el módulo en modo escucha

  pinMode(IRQ_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), handleRadioInterrupt, FALLING); // Interrupción en IRQ_PIN

  Serial.println("Receptor listo para recibir señales.");

  // Encoder Setup
  interrupts();
  attachInterrupt(digitalPinToInterrupt(encoderA), countPulseA, RISING); // Interrupción en flanco ascendente
  attachInterrupt(digitalPinToInterrupt(encoderB), countPulseB, RISING); // Interrupción en flanco ascendente
}


void loop() {

  // Reinicia el radio si no ha recibido nada por más de 5 segundos
  if (millis() - lastReceived > 5000) {
    Serial.println("Reiniciando radio por inactividad.");
    radio.stopListening();
    radio.startListening();
    lastReceived = millis();
  }

  delay(10);

    Serial.println("Pulsos A: ");
    Serial.println(pulseCountA);
    Serial.println("Pulsos B: ");
    Serial.println(pulseCountB);

  if (receivedCommand == DISABLE)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);

    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enB, 0);
  }

  else if (receivedCommand == DOWN)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 255);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 255);
  }

  else if (receivedCommand == UP)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 255);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 255);
  }

  else if (receivedCommand == LEFT)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, 255);

    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, 255);
  }

  else if (receivedCommand == RIGHT)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, 255);

    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, 255);
  }

}

void handleRadioInterrupt() {

  Serial.println("Dentro de la interrupción");

  radio.read(&receivedCommand, sizeof(receivedCommand));
  lastReceived = millis(); // Actualiza el tiempo de recepción
    
  Serial.print("Comando recibido: ");
  Serial.println(receivedCommand);


}

void countPulseA() {
    pulseCountA = pulseCountA+1; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}

void countPulseB() {
    pulseCountB = pulseCountB+1; // Incrementa el contador de pulsos cada vez que detecta un flanco ascendente
}
