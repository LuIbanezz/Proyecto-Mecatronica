#include <SPI.h>
#include <RF24.h>

// Definición de pines para el NRF24L01
#define CE_PIN 9
#define CSN_PIN 10
#define IRQ_PIN 2

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
}

void handleRadioInterrupt() {

  Serial.println("Dentro de la interrupción");

  int receivedCommand;
  radio.read(&receivedCommand, sizeof(receivedCommand));
  lastReceived = millis(); // Actualiza el tiempo de recepción
    
  Serial.print("Comando recibido: ");
  Serial.println(receivedCommand);


}
