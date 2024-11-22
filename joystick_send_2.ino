#include <SPI.h>
#include <RF24.h>

#define DEBUG

#define VRxPin A3 // Pin analógico para VRx
#define VRyPin A2 // Pin analógico para VRy
#define DISABLE_BUTTON_PIN 2 // Pin digital para deshabilitar
#define ENABLE_BUTTON_PIN 3

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001"; // Dirección para la comunicación por radio

#define deadZone 250

enum STATE_MACHINE {
  MANUAL,
  AUTO,
  FORWARD_M,
  BACKWARD_M,
  LEFT_T,
  RIGHT_T
};

STATE_MACHINE lastSent = MANUAL;

void setup() {
  Serial.begin(9600);

  pinMode(DISABLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENABLE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DISABLE_BUTTON_PIN), disableRobot, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENABLE_BUTTON_PIN), enableRobot, FALLING);

#ifdef DEBUG
  Serial.println("Pre radio configuration is ready.");
#endif

  // Inicializar radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

#ifdef DEBUG
  Serial.println("Ready to detect button presses and send via radio.");
#endif
}

void loop() {
  int xValue = analogRead(VRxPin); // Lectura del eje X
  int yValue = analogRead(VRyPin); // Lectura del eje Y

  // Centrar los valores en torno a 0
  int xOffset = xValue - 512;
  int yOffset = yValue - 512;

  // Verificar si estamos fuera de la zona muerta
  if (abs(xOffset) > deadZone || abs(yOffset) > deadZone) {
    if (lastSent == MANUAL) { // Solo permitir cambiar dirección desde MANUAL
      // Determinar la dirección dominante
      if (abs(xOffset) > abs(yOffset)) {
        if (xOffset > 0) {
          sendState(RIGHT_T);
        } else {
          sendState(LEFT_T);
        }
      } else {
        if (yOffset > 0) {
          sendState(BACKWARD_M);
        } else {
          sendState(FORWARD_M);
        }
      }
    }
  } else {
    if ((lastSent != MANUAL) && (lastSent != AUTO)) {
      sendState(MANUAL); // Siempre volver a MANUAL cuando no hay movimiento
    }
  }

  delay(10); // Pequeño retraso para evitar mensajes excesivos
}

void disableRobot() {
  sendState(MANUAL);
}

void enableRobot() {
  sendState(AUTO);
}

// Función para convertir el estado a una cadena de texto
const char* stateToString(STATE_MACHINE state) {
  switch (state) {
    case MANUAL:     return "MANUAL";
    case AUTO:       return "AUTO";
    case FORWARD_M:  return "FORWARD_M";
    case BACKWARD_M: return "BACKWARD_M";
    case LEFT_T:     return "LEFT_T";
    case RIGHT_T:    return "RIGHT_T";
    default:         return "UNKNOWN";
  }
}

void sendState(STATE_MACHINE state) {
  // Reglas para permitir el cambio de estado
  if ((state != MANUAL) && (lastSent != MANUAL)) {
#ifdef DEBUG
    Serial.println("Solo se puede cambiar de estado desde MANUAL.");
#endif
    return; // Bloquear cualquier cambio de dirección sin pasar por MANUAL
  }

  if (state != lastSent) {
    // Enviar solo si hay un cambio de estado
    bool result = radio.write(&state, sizeof(state));

#ifdef DEBUG
    Serial.print("Estado enviado: ");
    Serial.println(stateToString(state)); // Imprimir el nombre del estado
#endif

    lastSent = state;

#ifdef DEBUG
    if (result) {
      Serial.println("Transmisión exitosa.");
    } else {
      Serial.println("Transmisión fallida.");
    }
#endif
  }
}
