#include <SPI.h>
#include <RF24.h>

// Pin definitions for buttons and LED
#define DISABLE_BUTTON_PIN 2
#define ENABLE_BUTTON_PIN 3

#define UP_BUTTON_PIN 4
#define DOWN_BUTTON_PIN 5
#define LEFT_BUTTON_PIN 6
#define RIGHT_BUTTON_PIN 7

#define RELAY_PIN 8 // Built-in LED pin on Arduino Nano

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001"; // Address for the radio communication

// Define states as numbers
enum ButtonState {
  DISABLE = 0,
  ENABLE = 1,
  UP = 2,
  DOWN = 3,
  LEFT = 4,
  RIGHT = 5,
  IDLE = 6
};

int lastSent = IDLE;

void setup() {
  Serial.begin(9600);

  // Initialize the LED pin as output
  pinMode(RELAY_PIN, OUTPUT);

  // Initialize button pins with internal pull-up resistors
  pinMode(DISABLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENABLE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);

  // Attach interrupt to buttonPin, triggering on FALLING edge (button press)
  attachInterrupt(digitalPinToInterrupt(DISABLE_BUTTON_PIN), disableRelay, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENABLE_BUTTON_PIN), enableRelay, FALLING);

  Serial.println("Pre radio configuration is ready.");
  // Initialize radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  Serial.println("Ready to detect button presses and send via radio.");
}

void loop() {

  if (digitalRead(UP_BUTTON_PIN) == LOW) {
    Serial.println("Up is pressed");
    sendState(UP);
    while (digitalRead(UP_BUTTON_PIN) == LOW) {}  // Wait until button is released
    Serial.println("Up is released");
  }

  if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
    Serial.println("Down is pressed");
    sendState(DOWN);
    while (digitalRead(DOWN_BUTTON_PIN) == LOW) {}  // Wait until button is released
    Serial.println("Down is released");
  }

  if (digitalRead(LEFT_BUTTON_PIN) == LOW) {
    Serial.println("Left is pressed");
    sendState(LEFT);
    while (digitalRead(LEFT_BUTTON_PIN) == LOW) {}  // Wait until button is released
    Serial.println("Left is released");
  }

  if (digitalRead(RIGHT_BUTTON_PIN) == LOW) {
    Serial.println("Right is pressed");
    sendState(RIGHT);
    while (digitalRead(RIGHT_BUTTON_PIN) == LOW) {}  // Wait until button is released
    Serial.println("Right is released");
  }

  sendState(IDLE);
  Serial.println("Idle...");

  delay(10);
}

void disableRelay() {
  // Toggle the LED state
  sendState(DISABLE);
}

void enableRelay(){
  sendState(ENABLE);
}

void sendState(ButtonState state) {
//   if (lastSent == DISABLE) {
//     // Only send if transitioning from DISABLE to ENABLE
//     if (state == ENABLE) {
//         radio.write(&state, sizeof(state));
//         Serial.print("State sent: ");
//         Serial.println(state);  // For debugging
//         lastSent = state;
//     }
// } else if (state != lastSent) {
//     // Send only if there’s a state change that’s not from DISABLE to anything else
//     radio.write(&state, sizeof(state));
//     Serial.print("State sent: ");
//     Serial.println(state);  // For debugging
//     lastSent = state;
// }

  if (state != lastSent) {
    // Send only if there’s a state change that’s not from DISABLE to anything else
    bool result = radio.write(&state, sizeof(state));
    Serial.print("State sent: ");
    Serial.println(state);  // For debugging
    lastSent = state;

    if (result) {
        Serial.println("Transmission successful.");
    } else {
        Serial.println("Transmission failed.");
    }
  }  
}
