#include <SPI.h>
#include <RF24.h>

//#define DEBUG
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
};

int lastSent = DISABLE;

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

  #ifdef DEBUG
  Serial.println("Pre radio configuration is ready.");
  #endif

  // Initialize radio
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();

  #ifdef DEBUG
  Serial.println("Ready to detect button presses and send via radio.");
  #endif
}

void loop() {

  Serial.println("Loopping");
  if (digitalRead(UP_BUTTON_PIN) == LOW) {
    #ifdef DEBUG
    Serial.println("Up is pressed");
    #endif
    sendState(UP);
    while (digitalRead(UP_BUTTON_PIN) == LOW) {}  // Wait until button is released
    #ifdef DEBUG
    Serial.println("Up is released");
    #endif
  }

  if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
    #ifdef DEBUG
    Serial.println("Down is pressed");
    #endif
    sendState(DOWN);
    while (digitalRead(DOWN_BUTTON_PIN) == LOW) {}  // Wait until button is released
    #ifdef DEBUG
    Serial.println("Down is released");
    #endif
  }

  if (digitalRead(LEFT_BUTTON_PIN) == LOW) {
    #ifdef DEBUG
    Serial.println("Left is pressed");
    #endif
    sendState(LEFT);
    while (digitalRead(LEFT_BUTTON_PIN) == LOW) {}  // Wait until button is released
    #ifdef DEBUG
    Serial.println("Left is released");
    #endif
  }

  if (digitalRead(RIGHT_BUTTON_PIN) == LOW) {
    #ifdef DEBUG
    Serial.println("Right is pressed");
    #endif
    sendState(RIGHT);
    while (digitalRead(RIGHT_BUTTON_PIN) == LOW) {}  // Wait until button is released
    #ifdef DEBUG
    Serial.println("Right is released");
    #endif
  }

  if(lastSent != ENABLE){
    sendState(DISABLE);
  }

  delay(10);
}

void disableRelay() {
  sendState(DISABLE);
}

void enableRelay(){
  sendState(ENABLE);
}

void sendState(ButtonState state) {
  if (state != lastSent) {
    // Send only if there’s a state change that’s not from DISABLE to anything else
    bool result = radio.write(&state, sizeof(state));

    #ifdef DEBUG
    Serial.print("State sent: ");
    Serial.println(state);
    #endif

    lastSent = state;

    #ifdef DEBUG
    if (result) {
        Serial.println("Transmission successful.");
    } else {
        Serial.println("Transmission failed.");
    }
    #endif
  }  
}
