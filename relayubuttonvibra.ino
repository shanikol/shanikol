const int BUTTON_PIN = 2;  // Arduino pin connected to button's pin 
const int RELAY_PIN  = 5;  // Arduino pin connected to relay's pin 

bool relayState = LOW;     // Variable to store the state of the relay
bool lastButtonState = HIGH; // Variable to store the previous button state
bool toggled = false;      // Variable to ensure toggle only on button press

unsigned long lastDebounceTime = 0;  // Variable for the last time the button was toggled
unsigned long debounceDelay = 50;    // Debounce time; increase if necessary

void setup() {
  Serial.begin(9600);                  // Initialize serial communication at 9600 baud rate
  pinMode(BUTTON_PIN, INPUT_PULLUP);   // Set button pin to input with internal pull-up resistor
  pinMode(RELAY_PIN, OUTPUT);          // Set relay pin to output mode
  digitalWrite(RELAY_PIN, relayState); // Initialize relay state
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);  // Read the state of the button

  // If button state has changed from HIGH to LOW (button press detected), debounce it
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();  // Reset the debounce timer
  }

  // If enough time has passed since the last button state change, toggle the relay
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button is pressed (LOW) and hasn't been toggled yet
    if (buttonState == LOW && !toggled) {
      relayState = !relayState;    // Toggle relay state
      digitalWrite(RELAY_PIN, relayState); // Update relay state
      Serial.print("Relay is now ");
      Serial.println(relayState ? "ON" : "OFF");
      toggled = true;              // Set toggled flag
    }

    // If the button is released, reset the toggled flag
    if (buttonState == HIGH) {
      toggled = false;
    }
  }

  lastButtonState = buttonState;  // Update the last button state
}
