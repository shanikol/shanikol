const int BUTTON_PIN = 2;  
const int RELAY_PIN  = 5;  

bool relayState = LOW;     
bool lastButtonState = HIGH; 
bool toggled = false;      

unsigned long lastDebounceTime = 0;  
unsigned long debounceDelay = 50;    

void setup() {
  Serial.begin(9600);                  
  pinMode(BUTTON_PIN, INPUT_PULLUP);   
  pinMode(RELAY_PIN, OUTPUT);          
  digitalWrite(RELAY_PIN, relayState); 
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);  

  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();  
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW && !toggled) {
      relayState = !relayState;    
      digitalWrite(RELAY_PIN, relayState); 
      Serial.print("Relay is now ");
      Serial.println(relayState ? "ON" : "OFF");
      toggled = true;              
    }

    if (buttonState == HIGH) {
      toggled = false;
    }
  }

  lastButtonState = buttonState;  
}
