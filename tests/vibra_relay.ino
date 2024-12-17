// Number of ultrasonic sensors
const int numSensors = 3;

// Arrays to store the trigger and echo pins for each sensor
const int trigPins[numSensors] = {2, 4, 6};   // Trigger pins for sensors 1, 2, 3
const int echoPins[numSensors] = {3, 5, 7};   // Echo pins for sensors 1, 2, 3

float duration_us[numSensors];    // Array to store pulse durations
float distance_cm[numSensors];    // Array to store calculated distances

// Relay pin
const int RELAY_PIN = 8;
bool relayState = LOW;

void setup() {
  Serial.begin(9600);                 // Initialize serial communication
  
  // Set the trigger pins as outputs and echo pins as inputs for all sensors
  for (int i = 0; i < numSensors; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  
  // Set relay pin as output
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, relayState); // Initial relay state
}

void loop() {
  // Loop through each sensor and calculate the distance
  for (int i = 0; i < numSensors; i++) {
    // Send a pulse to the current ultrasonic sensor
    digitalWrite(trigPins[i], LOW);       
    delayMicroseconds(2);
    digitalWrite(trigPins[i], HIGH);     
    delayMicroseconds(10);
    digitalWrite(trigPins[i], LOW);

    // Measure the time taken for the echo to return
    duration_us[i] = pulseIn(echoPins[i], HIGH);

    // Check if a valid pulse was received
    if (duration_us[i] == 0) {
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.println(": No echo detected or object out of range");
    } else {
      // Calculate the distance in cm
      distance_cm[i] = 0.017 * duration_us[i];

      // Print the distance on the serial monitor
      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" Distance: ");
      Serial.print(distance_cm[i]);
      Serial.println(" cm");
    }
  }

  // Example logic to toggle relay based on a sensor threshold
  // If any sensor detects a distance less than 10 cm, toggle the relay
  bool obstacleDetected = false;
  for (int i = 0; i < numSensors; i++) {
    if (distance_cm[i] < 10) { // Adjust threshold as needed
      obstacleDetected = true;
      break;
    }
  }

  // Toggle relay based on obstacle detection
  if (obstacleDetected) {
    relayState = LOW;  // Turn on the relay if an obstacle is detected
  } else {
    relayState = HIGH;   // Turn off the relay if no obstacles are detected
  }

  digitalWrite(RELAY_PIN, relayState); // Update relay state

  delay(500);  // Delay before taking the next set of readings
}
