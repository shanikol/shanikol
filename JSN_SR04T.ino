#define echoPin 11 // attach pin D2 Arduino to pin Echo of JSN-SR04T
#define trigPin 12 // attach pin D3 Arduino to pin Trig of JSN-SR04T

// Defines variables
long duration;  // Variable for the duration of sound wave travel
int distance;   // Variable for the distance measurement

void setup() {
  pinMode(trigPin, OUTPUT); // Set the trigPin as OUTPUT
  pinMode(echoPin, INPUT);  // Set the echoPin as INPUT
  Serial.begin(115200);     // Start serial communication at 115200 baud rate
  Serial.println("Ultrasonic Sensor JSN-SR04T Test"); // Print sensor type
  Serial.println("with Arduino UNO R3");
}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);      // Delay for a stable LOW pulse

  // Set the trigPin HIGH (ACTIVE) for 20 microseconds to extend trigger duration
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);     // Extended from 10 to 20 microseconds
  digitalWrite(trigPin, LOW);

  // Read the echoPin, return the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters (Smaller factor to detect farther distances)
  distance = duration * 0.034 / 2;  // Reduced the speed of sound factor

  // Error handling: Ignore invalid readings
  if (distance >= 2 && distance <= 800) {  // Extended the range from 2 cm to 500 cm
    // Display the distance on the Serial Monitor
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  } else {
    Serial.println("Out of range");
  }

  delay(1000); // Wait a short period before the next measurement
}
