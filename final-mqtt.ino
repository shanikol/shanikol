#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino_LSM6DS3.h>

const double ULTRASONIC_THRESHOLD_1 = 500;  
const double ULTRASONIC_THRESHOLD_2 = 300;
const double ULTRASONIC_THRESHOLD_3 = 300;  
const double WATER_THRESHOLD = 100;
const double GYROSCOPE_THRESHOLD = 200;
const char DATA_CHANNEL[] = "data";

// PINS
const int buttonPins[3] = {10, 11, 12};
const int trigPins[3] = {2, 4, 6}; //Ultrasonic Pins
const int echoPins[3] = {3, 5, 7}; //Ultrasonic Pins
const int waterSensorPowerPin = 13; //Water Sensor Pin
const int waterSensorDataPin = A0; //Water Sensor Pin
const int relayPin = 8;  
bool relayState = LOW;   
const int buzzerPin = 9;
bool alarmActive = false;

// SETTINGS
double volume = 1;
double intensity = 1;

// MQTT AND WIFI COMMUNICATION
char ssid[] = "meowa";
char pass[] = "asdfghjkl";          
int status = WL_IDLE_STATUS;
char mqttServer[] = "34.1.195.29";
int mqttPort = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

//FCUNTION DECLARATIONS
void connectToWifi();
void connectToMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void sendData(bool isObstacle1, bool isObstacle2, bool isObstacle3, bool isWaterDetected, bool isFall, bool isEmergencyPressed, bool isPowerPressed, bool isStopPressed);
void alarmSignal();
bool alarmState();
bool getObstacleDetected(int sensorIndex, double threshold, int relayPin);
bool getWaterDetected();
bool getFall();

void setup() {
  Serial.begin(9600);
  while (!Serial);

  for (int i = 0; i < 3; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  pinMode(waterSensorPowerPin, OUTPUT);
  digitalWrite(waterSensorPowerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  for (int i = 0; i < 3; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  connectToWifi();
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(mqttCallback);
  connectToMQTT();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWifi();
  }

  if (!mqttClient.connected()) {
    connectToMQTT();
  }

  bool isEmergencyPressed = (digitalRead(buttonPins[0]) == LOW);
  bool isPowerPressed = (digitalRead(buttonPins[1]) == LOW);
  bool isStopPressed = (digitalRead(buttonPins[2]) == LOW);

  if (isEmergencyPressed) {
    alarmSignal();
    sendData(false, false, false, false, false, isEmergencyPressed, false, false);
    mqttClient.loop();
    return;
  }

  if (isPowerPressed) {
    sendData(false, false, false, false, false, false, isPowerPressed, false);
    mqttClient.loop();
    return;
  }

if (isStopPressed) {
  //Stop the vibration
  relayState = LOW;
  digitalWrite(relayPin, relayState);

  // Stop the alarm
  alarmActive = false;
  noTone(buzzerPin);

  sendData(false, false, false, false, false, false, false, isStopPressed);
  mqttClient.loop();
  return;
}


  bool isObstacle1 = getObstacleDetected(0, ULTRASONIC_THRESHOLD_1, relayPin);
  bool isObstacle2 = getObstacleDetected(1, ULTRASONIC_THRESHOLD_2, relayPin);
  bool isObstacle3 = getObstacleDetected(2, ULTRASONIC_THRESHOLD_3, relayPin);
  bool isWaterDetected = getWaterDetected();
  bool isFall = getFall();

  sendData(isObstacle1, isObstacle2, isObstacle3, isWaterDetected, isFall, isEmergencyPressed, isPowerPressed, isStopPressed);
  mqttClient.loop();
}

//BUZZER ALARM
void alarmSignal() {
  int frequency1 = 1000;
  int frequency2 = 1500;
  int duration = 1000;

  alarmActive = true;

  for (int i = 0; i < 10 && alarmActive; i++) {  // Stop if alarmActive becomes false
    tone(buzzerPin, frequency1);
    analogWrite(buzzerPin, volume * 255);
    delay(duration);
    noTone(buzzerPin);
    delay(50);

    tone(buzzerPin, frequency2);
    delay(duration);
    noTone(buzzerPin);
    delay(50);
  }
}

void connectToWifi() {
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
   
    if (status != WL_CONNECTED) {
      Serial.print("Attempting to reconnect");
      delay(3000);
    }
  }
  Serial.println("You're connected to the network");
  Serial.println("---------------------------------------");
}

void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("thisIsUniqueIDSDSDSD")) {
      Serial.println("connected");
      mqttClient.publish("connections", "Arduino connected");
      mqttClient.subscribe("blindstick");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Data format {"volume intensity"}
  sscanf(input.c_str(), "%lf %lf", &volume, &intensity);
  Serial.print("Volume: ");
  Serial.println(volume);
  Serial.print("Intensity: ");
  Serial.println(intensity);
}

void sendData(bool isObstacle1, bool isObstacle2, bool isObstacle3, bool isWaterDetected, bool isFall, bool isEmergencyPressed, bool isPowerPressed, bool isStopPressed) {
  String data = String((isObstacle1 ? "1" : "0")) + " " +
                String((isObstacle2 ? "1" : "0")) + " " +
                String((isObstacle3 ? "1" : "0")) + " " +
                String((isWaterDetected ? "1" : "0")) + " " +
                String((isFall ? "1" : "0")) + " " +
                String((isEmergencyPressed ? "1" : "0")) + " " +
                String((isPowerPressed ? "1" : "0")) + " " +
                String((isStopPressed ? "1" : "0"));

  mqttClient.publish(DATA_CHANNEL, data.c_str());
}

bool getObstacleDetected(int sensorIndex, double threshold, int relayPin) {
  digitalWrite(trigPins[sensorIndex], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[sensorIndex], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[sensorIndex], LOW);

  long duration_us = pulseIn(echoPins[sensorIndex], HIGH);
  
  if (duration_us == 0) {
    return false;
  } else {
    float distance_cm = 0.017 * duration_us;

    if (distance_cm >= 2 && distance_cm <= threshold) {
      if (distance_cm >= 20) {
        digitalWrite(relayPin, HIGH);
      } else {
        digitalWrite(relayPin, LOW);
      }
      return true;
    }
  }
  return false;
}

bool getWaterDetected() {
  digitalWrite(waterSensorPowerPin, HIGH);
  delay(10);  
  int value = analogRead(waterSensorDataPin);  
  digitalWrite(waterSensorPowerPin, LOW);  
  return (value >= WATER_THRESHOLD);
}

bool getFall() {
  float accelX = 0, accelY = 0, accelZ = 0;
  float gyroX = 0, gyroY = 0, gyroZ = 0;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(accelX, accelY, accelZ);
    Serial.print("Acceleration X: ");
    Serial.print(accelX);
    Serial.print(" | Y: ");
    Serial.print(accelY);
    Serial.print(" | Z: ");
    Serial.println(accelZ);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gyroX, gyroY, gyroZ);
    Serial.print("Gyroscope X: ");
    Serial.print(gyroX);
    Serial.print(" | Y: ");
    Serial.print(gyroY);
    Serial.print(" | Z: ");
    Serial.println(gyroZ);
  }

  if (abs(gyroX) > GYROSCOPE_THRESHOLD || abs(gyroY) > GYROSCOPE_THRESHOLD || abs(gyroZ) > GYROSCOPE_THRESHOLD) {
    Serial.println("Fall detected!");
    alarmSignal();
    return true;
  } else {
    return false;
  }
}

