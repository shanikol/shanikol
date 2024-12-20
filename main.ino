#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino_LSM6DS3.h>

const long WIFI_RECONNECT_COOLDOWN = 5000;
const long MQTT_RECONNECT_COOLDOWN = 5000;
const long EMERGENCY_COOLDOWN = 10000;
const long EMERGENCY_DURATION = 3000;
const long ALARM_COOLDOWN = 3000;
const long ALARM_DURATION = 1500;
const double DISTANCE_MIN_THRESHOLD = 20;  
const double DISTANCE_MAX_THRESHOLD = 100;  
const double DEPTH_MIN_THRESHOLD = 20;
const double DEPTH_MAX_THRESHOLD = 100;
const double WATER_THRESHOLD = 1000;
const double GYROSCOPE_THRESHOLD = 200;
const char DATA_CHANNEL[] = "data";
const char SETTINGS_CHANNEL[] = "settings";

const int trigPin1 = 2;
const int echoPin1 = 3;
const int trigPin2 = 4;
const int echoPin2 = 5;
const int trigPin3 = 7;
const int echoPin3 = 6;
const int emergencyButtonPin = 8;
const int stopButtonPin = 9;
const int buzzerPin = 10;
const int relayPin = 11;
const int waterSensorPowerPin = 12;
const int waterSensorDataPin = A0;

char ssid[] = "Infinix ZERO ULTRA"; 
char pass[] = "qwertyuiop123456789";
// const char mqttServer[] = "34.1.195.29";
const char mqttServer[] = "mqtt-dashboard.com";
const int mqttPort = 1883;

double volume = 0;
double intensity = 1;
unsigned long wifiLastReconnect = 0;
unsigned long mqttLastReconnect = 0;
unsigned long lastEmergencyTriggered = 0;
unsigned long lastAlarmTriggered = 0;
bool emergencyMode = false;
bool alarmMode = false;
String message = "";
bool lastObstacleDetected = false;
bool lastWaterDetected = false;
bool lastFallDetected = false;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(emergencyButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(waterSensorPowerPin, OUTPUT);
  digitalWrite(waterSensorPowerPin, LOW);

  digitalWrite(relayPin, LOW);
  digitalWrite(buzzerPin, HIGH);

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
  connectToWifi();
  connectToMQTT();

  unsigned long currentMillis = millis();
  if (!emergencyMode && digitalRead(emergencyButtonPin) == LOW && currentMillis - lastEmergencyTriggered >= EMERGENCY_COOLDOWN) {
    message = "emergency";
    sendData(message);
    emergencyMode = true;
    lastEmergencyTriggered = currentMillis;
  }

  if (!emergencyMode && !alarmMode && currentMillis - lastAlarmTriggered >= ALARM_COOLDOWN) {
    double distance = getDistance();
    double depth = getDepth();
    int moisture = getMoisture();
    bool fall = getFall();
    String distanceString = "null";
    String depthString = "null";

    bool obstacleDetected = distance >= DISTANCE_MIN_THRESHOLD && distance <= DISTANCE_MAX_THRESHOLD;
    if (obstacleDetected && !lastObstacleDetected) {
      alarmMode = true;
      lastAlarmTriggered = currentMillis;
      distanceString = String(distance);
    }

    bool depthDetected = depth >= DEPTH_MIN_THRESHOLD && distance <= DEPTH_MAX_THRESHOLD;
    if (depthDetected) {
      alarmMode = true;
      lastAlarmTriggered = currentMillis;
      depthString = String(depth);
    }

    bool waterDetected = false;
    if (waterDetected && !lastWaterDetected) {
      alarmMode = true;
      lastAlarmTriggered = currentMillis;
    }

    if (fall && !lastFallDetected) {
      alarmMode = true;
      lastAlarmTriggered = currentMillis;
    }

    bool depthOnly = depthDetected && !obstacleDetected && !waterDetected && !fall;

    if (alarmMode && !depthOnly) {
      message = String((obstacleDetected ? "1" : "0")) + " " +
                String((waterDetected ? "1" : "0")) + " " +
                String((fall ? "1" : "0")) + " " +
                distanceString + " " +
                depthString; 
      sendData(message);
    }
    
    lastObstacleDetected = obstacleDetected;
    lastWaterDetected = waterDetected;
    lastFallDetected = lastFallDetected;
  }

  if (digitalRead(stopButtonPin) == LOW) {
    Serial.println("Stop button pressed");
    digitalWrite(relayPin, LOW);
    digitalWrite(buzzerPin, HIGH);
    emergencyMode = false;
    alarmMode = false;
  }

  currentMillis = millis();

  if (emergencyMode && currentMillis - lastEmergencyTriggered > EMERGENCY_DURATION) {
    digitalWrite(relayPin, LOW);
    digitalWrite(buzzerPin, HIGH);
    emergencyMode = false;
  }

  if (emergencyMode && currentMillis - lastEmergencyTriggered <= EMERGENCY_DURATION) {
    digitalWrite(relayPin, LOW);
    digitalWrite(buzzerPin, LOW);
  }

  if (alarmMode && currentMillis - lastAlarmTriggered > ALARM_DURATION) {
    digitalWrite(relayPin, LOW);
    digitalWrite(buzzerPin, HIGH);
    alarmMode = false;
  }

  if (alarmMode && currentMillis - lastAlarmTriggered <= ALARM_DURATION) {
    digitalWrite(relayPin, HIGH);
    digitalWrite(buzzerPin, LOW);
  }

  mqttClient.loop();
}

void connectToWifi() {
  unsigned long currentMillis = millis();
  if (WiFi.status() != WL_CONNECTED && currentMillis - wifiLastReconnect >= WIFI_RECONNECT_COOLDOWN) {
    Serial.print("Attempting to connect to network: ");
    Serial.println(ssid);
    int status = WiFi.begin(ssid, pass);
    wifiLastReconnect = currentMillis;
    if (status == WL_CONNECTED) {
      Serial.println("Successfully connected to WIFI network");
      return;
    }
    Serial.println("WIFI connection failed. Will try again in " + String(WIFI_RECONNECT_COOLDOWN) + "ms");
  }
}

void connectToMQTT() {
  unsigned long currentMillis = millis();
  if (!mqttClient.connected() && currentMillis - mqttLastReconnect >= MQTT_RECONNECT_COOLDOWN) {
    Serial.print("Attempting MQTT connection...");
    mqttClient.connect("blindstick");
    mqttLastReconnect = currentMillis;
    if (mqttClient.connected()) {
      Serial.println("Successfully connected to MQTT network");
      return;
    }
    Serial.println("MQTT connection failed. Will try again in " + String(MQTT_RECONNECT_COOLDOWN) + "ms");
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
  String message = String((char*)payload).substring(0, length);
  sscanf(message.c_str(), "%lf %lf", &volume, &intensity);
  Serial.print("Volume: ");
  Serial.println(volume);
  Serial.print("Intensity: ");
  Serial.println(intensity);
}

double getDistance(){
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  double duration1 = pulseIn(echoPin1, HIGH);
  double distance1 = 0.017 * duration1;

  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  double duration2 = pulseIn(echoPin2, HIGH);
  double distance2 = 0.017 * duration2;

  Serial.println("Ultrasonic 1: " + String(distance1) + "cm");
  Serial.println("Ultrasonic 2: " + String(distance2) + "cm");
  return min(distance1, distance2);
}

double getDepth(){
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  double duration = pulseIn(echoPin3, HIGH);
  double distance = 0.017 * duration;
  Serial.println("Ultrasonic 3: " + String(distance) + "cm");
  return distance;
}

int getMoisture() {
  digitalWrite(waterSensorPowerPin, HIGH);
  delay(100);
  int value = analogRead(waterSensorDataPin);
  digitalWrite(waterSensorPowerPin, LOW);
  Serial.println("Water sensor: " + String(value));
  return value;
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
    Serial.print("Gyro X: ");
    Serial.print(gyroX);
    Serial.print(" | Y: ");
    Serial.print(gyroY);
    Serial.print(" | Z: ");
    Serial.println(gyroZ);
  }

  if (abs(gyroX) > GYROSCOPE_THRESHOLD || abs(gyroY) > GYROSCOPE_THRESHOLD || abs(gyroZ) > GYROSCOPE_THRESHOLD) {
    Serial.println("Fall detected!");
    return true;
  }

  return false;
}

void sendData(String message) {
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    Serial.println("Sending to MQTT: " + message);
    mqttClient.publish(DATA_CHANNEL, message.c_str());
  }
  else {
    Serial.println("Not connected, skipping sending data...");
  }
}
