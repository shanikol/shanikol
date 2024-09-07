#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino_LSM6DS3.h>

const double ULTRASONIC_THRESHOLD_1 = 500;  
const double ULTRASONIC_THRESHOLD_2 = 300; 
const double ULTRASONIC_THRESHOLD_3 = 300;  
const double ULTRASONIC_THRESHOLD_4 = 600;  
const double WATER_THRESHOLD = 100;
const double GYROSCOPE_THRESHOLD = 50;
const char DATA_CHANNEL[] = "data";

char ssid[] = "meowa"; 
char pass[] = "asdfghjkl";           
int status = WL_IDLE_STATUS; 
char mqttServer[] = "mqtt-dashboard.com";
int mqttPort = 1883;

// Ultrasonic Sensor
const int trigPins[4] = {3, 5, 7, 9}; 
const int echoPins[4] = {2, 4, 6, 8};

// Water Sensor
const int waterSensorPowerPin = 10;
const int waterSensorDataPin = A0;

//Buttons
const int buttonPins[3] = {11, 12, 13};

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

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
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < 4; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
  
  pinMode(waterSensorPowerPin, OUTPUT);
  digitalWrite(waterSensorPowerPin, LOW);

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

  bool isObstacle1 = getObstacleDetected(0, ULTRASONIC_THRESHOLD_1);
  bool isObstacle2 = getObstacleDetected(1, ULTRASONIC_THRESHOLD_2);
  bool isObstacle3 = getObstacleDetected(2, ULTRASONIC_THRESHOLD_3);
  bool isObstacle4 = getObstacleDetected(3, ULTRASONIC_THRESHOLD_4);
  bool isWaterDetected = getWaterDetected();
  bool isFall = getFall();
  
  // Button states
  bool isEmergencyPressed = (digitalRead(buttonPins[0]) == LOW);
  bool isPowerPressed = (digitalRead(buttonPins[1]) == LOW);
  bool isStopPressed = (digitalRead(buttonPins[2]) == LOW);
  
  sendData(isObstacle1, isObstacle2, isObstacle3, isObstacle4, isWaterDetected, isFall, isEmergencyPressed, isPowerPressed, isStopPressed);

  mqttClient.loop();
}

void sendData(bool isObstacle1, bool isObstacle2, bool isObstacle3, bool isObstacle4, bool isWaterDetected, bool isFall, bool isEmergencyPressed, bool isPowerPressed, bool isStopPressed) {
  String data = String((isObstacle1 ? "1" : "0")) + " " +
                String((isObstacle2 ? "1" : "0")) + " " +
                String((isObstacle3 ? "1" : "0")) + " " +
                String((isObstacle4 ? "1" : "0")) + " " +
                String((isWaterDetected ? "1" : "0")) + " " +
                String((isFall ? "1" : "0")) + " " +
                String((isEmergencyPressed ? "1" : "0")) + " " +
                String((isPowerPressed ? "1" : "0")) + " " +
                String((isStopPressed ? "1" : "0"));

  mqttClient.publish(DATA_CHANNEL, data.c_str());
}

bool getObstacleDetected(int sensorIndex, double threshold) {
  digitalWrite(trigPins[sensorIndex], LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPins[sensorIndex], HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPins[sensorIndex], LOW);
  long duration = pulseIn(echoPins[sensorIndex], HIGH);
  double distance = duration * 0.034 / 2;
  if (distance >= 2 && distance <= threshold) {
    return true;
  } 
  return false;
}

bool getWaterDetected() {
  digitalWrite(waterSensorPowerPin, HIGH);
  delay(10);  
  int value = analogRead(waterSensorDataPin);  
  digitalWrite(waterSensorPowerPin, LOW);  
  if (value >= WATER_THRESHOLD) {
    return true;
  }
  return false;
}

bool getFall() {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;

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

    if (gyroX < -10 - GYROSCOPE_THRESHOLD || gyroX > 8 + GYROSCOPE_THRESHOLD || 
        gyroY < -10 - GYROSCOPE_THRESHOLD || gyroY > 8 + GYROSCOPE_THRESHOLD || 
        gyroZ < -10 - GYROSCOPE_THRESHOLD || gyroZ > 8 + GYROSCOPE_THRESHOLD) {
      return true;
    }
  }
  
  return false;
}