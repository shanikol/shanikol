#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <Arduino_LSM6DS3.h>

const double ULTRASONIC_THRESHOLD = 500;
const double WATER_THRESHOLD = 700;
const double GYROSCOPE_THRESHOLD = 50;
const String DATA_CHANNEL = "data";

char ssid[] = "Infinix ZERO ULTRA"; 
char pass[] = "qwertyuiop";           
int status = WL_IDLE_STATUS; 
char mqttServer[] = "mqtt-dashboard.com";
int mqttPort = 1883;

const int echoPin = 11;
const int trigPin = 12;
const int waterSensorPowerPin = 7;
const int waterSensorDataPin = A0;

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
      mqttClient.publish("connections","Arduino connected");
      mqttClient.subscribe("blindstick");
    }
    else {
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
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(waterSensorPowerPin, OUTPUT);
  digitalWrite(waterSensorPowerPin, LOW);

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

  bool isObstacleDetected = getObstacleDetected();
  bool isWaterDetected = getWaterDetected();
  bool isFall = getFall();

  // Add condition to trigger actuators
  if (isObstacleDetected || isWaterDetected || isFall) {
    // Trigger buzzer and vibrator
  }

  sendData(isObstacleDetected, isWaterDetected, isFall);

  mqttClient.loop();
}

void sendData(bool isObstacleDetected, bool isWaterDetected, bool isFall){
  String data = String((isObstacleDetected ? "1" : "0")) + " " +
                String((isWaterDetected ? "1" : "0")) + " " +
                String((isFall ? "1" : "0"));

  mqttClient.publish("data", data.c_str());
}

bool getObstacleDetected() {
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  double distance = duration * 0.034 / 2;
  if (distance >= 2 && distance <= ULTRASONIC_THRESHOLD) {
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
