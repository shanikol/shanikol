#include <WiFiNINA.h>
#include <PubSubClient.h>

//please enter your sensitive data in the Secret tab
char ssid[] = "Infinix ZERO ULTRA"; 
char pass[] = "qwertyuiop";           
int status = WL_IDLE_STATUS; 
char mqttServer[] = "mqtt-dashboard.com";
int mqttPort = 1883;

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
  Serial.begin(9600);
  while (!Serial);

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

  mqttClient.loop();
}
