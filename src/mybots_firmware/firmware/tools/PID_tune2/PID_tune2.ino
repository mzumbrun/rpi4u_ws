// for tuning PID constants
// 12/2/2024 - initial


#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"

// WiFi network details
char ssid[] = SECRET_SSID;
char password[] = SECRET_PASS;

// MQTT server details
const char mqtt_server[] = "test.mosquitto.org";
const int mqtt_port = 1883;
String PIDmessage;

// PID variables
double Kp = 1.;
double Ki = 2.;
double Kd = 3.;



WiFiClient nanoClient;
MqttClient mqttClient(nanoClient);



void setup() {

  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to WiFi");

  if (!mqttClient.connect(mqtt_server, mqtt_port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    while (1)
      ;
  }
  Serial.println("Connected to MQTT");
  // subscribe to a topics
  mqttClient.subscribe("PID/Kp");
  mqttClient.subscribe("PID/Ki");
  mqttClient.subscribe("PID/Kd");
}



void loop() {

  getPIDconstants();
  Serial.print("Kp= ");
  Serial.println(Kp);
  Serial.print("Ki= ");
  Serial.println(Ki);
  Serial.print("Kd= ");
  Serial.println(Kd);
}

void getPIDconstants() {
  mqttClient.poll();

  String messageTopic = mqttClient.messageTopic();

  if (messageTopic == "PID/Kp") {
    Serial.println(messageTopic);
    while (mqttClient.available()) {
      char inChar = ((char)mqttClient.read());
      PIDmessage += inChar;
    }
    Kp = PIDmessage.toDouble();
    PIDmessage = "";
  } else if (messageTopic == "PID/Ki") {
    Serial.println(messageTopic);
    while (mqttClient.available()) {
      char inChar = ((char)mqttClient.read());
      PIDmessage += inChar;
    }
    Ki = PIDmessage.toDouble();
    PIDmessage = "";
  } else if (messageTopic == "PID/Kd") {
    Serial.println(messageTopic);
    while (mqttClient.available()) {
      char inChar = ((char)mqttClient.read());
      PIDmessage += inChar;
    }
    Kd = PIDmessage.toDouble();
    PIDmessage = "";
    // }
  }


  delay(1000);
}

void writePIDconstants() {
  // Publish data to MQTT topic

  char message[20];

  dtostrf(Kp, 6, 2, message);
  mqttClient.beginMessage("PID/Kp");
  mqttClient.print(message);
  mqttClient.endMessage();

  dtostrf(Ki, 6, 2, message);
  mqttClient.beginMessage("PID/Ki");
  mqttClient.print(message);
  mqttClient.endMessage();

  dtostrf(Kd, 6, 2, message);
  mqttClient.beginMessage("PID/Kd");
  mqttClient.print(message);
  mqttClient.endMessage();
}





void mqttCallback(char* topic, byte* payload, unsigned int length) {

  // Optional callback function if you need to handle incoming messages
}

char* dtostrf(double val, signed char width, unsigned char prec, char* sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}