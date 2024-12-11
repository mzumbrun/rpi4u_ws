/*
  12/9/2024 - removed Serial.print on some lines so myController works
  12/3/2024 - removed BLE, added wifi and mqtt
  12/1/2024 - adding BLE to read encoder value
*/

#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>
#include "arduino_secrets.h"
#include <PID_v1.h>

// Drive Train
#define motor_pwm_pin 9    // PWM Motor 1B
#define motor_forward 12   // Dir Motor B
#define motor_backward 7   // Dir Motor B
#define motor_select 6     // connect to pin 5 for right motor (NANO) ... connect to GND for left motor (NANO 33 IOT)
#define dc_high 5          // driven high
#define encoder_counter 2  // Interrupt

// WiFi network details
char ssid[] = SECRET_SSID;
char password[] = SECRET_PASS;

// MQTT server details
const char mqtt_server[] = "test.mosquitto.org";
const int mqtt_port = 1883;
String PIDmessage;
String messageTopic;

WiFiClient nanoClient;
MqttClient mqttClient(nanoClient);

// Encoders
volatile bool state = false;
unsigned int encoder_count_ = 0;
unsigned int clicks_per_rev = 110;
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
bool is_wheel_cmd = false;
bool is_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
String encoder_read = "rp00.00,";
char wheel_side[] = "right";
bool is_right = true;

// speed control
int max_speed = 0;            // 0-255 range with value from CALIBRATION to max rad/s from ROS
int max_rads_per_sec = 22;    // corresponds to that in ROS
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double Kp = 0.;               // orig 12.8
double Ki = 0.;               // orig 8.3
double Kd = 0.;               // orig 0.1
PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);


void setup() {

  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_forward, OUTPUT);
  pinMode(motor_backward, OUTPUT);
  pinMode(motor_select, INPUT);
  pinMode(encoder_counter, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, RISING);
  digitalWrite(motor_forward, HIGH);
  digitalWrite(motor_backward, LOW);
  digitalWrite(dc_high, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);  // initialize the built-in LED pin to indicate when a central is connected

  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
   // Serial.println("Connecting to WiFi.."); // comment when using ROS
  }
 // Serial.println("Connected to WiFi"); // comment when using ROS

 //mqttClient.connect(mqtt_server, mqtt_port);

  if (!mqttClient.connect(mqtt_server, mqtt_port)) {
    Serial.print("MQTT connection failed! Error code = "); // comment when using ROS
    Serial.println(mqttClient.connectError()); // comment when using ROS
    while (1)
      ;
  }
  // Serial.println("Connected to MQTT"); // comment when using ROS
  mqttClient.onMessage(mqttCallback);
  mqttClient.subscribe("PID/encoder");
  mqttClient.subscribe("PID/clicks");
  mqttClient.subscribe("PID/cmd_vel");
  mqttClient.subscribe("PID/actual_vel");

  //delay(100);
  // Read and Interpret Wheel Velocity Commands
  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    max_speed = 255;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    mqttClient.subscribe("PID/KpR");
    mqttClient.subscribe("PID/KiR");
    mqttClient.subscribe("PID/KdR");
    Kp = 12.0;
    Ki = 8.0;
    Kd = 0.1;
    Motor.SetTunings(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    max_speed = 255;
    mqttClient.subscribe("PID/KpL");
    mqttClient.subscribe("PID/KiL");
    mqttClient.subscribe("PID/KdL");
    Kp = 12.0;
    Ki = 8.0;
    Kd = 0.1;
    Motor.SetTunings(Kp, Ki, Kd);
  }

  Motor.SetMode(AUTOMATIC);
}

void loop() {
  mqttClient.poll();
 // Serial.print("Kp= ");
 // Serial.println(Kp);
  // Serial.print("Ki= ");
  // Serial.println(Ki);
  // Serial.print("Kd= ");
  // Serial.println(Kd);
  //Serial.println(wheel_side[0]);
  if (Serial.available()) {
    char chr = Serial.read();
    // determine Wheel Motor
    if (chr == wheel_side[0]) {
      is_wheel_cmd = true;
      value_idx = 0;
      is_cmd_complete = false;
    }

    // Positive direction
    else if (chr == 'p') {
      if (is_wheel_cmd && !is_wheel_forward) {
        // change the direction of the rotation
        digitalWrite(motor_forward, HIGH - digitalRead(motor_forward));
        digitalWrite(motor_backward, HIGH - digitalRead(motor_backward));
        is_wheel_forward = true;
        wheel_sign = "p";
      }
    }
    // Negative direction
    else if (chr == 'n') {
      if (is_wheel_cmd && is_wheel_forward) {
        // change the direction of the rotation
        digitalWrite(motor_forward, HIGH - digitalRead(motor_forward));
        digitalWrite(motor_backward, HIGH - digitalRead(motor_backward));
        is_wheel_forward = false;
        wheel_sign = "n";
      }
    }
    // Separator
    else if (chr == ',' || chr == 'X') {
      if (is_wheel_cmd) {
        wheel_cmd_vel = atof(value);
        is_cmd_complete = true;
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '.';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
      is_wheel_cmd = false;
    }
    // Command Value
    else {
      if (value_idx < 5) {
        value[value_idx] = chr;
        value_idx++;
      }
    }
    noInterrupts();
    if (state) {
      encoder_count_++;
      state = false;
    }
    interrupts();
  }

  // Encoder
  unsigned long current_millis = millis();
  if (current_millis - last_millis >= interval) {
    last_millis = current_millis;
    wheel_meas_vel = (10 * encoder_count_ * (60.0 / clicks_per_rev)) * 0.10472;
    writeEncoderCount();
    encoder_count_ = 0;
    Motor.Compute();  // output is wheel_cmd

    // if setpoint is 0, then make sure cmd to wheels is 0
    if (wheel_cmd_vel == 0.0) {
      wheel_cmd = 0.0;
    }

    // following commands not necessary if using PID to zero in on matched speed with ROS
    // wheel_cmd = constrain(wheel_cmd, 0, max_rads_per_sec);
    // wheel_cmd = map((int)wheel_cmd, 0, max_rads_per_sec, 0, max_speed);
    // above not necessary if using PID

    analogWrite(motor_pwm_pin, wheel_cmd);
    if (is_right) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel) + ",";
    } else {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel) + ",";
    }

    Serial.println(encoder_read);
  }
}

// New pulse from Left Wheel Encoder
void EncoderCallback() {
  state = true;
}

void mqttCallback(int length) {
  char message[20];
  double k;
  messageTopic = mqttClient.messageTopic();
  if (messageTopic == "PID/KpR") {
    Kp = convertMessage();
  } else if (messageTopic == "PID/KiR") {
    Ki = convertMessage();
  } else if (messageTopic == "PID/KdR") {
    Kd = convertMessage();
  } else if (messageTopic == "PID/KpL") {
    Kp = convertMessage();
  } else if (messageTopic == "PID/KiL") {
    Ki = convertMessage();
  } else if (messageTopic == "PID/KdL") {
    Kd = convertMessage();
  } else if (messageTopic == "PID/clicks") {
    clicks_per_rev = convertMessage();
  }
  Motor.SetTunings(Kp, Ki, Kd);
 // delay(100);
}

double convertMessage() {
  // Serial.println(messageTopic);
  while (mqttClient.available()) {
    char inChar = ((char)mqttClient.read());
    PIDmessage += inChar;
  }
  double k = PIDmessage.toDouble();
  PIDmessage = "";
  return k;
}

void writeEncoderCount() {
  char message[20];

  dtostrf(encoder_count_, 6, 2, message);
  mqttClient.beginMessage("PID/encoder");
  mqttClient.print(message);
  mqttClient.endMessage();

  dtostrf(clicks_per_rev, 6, 0, message);
  mqttClient.beginMessage("PID/clicks");
  mqttClient.print(message);
  mqttClient.endMessage();

  dtostrf(wheel_cmd_vel, 6, 2, message);  // rad/s command from ROS
  mqttClient.beginMessage("PID/cmd_vel");
  mqttClient.print(message);
  mqttClient.endMessage();

  dtostrf(wheel_meas_vel, 6, 2, message); // rad/s measured from encoder values
  mqttClient.beginMessage("PID/actual_vel");
  mqttClient.print(message);
  mqttClient.endMessage();
}

char* dtostrf(double val, signed char width, unsigned char prec, char* sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}