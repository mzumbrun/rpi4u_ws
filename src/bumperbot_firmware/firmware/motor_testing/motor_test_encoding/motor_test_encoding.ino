/*
7/29/24 - debug arduino to vesc
*/
#include <Servo.h>

Servo m1_servo;
Servo m2_servo;
String inString = "";  // string to hold input from keyboard

#define m1_ppm_pin 5
#define m2_ppm_pin 6
#define m2_encoder_phaseA 3  // Interrupt
//#define m2_encoder_phaseB 9
#define m1_encoder_phaseA 2  // Interrupt
//#define m1_encoder_phaseB 4

double speed = 0.0;
double m1_wheel_meas_vel = 0;
double m2_wheel_meas_vel = 0;

// Encoders
unsigned int m1_encoder_counter = 0;
unsigned int m2_encoder_counter = 0;
String m1_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String m2_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
unsigned long current_millis = 0;
const unsigned long interval = 1000;  // the desired time between encoder measurements, msec

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }
  m1_servo.attach(m1_ppm_pin);
  m2_servo.attach(m2_ppm_pin);

  // Init encoders
  //pinMode(m2_encoder_phaseB, INPUT);
  //pinMode(m1_encoder_phaseB, INPUT);
  // pinMode(m2_encoder_phaseA, INPUT_PULLUP);
  //pinMode(m1_encoder_phaseA, INPUT_PULLUP);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(m2_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(m1_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read serial input:
  ///*
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Serial.print("ppm speed: ");
      speed = inString.toFloat();
      //speed = inString.toInt();
      Serial.println(speed);
      inString = "";
      // speed = map(speed, 0, 1, 0, 2000);
      Serial.println(speed);
      m1_servo.writeMicroseconds(speed);
      m2_servo.writeMicroseconds(speed);
    }
  }
  // Encoder
  current_millis = millis();
  // real_interval = current_millis - last_millis;
  if (current_millis - last_millis >= interval) {

    // 60 = min to sec
    // 14 = encoder pulses per revolution
    // 5 = gearbox ratio
    // .10472 = converts rpm to rads/sec

    m2_wheel_meas_vel = m2_encoder_counter * (60.0 / 70.) * 0.10472;  // rad/s
    m1_wheel_meas_vel = m1_encoder_counter * (60.0 / 70.) * 0.10472;

    String counters = "M1=" + m1_wheel_sign + String(m1_encoder_counter) + " " + String(m1_wheel_meas_vel) + " M2 = " + m2_wheel_sign + String(m2_encoder_counter) + " " + String(m2_wheel_meas_vel);
    Serial.println(counters);


    last_millis = current_millis;
    m2_encoder_counter = 0;
    m1_encoder_counter = 0;
  }
}
// New pulse from Right Wheel Encoder
void rightEncoderCallback() {
  // if (digitalRead(m2_encoder_phaseB) == HIGH) {
  //   m2_wheel_sign = "p";
  // } else {
  //  m2_wheel_sign = "n";
  // }
  m2_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback() {
  // if (digitalRead(m1_encoder_phaseB) == HIGH) {
  //   m1_wheel_sign = "n";
  // } else {
  //   m1_wheel_sign = "p";
  // }
  m1_encoder_counter++;
}