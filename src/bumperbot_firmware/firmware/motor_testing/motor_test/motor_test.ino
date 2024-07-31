/*
7/29/24 - debug arduino to vesc
*/
#include <Servo.h>

Servo m1_servo;
Servo m2_servo;
String inString = "";  // string to hold input from keyboard
int m1_ppm_pin = 5;
int m2_ppm_pin = 6;

// int speed = 0;
double speed = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }
  m1_servo.attach(m1_ppm_pin);
  m2_servo.attach(m2_ppm_pin);
  //  pinMode(m1_ppm_pin, OUTPUT);
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
  //*/
  //m1_servo.writeMicroseconds(1750);
  //m2_servo.writeMicroseconds(1750);
}