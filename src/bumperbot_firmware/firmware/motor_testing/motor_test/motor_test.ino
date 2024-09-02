/*
9/1/2024 - use this with vesc program to calibrate ppm to rad/s
         - connect arduino to ubuntu pc, connect vesc to wsl pc with vesc program
         - enter ppm and use vesc program with real time data on for erpm, voltage, etc. Record data.
7/29/24 - debug arduino to vesc
*/
#include <Servo.h>

Servo m1_servo;
String inString = "";  // string to hold input from keyboard
#define m1_ppm_pin 9
double speed = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
  }
  m1_servo.attach(m1_ppm_pin);
  pinMode(m1_ppm_pin, OUTPUT);
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
    // if a newline, print the string, then the string's value:
    if (inChar == '\n') {
      Serial.print("ppm speed: ");
      speed = inString.toFloat();
      //speed = inString.toInt();
      Serial.println(speed);
      inString = "";
      Serial.println(speed);
      m1_servo.writeMicroseconds(speed);
        }
    
  }
}