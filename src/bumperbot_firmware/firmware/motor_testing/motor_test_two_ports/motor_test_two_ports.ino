#include <SoftwareSerial.h>

#define left_rx_pin 8
#define left_tx_pin 9
#define right_rx_pin 10
#define right_tx_pin 11

String inString = "";  // string to hold input
unsigned long left_count = 1;
unsigned long right_count = 1;

SoftwareSerial left_wheel(left_rx_pin, left_tx_pin);
SoftwareSerial right_wheel(right_rx_pin, right_tx_pin);

void setup() {

  pinMode(left_rx_pin, INPUT);
  pinMode(left_tx_pin, OUTPUT);
  pinMode(right_rx_pin, INPUT);
  pinMode(right_tx_pin, OUTPUT);

  Serial.begin(115200);  //USB port
  while (!Serial) {
    ;
  }
  left_wheel.begin(115200);
  delay(100);
  right_wheel.begin(115200);
}

void loop() {

  left_wheel.listen();
  delay(200);
  if (left_wheel.available()) {
    // for (int i = 0; i <= 3; i++) {
    char inChar = left_wheel.read();
    if (inChar == 'l') {
      char len = left_wheel.read();
      for (int i = 0; i <= len - 1; i++) {
        inChar = left_wheel.read();
        inString += (char)inChar;
      }
  //    Serial.print("left counter: ");
      left_count = inString.toInt();
   //   Serial.println(left_count);
      inString = "";
    }
    // }
  }

  // Serial.println("in loop");

  right_wheel.listen();
  delay(200);
  if (right_wheel.available()) {
    // for (int i = 0; i <= 3; i++) {
    char inChar = right_wheel.read();
    if (inChar == 'r') {
      char len = right_wheel.read();
      for (int i = 0; i <= len - 1; i++) {
        inChar = right_wheel.read();
        inString += (char)inChar;
      }
 //     Serial.print("right counter: ");
      right_count = inString.toInt();
 //     Serial.println(right_count);
      inString = "";
    }
  }
  Serial.print("left counter ");
  Serial.print(left_count);
  Serial.print(",");
  Serial.print("         right counter ");
  Serial.println(right_count);
}
