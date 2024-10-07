// 8/11/2024 modified udemy code for measuring encoder values
// physically move the motor by hand and use serial port to read encoder clicks
//
#include <PID_v1.h>

// Wheel Encoders Connection PINs
// motor 
#define encoder_interrupt_1 2   
#define encoder_interrupt_2 3

// Encoders
unsigned int encoder_counter1 = 0;
unsigned int encoder_counter2 = 0;

void setup() {
  // Connection PINs
  pinMode(encoder_interrupt_1, INPUT_PULLUP);
  pinMode(encoder_interrupt_2, INPUT_PULLUP);

  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_1), Encoder1Callback, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_interrupt_2), Encoder2Callback, RISING);
}

void loop() {
  // 
  if (Serial.available())
  {
    char chr = Serial.read();
    // start count
    if(chr == 'r')
    {
    encoder_counter1 = 0;
    encoder_counter2 = 0;
    }
  
  }
delay(1000);
Serial.println("count 1 =     " + String(encoder_counter1) + "      counter 2 =     " + String(encoder_counter2));
Serial.println(" ");

}

// New pulse from Right Wheel Encoder
void Encoder1Callback()
{
  encoder_counter1++;
}

// New pulse from Left Wheel Encoder
void Encoder2Callback()
{
  encoder_counter2++;
}

