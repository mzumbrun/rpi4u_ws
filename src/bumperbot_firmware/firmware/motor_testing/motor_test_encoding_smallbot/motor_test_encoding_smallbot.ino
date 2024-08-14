// 8/11/2024 modified udemy code for measuring encoder values
#include <PID_v1.h>

// Wheel Encoders Connection PINs
// motor 1B
#define left_encoder_phaseA 2   // Interrupt was pin 2
#define left_encoder_phaseB 4
// motor 2A
#define right_encoder_phaseA 3  // Interrupt was pin 3
#define right_encoder_phaseB 5 

// Encoders
unsigned int right_encoder_counterA = 0;
unsigned int left_encoder_counterA = 0;
unsigned int right_encoder_counterB = 0;
unsigned int left_encoder_counterB = 0;



void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(right_encoder_counterA, INPUT_PULLUP);
  pinMode(left_encoder_counterA, INPUT_PULLUP);
  pinMode(right_encoder_counterB, INPUT_PULLUP);
  pinMode(left_encoder_counterB, INPUT_PULLUP);


  Serial.begin(115200);

  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // 
  if (Serial.available())
  {
    char chr = Serial.read();
    // start count
    if(chr == 'r')
    {
    right_encoder_counterA = 0;
    left_encoder_counterA = 0;
    right_encoder_counterB = 0;
    left_encoder_counterB = 0;
    }
  
  }
  delay(2000);
Serial.println("left count A = " + String(left_encoder_counterA) + "left count B = " + String(left_encoder_counterB));
Serial.println("right count A = " + String(right_encoder_counterA) + "right count B = " + String(right_encoder_counterB));
Serial.println(" ");

}

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  right_encoder_counterA++;
  right_encoder_counterB++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  left_encoder_counterA++;
  left_encoder_counterB++;
}
