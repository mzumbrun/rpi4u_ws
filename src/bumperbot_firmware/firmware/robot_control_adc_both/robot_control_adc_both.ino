// 8/15/2024 -
// left motor 1B uses nano 33 iot 'l'
// right motor 2A uses nano 'r'
// change line 14 and select correct board

#include <PID_v1.h>

#define motor_pwm_pin 9    // PWM Motor 1B
#define motor_forward 12   // Dir Motor B
#define motor_backward 7  // Dir Motor B
#define motor_select 6     // connect to pin 5 for right motor (NANO) ... connect to GND for left motor (NANO 33 IOT)
#define dc_high 5          // driven high 

#define encoder_counter 2  // Interrupt

// Encoders
volatile bool state = false;
unsigned int encoder_count_ = 0;
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;

// Interpret Serial Messages
bool is_wheel_cmd = false;
bool is_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
String encoder_read = "rp00.00,";
char wheel_side[] ="right";
bool is_right = true;

// PID
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double Kp = 12.0;             // orig 12.8
double Ki = 8.0;              // orig 8.3
double Kd = 0.1;              // orig 0.1
PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);

void setup() {
  //
  pinMode(motor_pwm_pin, OUTPUT);
  pinMode(motor_forward, OUTPUT);
  pinMode(motor_backward, OUTPUT);
  pinMode(motor_select, INPUT);
  pinMode(encoder_counter, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, RISING);
  digitalWrite(motor_forward, HIGH);
  digitalWrite(motor_backward, LOW);
  digitalWrite(dc_high, HIGH);
  delay(100);
    // Read and Interpret Wheel Velocity Commands
  if (digitalRead(motor_select) == HIGH) {
    is_right = true;
    wheel_side[0] = 'r';
  }
  else {
    wheel_side[0] = 'l';
    is_right = false;
  }

  Motor.SetMode(AUTOMATIC);
  Serial.begin(115200);
}

void loop() {

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
    else if (chr == ',') {
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
    wheel_meas_vel = (10 * encoder_count_ * (60.0 / 110.)) * 0.10472;
    encoder_count_ = 0;
    Motor.Compute();

    // if setpoint is 0, then make sure cmd to wheels is 0
    if (wheel_cmd_vel == 0.0) {
      wheel_cmd = 0.0;
    }

    //  wheel_cmd = map((int)wheel_cmd, 0, 15, 0, 200);
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
