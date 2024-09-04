// POR code for both left and right arduinos for bigbot
// to do - remove dc_high pin, use input_pullup. not urgent
// 08/21/2024 - initial code


#include <PID_v1.h>
#include <Servo.h>

#define motor_ppm_pin 9    // ppm control signal to esc
#define motor_select 6     // connect to dc_high for right motor, dc_lowfor left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt

Servo bigbot_servo;

// Encoders
volatile bool state = false;
unsigned int encoder_count_ = 0;
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
int max_pos_speed = 1500;            //  value from CALIBRATION to max rad/s from ROS
int max_neg_speed = 1500;
int max_rads_per_sec = 8;     // corresponds to that in ROS
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double Kp = 0.;               // orig 12.8
double Ki = 0.;               // orig 8.3
double Kd = 0.;               // orig 0.1
PID Motor(&wheel_meas_vel, &wheel_cmd, &wheel_cmd_vel, Kp, Ki, Kd, DIRECT);

void setup() {

  Motor.SetMode(AUTOMATIC);

  bigbot_servo.attach(motor_ppm_pin);
  pinMode(motor_ppm_pin, OUTPUT);
  bigbot_servo.writeMicroseconds(1500);  // start the motor at 0 speed

  pinMode(dc_low, OUTPUT);
  pinMode(dc_high, OUTPUT);
  pinMode(motor_select, INPUT);
  pinMode(encoder_counter, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, RISING);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    max_pos_speed = 2000;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    max_neg_speed = 1000;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
    Motor.SetTunings(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    max_pos_speed = 2000;
    max_neg_speed = 1000;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
    Motor.SetTunings(Kp, Ki, Kd);
  }
  Serial.begin(115200);
}

void loop() {

  if (Serial.available()) {
    char chr = Serial.read();
    if (chr == wheel_side[0]) {  // if true, then command is for this motor
      is_wheel_cmd = true;
      value_idx = 0;
      //      is_cmd_complete = false;
    } else if (chr == 'p') {
      if (is_wheel_cmd && !is_wheel_forward) {  //want 'p' but going opposite, so need to change direction
        is_wheel_forward = true;
        wheel_sign = "p";
      }
    } else if (chr == 'n') {
      if (is_wheel_cmd && is_wheel_forward) {  //want 'n' but going opposite, so need to change direction
        is_wheel_forward = false;
        wheel_sign = "n";
      }
    } else if (chr == ',') {  // if ',' then command is over format is 'xyz.zz', x=wheel, y=dir, z = speed
      if (is_wheel_cmd) {
        wheel_cmd_vel = atof(value);
        //       is_cmd_complete = true;
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
    wheel_meas_vel = (10 * encoder_count_ * (60.0 / 35.)) * 0.10472;  //  rads/sec
    encoder_count_ = 0;
   // Motor.Compute();  // output is wheel_cmd in rad/s

    if (wheel_cmd_vel == 0.0) {      // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }

    if (is_right) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel) + ",";
    } else {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel) + ",";
    }
    Serial.println(encoder_read);

    //*****************************************************
    if (is_wheel_cmd && is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd_vel, 0, 10, 1500, max_pos_speed));
      wheel_sign = "p";
    }
    if (is_wheel_cmd && !is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd_vel, 0, 10, 1500, max_neg_speed));
      wheel_sign = "n";
    }
  }
}


// New pulse from Left Wheel Encoder
void EncoderCallback() {
  state = true;
}
