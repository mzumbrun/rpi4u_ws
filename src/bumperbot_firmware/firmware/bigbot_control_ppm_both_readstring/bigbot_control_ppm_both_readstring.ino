// POR code for both left and right arduinos for bigbot
// to do - remove dc_high pin, use input_pullup. not urgent
// 09/05/2024 - changed serial read from ROS to read entire string
// 08/21/2024 - initial code


#include <PID_v1.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define motor_ppm_pin 9    // ppm control signal to esc
#define motor_select 6     // connect to dc_high for right motor, dc_lowfor left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt
#define H2 3
#define H3 7
#define rxPin 10
#define txPin 11

Servo bigbot_servo;
SoftwareSerial hardwire_port(rxPin, txPin);

// Encoders
unsigned long encoder_count_ = 0;
unsigned long last_millis = 0;
const unsigned long interval = 100;
unsigned long real_interval = 0;
char len = "1";

// Interpret Serial Messages
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
bool is_wheel_forward = true;
String value = "00.00";
String ROS_input = "rp00.00,lp00.00,";
String encoder_read = "rp00.00,";
char wheel_side[] = "right";
bool is_right = true;

// speed control
int max_pos_speed = 1500;  //  value from CALIBRATION to max rad/s from ROS
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
  pinMode(H2, INPUT_PULLUP);
  pinMode(H3, INPUT_PULLUP);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, FALLING);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    max_pos_speed = 2450;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    max_neg_speed = 1000;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
    Motor.SetTunings(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    max_pos_speed = 2450;
    max_neg_speed = 1000;
    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;
    Motor.SetTunings(Kp, Ki, Kd);
  }
  Serial.begin(115200);
  hardwire_port.begin(115200);
  bigbot_servo.writeMicroseconds(1500);  // start with motors off
}

void loop() {

  // format from ros: "rdxx.xx,ldxx.xx,"
  if (Serial.available() > 0) {
    ROS_input = Serial.readStringUntil('X');  // \0 is null character  \n is new line
    if (wheel_side[0] == 'r') {               // if true, then get its direction and speed
      is_right = true;
      wheel_sign = ROS_input[1];
      value[0] = ROS_input[2];
      value[1] = ROS_input[3];
      value[2] = ROS_input[4];
      value[3] = ROS_input[5];
      value[4] = ROS_input[6];
      value[5] = '\0';
    }
    if (wheel_side[0] == 'l') {  // if true, then get its direction and speed
      wheel_sign = ROS_input[9];
      value[0] = ROS_input[10];
      value[1] = ROS_input[11];
      value[2] = ROS_input[12];
      value[3] = ROS_input[13];
      value[4] = ROS_input[14];
      value[5] = '\0';
    }
    wheel_cmd_vel = value.toFloat();
    if (wheel_sign == "p") {
      is_wheel_forward = true;
    } else {
      is_wheel_forward = false;
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  real_interval = current_millis - last_millis;
  if (real_interval >= interval) {
    last_millis = current_millis;
    wheel_meas_vel = (1000./real_interval) * encoder_count_ * (60.0 / 35.) * 0.10472;  //  rads/sec

    // hardwire_port.write(encoder_count_);
    //Motor.Compute();  // output is wheel_cmd in rad/s

    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }
    // *** UNTIL ENCODER CAN WORK, SEND BACK TO ROS SAME AS INPUT. ULTIMATE CHANGE TO wheel_meas_vel
    if (is_right) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";
      hardwire_port.write('r');
    //  encoder_count_ = 12899;
      String enc = String(encoder_count_);
      len = (char)enc.length();
      hardwire_port.write(len);
      hardwire_port.write(enc.c_str());
    } else {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";
      hardwire_port.write('l');
     // encoder_count_ = 34045;
      String enc = String(encoder_count_);
      len = (char)enc.length();
      hardwire_port.write(len);
      hardwire_port.write(enc.c_str());
    }
    encoder_count_ = 0;
    Serial.println(encoder_read);

    //*****************************************************
    if (is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd_vel, 0, 10, 1500, max_pos_speed));
      // wheel_sign = "p";
    }
    if (!is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd_vel, 0, 10, 1500, max_neg_speed));
      // wheel_sign = "n";
    }
  }
}


// New pulse from Left Wheel Encoder
void EncoderCallback() {
  encoder_count_++;
}
