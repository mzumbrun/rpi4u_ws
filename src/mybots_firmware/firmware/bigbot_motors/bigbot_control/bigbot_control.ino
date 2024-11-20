// POR code for both left and right arduinos for bigbot
// to do - remove dc_high pin, use input_pullup. not urgent
// 11/20/2024 - removed softserial, removed string to end with #
// 09/16/2024 - enabled PID and encoder
// 09/05/2024 - changed serial read from ROS to read entire string
// 08/21/2024 - initial code


#include <PID_v1.h>
#include <Servo.h>

#define motor_ppm_pin 9    // ppm control signal to esc
#define motor_select 6     // connect to dc_high for right motor, dc_low for left motor
#define dc_high 5          // driven high - right
#define dc_low 4           // driven low - left
#define encoder_counter 2  // Interrupt
#define H2 3
#define H3 7

Servo bigbot_servo;

// Encoders
unsigned long encoder_count_ = 0;
unsigned long last_millis = 0;
const unsigned long interval = 100;
unsigned long real_interval = 0;
char len = "1";

// Interpret Serial Messages
String wheel_sign = "p";  // 'p' = positive, 'n' = negative
bool is_wheel_cmd = false;
bool is_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;
String encoder_read = "rp00.00,";
char wheel_side[] ="right";
bool is_right = true;
char chr= "x";

// speed control
int max_pos_speed = 1500;  //  value from CALIBRATION to max rad/s from ROS
int max_neg_speed = 1500;
int max_rads_per_sec = 8;     // corresponds to that in ROS
double wheel_cmd_vel = 0.0;   // setpoint from ROS_CONTROL rad/s
double wheel_meas_vel = 0.0;  // Measured from motor encoders, rad/s
double wheel_cmd = 0.0;       // output from PID to send to motor
double Kp = 1.;               // orig 12.8
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
  attachInterrupt(digitalPinToInterrupt(encoder_counter), EncoderCallback, FALLING);
  digitalWrite(dc_low, LOW);
  digitalWrite(dc_high, HIGH);

  if (digitalRead(motor_select) == HIGH) {
    wheel_side[0] = 'r';
    is_right = true;
    max_pos_speed = 2000;  // corresponds to max rad/s for RIGHT motor to match max provided by ROS
    max_neg_speed = 1000;
    Kp = 10.;
    Ki = .8;
    Kd = 0.1;
    Motor.SetTunings(Kp, Ki, Kd);
  } else {
    wheel_side[0] = 'l';
    is_right = false;
    max_pos_speed = 2000;
    max_neg_speed = 1000;
    Kp = 10.;
    Ki = .8;
    Kd = 0.1;
    Motor.SetTunings(Kp, Ki, Kd);
  }
  Serial.begin(115200);
  bigbot_servo.writeMicroseconds(1500);  // start with motors at zero speed
}

void loop() {

  // format from ros: "rdxx.xx,ldxx.xx,"
  if (Serial.available() > 0) {
    chr = Serial.read();
 // \0 is null character  \n is new line
    if (chr == wheel_side[0]) {               
      is_wheel_cmd = true;
      value_idx = 0;
      is_cmd_complete = false;
    }
    else if (chr == 'p') {
     if (is_wheel_cmd) {
      wheel_sign ="p";
      is_wheel_forward = true;
     }
    }
    else if (chr == 'n') {
     if (is_wheel_cmd) {
      wheel_sign ="n";
      is_wheel_forward = false;
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
  }

  // Encoder
  unsigned long current_millis = millis();
  real_interval = current_millis - last_millis;
  if (real_interval >= interval) {
    last_millis = current_millis;
  
    wheel_meas_vel = 1.*(float(encoder_count_)/float(real_interval)) * (60.0 / 35.) * 0.10472;  //  rads/sec

    Motor.Compute();  // output is wheel_cmd 0-255

    if (wheel_cmd_vel == 0.0) {  // if setpoint is 0, then make sure cmd to wheels is 0
      wheel_cmd = 0.0;
    }
    if ('r'== wheel_side[0]) {
      encoder_read = "r" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    } else if ('l' == wheel_side[0]) {
      encoder_read = "l" + wheel_sign + String(wheel_meas_vel, 2) + ",";

    }
    encoder_count_ = 0;
    Serial.println(encoder_read);

    //*****************************************************
    if (is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd, 0, 255, 1500, max_pos_speed));
      // wheel_sign = "p";
    }
    if (!is_wheel_forward) {
      bigbot_servo.writeMicroseconds(map(wheel_cmd, 0, 255, 1500, max_neg_speed));
      // wheel_sign = "n";
    }
  }
}


// New pulse from Left Wheel Encoder
void EncoderCallback() {
  encoder_count_++;
}
