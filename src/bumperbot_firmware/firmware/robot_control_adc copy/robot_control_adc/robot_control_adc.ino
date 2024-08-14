// base code from Udemy locomotion class
// 7/22/24
// real robot motor 2 is on the right side
// real robot motor 1 is on the left side

#include <PID_v1.h>

// clip pwm so nano does't output beyond 3.3V to ESC
unsigned int clip = 168;  //  5V*168/255 = 3.3V

// vesc Connection PINs
// motor M1
#define pin_m1_adc1 5  // connect to m1_adc1 on esc for M1 << SPEED CONTROL >>
#define pin_m1_rx 11    // connect to RX/SDA on ESC for M1 << DIRECTIONAL CONTROL >>
// motor M2
#define pin_m2_adc1 6  // connect to m2_adc1 on ESC for M2 <<SPEED CONTROL>>
#define pin_m2_rx 10   // connect to RX/SDA on ESC for M2  << DIRECTIONAL CONTROL>>

// Wheel Encoders Connection PINs
#define m2_encoder_phaseA 3 // Interrupt
#define m2_encoder_phaseB 9
#define m1_encoder_phaseA 2  // Interrupt
#define m1_encoder_phaseB 4

// Encoders
unsigned int m1_encoder_counter = 0;
unsigned int m2_encoder_counter = 0;
String m1_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String m2_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 100;  // the desired time between encoder measurements, msec
double real_interval = 0.;           // the real time between encoder measurements

// Interpret Serial Messages
bool is_m2_wheel_cmd = false;
bool is_m1_wheel_cmd = false;
bool is_m2_wheel_forward = true;
bool is_m1_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double m2_wheel_cmd_vel = 0.0;  // rad/s
double m1_wheel_cmd_vel = 0.0;  // rad/s
// Input - Measurement
double m2_wheel_meas_vel = 0.0;  // rad/s
double m1_wheel_meas_vel = 0.0;  // rad/s
// Output - Command
double m2_wheel_cmd = 0.0;  // 0-255
double m1_wheel_cmd = 0.0;  // 0-255
// Tuning
double Kp_r = 1.; // was 11.5
double Ki_r = 0.; // was 7.5
double Kd_r = 0.; // was 0.1
double Kp_l = 12.8;
double Ki_l = 8.3;
double Kd_l = 0.1;
// Controller
// PID syntax
// &input, &output, &setpoint, Kp, Ki, Kd, Pon, Direction
// &input is variable we are trying to control
// &output is variable that will be adjusted by the PID
// &setpoint is value we want to maintain
// Kp, Ki Kd are tuning parameters (emperical or guesses)
// Direction is either DIRECT or REVERSE
// Pon is P_ON_E by default or P_ON_M allows proportional on measurement
PID rightMotor(&m2_wheel_meas_vel, &m2_wheel_cmd, &m2_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&m1_wheel_meas_vel, &m1_wheel_cmd, &m1_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {

  // Connection to ESC controllers
  // motor M1
  pinMode(pin_m1_adc1, OUTPUT);
  pinMode(pin_m1_rx, OUTPUT);
  // motor M2
  pinMode(pin_m2_adc1, OUTPUT);
  pinMode(pin_m2_rx, OUTPUT);

  // Set Motor Rotation Direction
  analogWrite(pin_m2_rx, clip);
  analogWrite(pin_m1_rx, clip);

  // start with motors stopped
  analogWrite(pin_m1_adc1, 0);
  analogWrite(pin_m2_adc1, 0);

  // enable PID
  // AUTOMATIC = ON; MANUAL = OFF
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);
  Serial.begin(115200);

  // Init encoders
  pinMode(m2_encoder_phaseB, INPUT);
  pinMode(m1_encoder_phaseB, INPUT);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(m2_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(m1_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available()) {
    char chr = Serial.read();
    // Right Wheel Motor
    if (chr == 'r') {
      is_m2_wheel_cmd = true;
      is_m1_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Motor
    else if (chr == 'l') {
      is_m2_wheel_cmd = false;
      is_m1_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if (chr == 'p') {
      if (is_m2_wheel_cmd && !is_m2_wheel_forward) {
        // change the direction of the rotation
        analogWrite(pin_m2_rx, clip);
        is_m2_wheel_forward = true;
      } else if (is_m1_wheel_cmd && !is_m1_wheel_forward) {
        // change the direction of the rotation
        analogWrite(pin_m1_rx, clip);

        is_m1_wheel_forward = true;
      }
    }
    // Negative direction
    else if (chr == 'n') {
      if (is_m2_wheel_cmd && is_m2_wheel_forward) {
        // change the direction of the rotation
        analogWrite(pin_m2_rx, 0);
        is_m2_wheel_forward = false;
      } else if (is_m1_wheel_cmd && is_m1_wheel_forward) {
        // change the direction of the rotation
        analogWrite(pin_m1_rx, 0);

        is_m1_wheel_forward = false;
      }
    }
    // Separator
    else if (chr == ',') {
      if (is_m2_wheel_cmd) {
        m2_wheel_cmd_vel = atof(value);
      } else if (is_m1_wheel_cmd) {
        m1_wheel_cmd_vel = atof(value);
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
  if (real_interval >= interval)

  // 60 = min to sec
  // 1 = encoder pulses per revolution
  // 5 = gearbox ration
  // .10472 = converts rpm to rads/sec


  {
    m2_wheel_meas_vel = (1000. / real_interval) * m2_encoder_counter * (60.0 / (75. * 5.)) * 0.10472;  // rad/s
    m1_wheel_meas_vel = (1000. / real_interval) * m1_encoder_counter * (60.0 / (1. * 5.)) * 0.10472;

    rightMotor.Compute();
    leftMotor.Compute();

    // Ignore commands smaller than inertia
    if (m2_wheel_cmd_vel == 0.0) {
      m2_wheel_cmd = 0.0;
    }
    if (m1_wheel_cmd_vel == 0.0) {
      m1_wheel_cmd = 0.0;
    }

    //
    // send data to ROS2 over serial port
    String encoder_read = "r" + m2_wheel_sign + String(m2_wheel_meas_vel) + ",l" + m1_wheel_sign + String(m1_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    //
    //
    last_millis = current_millis;
    m2_encoder_counter = 0;
    m1_encoder_counter = 0;

    analogWrite(pin_m2_adc1, 0.1*m2_wheel_cmd);   //was 0.66
    analogWrite(pin_m1_adc1, 0.66*m1_wheel_cmd);
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback() {
  if (digitalRead(m2_encoder_phaseB) == HIGH) {
    m2_wheel_sign = "p";
  } else {
    m2_wheel_sign = "n";
  }
  m2_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback() {
  if (digitalRead(m1_encoder_phaseB) == HIGH) {
    m1_wheel_sign = "n";
  } else {
    m1_wheel_sign = "p";
  }
  m1_encoder_counter++;
}
