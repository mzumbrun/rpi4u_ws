// 7/30/24 changing to ppm from adc control
// 7/22/24 base code for Udemy locomotion class
// real robot motor 2 is on the right side
// real robot motor 1 is on the left side

#include <PID_v1.h>
#include <Servo.h>

// vesc Connection PINs
#define pin_m1_ppm 5  // connect to m1_ppm on esc for M1 << SPEED CONTROL >>
#define pin_m2_ppm 6  // connect to m2_ppm on ESC for M2 <<SPEED CONTROL>>

// Wheel Encoders Connection PINs
#define m2_encoder_phaseA 3  // Interrupt
// #define m2_encoder_phaseB 9
#define m1_encoder_phaseA 2  // Interrupt
// #define m1_encoder_phaseB 4

Servo m1_servo;
Servo m2_servo;


// Encoders
int m1_encoder_counter = 0;
unsigned int m2_encoder_counter = 0;
String m1_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String m2_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 50;  // the desired time between encoder measurements, msec
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
int m1_wheel_speed = 0;
int m2_wheel_speed = 0;

// Tuning
double Kp_r = 12.;  // was 11.5
double Ki_r = 0.;   // was 7.5
double Kd_r = 0.;   // was 0.1
double Kp_l = 1.;   // was 12.8
double Ki_l = 0.;   //was 8.3
double Kd_l = 0.;   //was 0.1
// Controller
// PID syntax
// &input is variable we are trying to control
// &output is variable that will be adjusted by the PID
// &setpoint is value we want to maintain
// Kp, Ki Kd are tuning parameters (emperical or guesses)
// Direction is either DIRECT or REVERSE
// Pon is P_ON_E by default or P_ON_M allows proportional on measurement
//                 &input,            &output,        &setpoint,      Kp,  Ki,    Kd, Pon, Direction
//                 rad/s              0-255           rad/s
PID rightMotor(&m2_wheel_meas_vel, &m2_wheel_cmd, &m2_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&m1_wheel_meas_vel, &m1_wheel_cmd, &m1_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {

  m1_servo.attach(pin_m1_ppm);
  m2_servo.attach(pin_m2_ppm);
  // start with motors stopped
  m1_servo.writeMicroseconds(1500);
  m2_servo.writeMicroseconds(1500);

  // enable PID
  // AUTOMATIC = ON; MANUAL = OFF
  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  // start serial port
  Serial.begin(115200);

  // Init encoders
  // pinMode(m2_encoder_phaseB, INPUT);
  pinMode(m1_encoder_phaseA, INPUT_PULLUP);
  pinMode(m2_encoder_phaseA, INPUT_PULLUP);
  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(m2_encoder_phaseA), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(m1_encoder_phaseA), leftEncoderCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available()) {
    // while (Serial.available() >0) {
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
    else if (chr == 'p' && is_m2_wheel_cmd) {
      is_m2_wheel_forward = true;
    } else if (chr == 'p' && is_m1_wheel_cmd) {
      is_m1_wheel_forward = true;
    }

    // Negative direction
    else if (chr == 'n' && is_m2_wheel_cmd) {
      is_m2_wheel_forward = false;
    } else if (chr == 'n' && is_m1_wheel_cmd) {
      is_m1_wheel_forward = false;
    }

    // wheel velocity
    else if (chr == ',') {
      if (is_m2_wheel_cmd) {
        m2_wheel_cmd_vel = atof(value);  // is this rad/s
        m2_wheel_cmd_vel = map(m2_wheel_cmd_vel, 0, 1, 0, 200);
      } else if (is_m1_wheel_cmd) {
        m1_wheel_cmd_vel = atof(value);
        m1_wheel_cmd_vel = map(m1_wheel_cmd_vel, 0, 1, 0, 200);
        is_cmd_complete = true;
      }
     // Serial.println(value);
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
  // real_interval = current_millis - last_millis;
  if (current_millis - last_millis >= interval) {

    // 60 = min to sec
    // 14 = encoder pulses per revolution
    // 5 = gearbox ratio
    // .10472 = converts rpm to rads/sec

    m2_wheel_meas_vel =  m2_encoder_counter * (60.0 / (14. * 5.)) * 0.10472;  // rad/s
    m1_wheel_meas_vel =  m1_encoder_counter * (60.0 / (14. * 5.)) * 0.10472;
    //m1_wheel_meas_vel = m1_wheel_cmd;

   // rightMotor.Compute();  //yields output 0-255
   // leftMotor.Compute();

    // Ignore commands smaller than inertia
    if (m2_wheel_cmd_vel == 0.0) {
      m2_wheel_cmd = 0.0;
    }
    if (m1_wheel_cmd_vel == 0.0) {
      m1_wheel_cmd = 0.0;
    }
    //
    m1_wheel_meas_vel=m1_wheel_cmd;
    m2_wheel_meas_vel=m2_wheel_cmd;

    // send data to ROS2 over serial port
    String encoder_read = "r" + m2_wheel_sign + String(m2_wheel_meas_vel) + ",l" + m1_wheel_sign + String(m1_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    //5189

   // encoder_read = m1_wheel_sign + "M1cmd=  " + String(m1_wheel_cmd,5) + "count=  " + String(m1_encoder_counter, 5) + "  M1meas =  " + String(m1_wheel_meas_vel,5) + ",";
   // Serial.println(encoder_read);
    //
   // encoder_read = m2_wheel_sign + "M2cmd=  " + String(m2_wheel_cmd) + "count=  " + String(m2_encoder_counter, 5) + "  M2meas =  " + String(m2_wheel_meas_vel) + ",";
   // Serial.println(encoder_read);
    //
    //
    last_millis = current_millis;
    m2_encoder_counter = 0.;
    m1_encoder_counter = 0.;

    //change to m/s
    m1_wheel_speed = (int)m1_wheel_cmd_vel; // TAKE OUT VEL
    m2_wheel_speed = (int)m2_wheel_cmd_vel;


    if (is_m1_wheel_cmd && is_m1_wheel_forward) {
      m1_servo.writeMicroseconds(map(m1_wheel_speed, 0, 255, 1500, 2450));
      //m1_servo.writeMicroseconds(1800);
      m1_wheel_sign = "p";
    }
    if (is_m1_wheel_cmd && !is_m1_wheel_forward) {
      m1_servo.writeMicroseconds(map(m1_wheel_speed, 0, 255, 1500, 600));
      //m1_servo.writeMicroseconds(1500);
      m1_wheel_sign = "n";
    }
    if (is_m2_wheel_cmd && is_m2_wheel_forward) {
      m2_servo.writeMicroseconds(map(m2_wheel_speed, 0, 255, 1500, 2450));
      m2_wheel_sign = "p";
    }
    if (is_m2_wheel_cmd && !is_m2_wheel_forward) {
      m2_servo.writeMicroseconds(map(m2_wheel_speed, 0, 255, 1500, 600));
      m2_wheel_sign = "n";
    }
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback() {
  // if (digitalRead(m2_encoder_phaseB) == HIGH) {
  //   m2_wheel_sign = "p";
  // } else {
  //  m2_wheel_sign = "n";
  // }
  m2_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback() {
  // if (digitalRead(m1_encoder_phaseB) == HIGH) {
  //   m1_wheel_sign = "n";
  // } else {
  //   m1_wheel_sign = "p";
  // }
  m1_encoder_counter++;
}
