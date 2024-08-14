// 8/14/24 changed interrupt to states
// 8/13/24 updated code for faster interrupts

#include <PID_v1.h>

#define L298N_enA 9  // PWM Motor 2A - 
#define L298N_in1 12  // Dir Motor A
#define L298N_in2 13  // Dir Motor A
#define L298N_enB 11  // PWM Motor 1B
#define L298N_in3 7  // Dir Motor B
#define L298N_in4 8  // Dir Motor B
#define left_encoder_phaseA 2   // Interrupt was pin 2
#define right_encoder_phaseA 3  // Interrupt was pin 3

// Encoders
volatile bool state_right= false;
volatile bool state_left = false;
unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
String right_wheel_sign = "p";  // 'p' = positive, 'n' = negative
String left_wheel_sign = "p";  // 'p' = positive, 'n' = negative
unsigned long last_millis = 0;
const unsigned long interval = 200;

// Interpret Serial Messages
bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
char value[] = "00.00";
uint8_t value_idx = 0;
bool is_cmd_complete = false;

// PID
// Setpoint - Desired
double right_wheel_cmd_vel = 0.0;     // rad/s
double left_wheel_cmd_vel = 0.0;      // rad/s
// Input - Measurement
double right_wheel_meas_vel = 0.0;    // rad/s
double left_wheel_meas_vel = 0.0;     // rad/s
// Output - Command
double right_wheel_cmd = 0.0;             // 0-255
double left_wheel_cmd = 0.0;              // 0-255
// Tuning
double Kp_r = 12.8; // orig 11.5
double Ki_r = 8.3;  // orig 7.5
double Kd_r = 0.1;  // orig 0.1
double Kp_l = 12.8; // orig 12.8
double Ki_l = 8.3;  // orig 8.3
double Kd_l = 0.1;  // orig 0.1
// Controller
PID rightMotor(&right_wheel_meas_vel, &right_wheel_cmd, &right_wheel_cmd_vel, Kp_r, Ki_r, Kd_r, DIRECT);
PID leftMotor(&left_wheel_meas_vel, &left_wheel_cmd, &left_wheel_cmd_vel, Kp_l, Ki_l, Kd_l, DIRECT);

void setup() {
  // 
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);
  pinMode(right_encoder_phaseA, INPUT_PULLUP);
  pinMode(left_encoder_phaseA, INPUT_PULLUP);
  //pinMode(right_encoder_phaseB, INPUT_PULLUP);
  //pinMode(left_encoder_phaseB, INPUT_PULLUP);
  // right then left has right working
  // left then right has still right working
  //attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, FALLING); //was RISING
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallback, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallback, FALLING); //was RISING
  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  rightMotor.SetMode(AUTOMATIC);
  leftMotor.SetMode(AUTOMATIC);

  Serial.begin(115200);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
      is_cmd_complete = false;
    }
    // Left Wheel Mo tor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
        right_wheel_sign = "p";
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
        left_wheel_sign = "p";
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
        right_wheel_sign = "n";
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
        left_wheel_sign = "n";
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
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
    else
    {
      if(value_idx < 5)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
    noInterrupts();
    if(state_right){
      right_encoder_counter++;
      state_right = false;
    }
    if(state_left){
      left_encoder_counter++;
      state_left = false;
    }
    interrupts();
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    right_wheel_meas_vel = (5 * right_encoder_counter * (60.0/110.)) * 0.10472; //110 was 385
    left_wheel_meas_vel =  (5* left_encoder_counter  * (60.0/110.)) * 0.10472;
     
    rightMotor.Compute();
    leftMotor.Compute();

    // Ignore commands smaller than inertia
    if(right_wheel_cmd_vel == 0.0)
    {
      right_wheel_cmd = 0.0;
    }
    if(left_wheel_cmd_vel == 0.0)
    {
      left_wheel_cmd = 0.0;
    }

    String encoder_read = "r" + right_wheel_sign + String(right_wheel_meas_vel) + ",l" + left_wheel_sign + String(left_wheel_meas_vel) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;

    //ignore PID
    //right_wheel_cmd = 10* right_wheel_cmd_vel;
    //left_wheel_cmd = 10* left_wheel_cmd_vel;

    analogWrite(L298N_enA, right_wheel_cmd);
    analogWrite(L298N_enB, left_wheel_cmd);
  }
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  state_right = true;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  state_left = true;
}
