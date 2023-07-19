#include <PID_v1.h>

// Pin
const int potentiometerPin = A8;
const int motor_f_1 = 2;
const int motor_f_2 = 3;
int motor_b_l_1 = 4;
int motor_b_l_2 = 5;
int motor_b_r_1 = 7;
int motor_b_r_2 = 6;

// 속도
int Deceleration_threshold = 20;
int speed;
int speed_offset = 255;

// PID
// 앞바퀴
double setpoint;
double pid_input, pid_output;
const double consKp = 4.0;
const double consKi = 0.011;
const double consKd = 0.125;
const double aggKp = 10;
const double aggKi = 0.0075;
const double aggKd = 0.6;
const double A_chng_max = 30;
const double W_chng_max = 40;
const double chng_max = 30;

// 폭에 의한 제어값 변화량
double W_setpoint = 530;
double W_pid_input, W_chng_output;
const double W_consKp = 1.4;
const double W_consKi = 0.01;
const double W_consKd = 0.1;

// 각도에 의한 제어값 변화량
double A_setpoint = 60;
double A_pid_input, A_chng_output;
const double A_consKp = 2;
const double A_consKi = 0.01;
const double A_consKd = 0.1;

double CarLine_W, CarLine_A;
double chng_sum;
const double N_setpoint = 570;

PID pid(&pid_input, &pid_output, &setpoint, consKp, consKi, consKd, DIRECT);
PID W_pid(&W_pid_input, &W_chng_output, &W_setpoint, W_consKp, W_consKi, W_consKd, DIRECT);
PID A_pid(&A_pid_input, &A_chng_output, &A_setpoint, A_consKp, A_consKi, A_consKd, DIRECT);

void setup() {
  pinMode(motor_f_1, OUTPUT);
  pinMode(motor_f_2, OUTPUT);
  pinMode(motor_b_l_1, OUTPUT);
  pinMode(motor_b_l_2, OUTPUT);
  pinMode(motor_b_r_1, OUTPUT);
  pinMode(motor_b_r_2, OUTPUT);
  Serial.begin(9600);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);

  W_pid.SetMode(AUTOMATIC);
  W_pid.SetOutputLimits(-W_chng_max, W_chng_max);

  A_pid.SetMode(AUTOMATIC);
  A_pid.SetOutputLimits(-A_chng_max, A_chng_max);
}

// Function to implement obstacle avoidance
void obstacle_avoidance() {
  // Stop the car
  analogWrite(motor_f_1, 0);
  analogWrite(motor_f_2, 0);
  analogWrite(motor_b_l_1, 0);
  analogWrite(motor_b_l_2, 0);
  analogWrite(motor_b_r_1, 0);
  analogWrite(motor_b_r_2, 0);

  delay(1000);  // Wait for 1 second

  // Turn left for 4 seconds (assuming you have appropriate motor control logic in Arduino)
  analogWrite(motor_f_1, 0);
  analogWrite(motor_f_2, 255);
  analogWrite(motor_b_l_1, 0);
  analogWrite(motor_b_l_2, 255);
  analogWrite(motor_b_r_1, 0);
  analogWrite(motor_b_r_2, 0);

  delay(4000);

  // Turn right for 4 seconds
  analogWrite(motor_f_1, 255);
  analogWrite(motor_f_2, 0);
  analogWrite(motor_b_l_1, 255);
  analogWrite(motor_b_l_2, 0);
  analogWrite(motor_b_r_1, 0);
  analogWrite(motor_b_r_2, 0);

  delay(4000);

  // Resume normal operation
}

void loop() {
  if (Serial.available()) {
    String receivedString = Serial.readStringUntil('\n');
    int commaIndex = receivedString.indexOf(',');
    String line_angle_input_str = receivedString.substring(0, commaIndex);
    String width_input_str = receivedString.substring(commaIndex + 1);
    CarLine_A = line_angle_input_str.toFloat();
    CarLine_W = width_input_str.toFloat();

    // Check if the Python code sent "0" to indicate obstacle avoidance
    if (CarLine_A == 0 && CarLine_W == 0) {
      // Implement the obstacle avoidance logic here
      obstacle_avoidance();

    } else {
      // Implement the existing PID control logic here.

      //제어 인풋 없을 경우 직진
      if (!W_pid_input && !A_pid_input) {
        setpoint = N_setpoint;
      }
      // C 구간 극복 (값이 이상하게 받아질 경우 직진)
      else if (W_pid_input >= 3000) {
        setpoint = N_setpoint;
      } else {
        chng_sum = W_chng_output + A_chng_output;
        chng_sum = constrain(chng_sum, -chng_max, chng_max);
        pid_input = analogRead(potentiometerPin);
        setpoint = N_setpoint + chng_sum;
      }
      double gap = abs(setpoint - pid_input);
      if (gap > 8) {
        pid.SetTunings(consKp, consKi, consKd);
      } else {
        pid.SetTunings(aggKp, aggKi, aggKd);
      }
      pid.Compute();

      // 앞바퀴 DC 모터 회전 제어
      if (pid_output > 0) {
        analogWrite(motor_f_1, pid_output);
        analogWrite(motor_f_2, 0);
      } else if (pid_output < 0) {
        analogWrite(motor_f_1, 0);
        analogWrite(motor_f_2, -pid_output);
      } else {
        analogWrite(motor_f_1, 0);
        analogWrite(motor_f_2, 0);
      }
      //급회전시 감속
      if (abs(pid_output) >= chng_max - 1) {
        speed = speed_offset / 4;
      } else {
        speed = speed_offset / 2;
      }

      // 뒷바퀴
      analogWrite(motor_b_l_1, speed);
      analogWrite(motor_b_l_2, 
