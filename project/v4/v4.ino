//최적화 버전
#include <PID_v1.h>
// Pin
const int potentiometerPin = A8;  // 가변저항 연결 핀
const int motor_f_1 = 2;
const int motor_f_2 = 3;
int motor_b_l_1 = 4;
int motor_b_l_2 = 5;
int motor_b_r_1 = 7;
int motor_b_r_2 = 6;
//속도
int Deceleration_threshold = 20;
int speed;
int speed_offset = 255;
// PID
// 앞바퀴
double setpoint;  // 목표값(튜닝 완료)
double pid_input, pid_output;     // PID 제어에 사용할 변수
const double consKp = 4.0;  // 보수적인 제어에 사용할 하이퍼 파라미터닝
const double consKi = 0.011;
const double consKd = 0.125;
const double aggKp = 10;  // 공격적인 제어에 사용할 하이퍼 파라미터
const double aggKi = 0.0075;
const double aggKd = 0.6;
const double A_chng_max = 30; // (튜닝 필요) (30+40:40은 오류 발생 이슈)
const double chng_max = 30;
// 폭 제어
double W_setpoint = 520; // 이상적인 차폭 (offset)
// 각도에 의한 제어값 변화량 (튜닝 필요)
double A_setpoint = 60;  // 이상적인 차선 각도 (offset)
double A_pid_input, A_chng_output;     // 차선과의 폭 PID 제어에 사용할 변수
const double A_consKp = 2;  // 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 때
const double A_consKi = 0.01;
const double A_consKd = 0.1;
double chng_sum;
const double N_setpoint = 570;
PID pid(&pid_input, &pid_output, &setpoint, consKp, consKi, consKd, DIRECT);  // PID 객체 생성
PID A_pid(&A_pid_input, &A_chng_output, &A_setpoint, A_consKp, A_consKi, A_consKd, DIRECT);  // 차선과 각도 PID 객체 생성
void setup() {
  // 모터 설정
  pinMode(motor_f_1, OUTPUT);
  pinMode(motor_f_2, OUTPUT);
  pinMode(motor_b_l_1, OUTPUT);
  pinMode(motor_b_l_2, OUTPUT);
  pinMode(motor_b_r_1, OUTPUT);
  pinMode(motor_b_r_2, OUTPUT);
  Serial.begin(9600);
  // PID 제어 설정
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  // 출력 제한 설정
  A_pid.SetMode(AUTOMATIC);
  A_pid.SetOutputLimits(-A_chng_max, A_chng_max);  // 출력 제한 설정
}
void loop() {
  // ##### 파이시리얼 통신
  if (Serial.available()) {
    String receivedString = Serial.readStringUntil('\n');
    int commaIndex = receivedString.indexOf(',');
    String line_angle_input_str = receivedString.substring(0, commaIndex);
    String width_input_str = receivedString.substring(commaIndex + 1);
    A_pid_input = line_angle_input_str.toFloat();
    double W_input = width_input_str.toFloat();
    // ##### 셋포인트 제어
    // ### error cases
    if (not W_input and not A_pid_input){     //제어 인풋 없을 경우 직진
      setpoint = N_setpoint;
    } else if (W_input >= 3000){    // C 구간 극복 (값이 이상하게 받아질 경우 직진)
      setpoint = N_setpoint;
    }
    // ### 폭에 의한 핸들   (임계값 넘어갈 경우 제어)
    else if (W_input - W_setpoint < -35){ //offset 오른쪽으로 갔을 경우 왼쪽으로
      setpoint = N_setpoint + chng_max;
    } else if (W_input - W_setpoint > 35){ //offset 왼쪽으로 갔을 경우 오른쪽으로
      setpoint = N_setpoint - chng_max;
    }
    // ### 각도 차에 의한 PID 제어
    else {
      A_pid.SetTunings(A_consKp, A_consKi, A_consKd);
      A_pid.Compute();
      setpoint = N_setpoint + A_chng_output;
    }
    // ##### 핸들 PID 제어
    pid_input = analogRead(potentiometerPin);
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
    // ##### 뒷바퀴 제어
    //각도 급회전시 감속
    if (abs(A_pid_input - A_setpoint) >= A_chng_max - 4 || A_pid_input == 0){
      speed = speed_offset/4.3;
    } else {
      speed = 135;
    }
    analogWrite(motor_b_l_1, speed);
    analogWrite(motor_b_l_2, 0);
    analogWrite(motor_b_r_1, speed);
    analogWrite(motor_b_r_2, 0);
    //##### 디버깅 정보 출력 #####//
    // +면 좌회전
    // Serial.println("A_pid_input: " + String(A_pid_input));
    // Serial.println("A_chng_output: " + String(A_chng_output));
    // Serial.println("chng_sum: " + String(chng_sum));
    // Serial.println("setpoint: " + String(setpoint));
    // Serial.println("pid_output: " + String(pid_output));
    // Serial.println();
    // delay(50);  // 딜레이
    //##### 디버깅 정보 출력 #####//
  }
}
