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
const double consKp = 4.0;  // 보수적인 제어에 사용할 하이퍼 파라미터
const double consKi = 0.011;
const double consKd = 0.125;
const double aggKp = 10;  // 공격적인 제어에 사용할 하이퍼 파라미터
const double aggKi = 0.0075;
const double aggKd = 0.6;
const double chng_max = 25; // (튜닝 필요)

// 폭에 의한 제어값 변화량 (튜닝 필요)
double W_setpoint = 100.0;  // 이상적인 차폭
double W_pid_input, W_chng_output;     // 차선과의 폭 PID 제어에 사용할 변수
const double W_consKp = 0.05;  // 보수적인 제어에 사용할 하이퍼 파라미터: 멀리 있을 때
const double W_consKi = 0.01;
const double W_consKd = 0.1;
const double W_aggKp = 1;  // 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 때
const double W_aggKi = 0.01;
const double W_aggKd = 0.1;

// 폭에 의한 제어값 변화량 (튜닝 필요)
double A_setpoint = 0.0;  // 이상적인 차폭
double A_pid_input, A_chng_output;     // 차선과의 폭 PID 제어에 사용할 변수
const double A_consKp = 1.0;  // 보수적인 제어에 사용할 하이퍼 파라미터: 멀리 있을 때
const double A_consKi = 0.01;
const double A_consKd = 0.1;
const double A_aggKp = 1;  // 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 때
const double A_aggKi = 0.01;
const double A_aggKd = 0.1;

double CarLine_W, CarLine_A;
double chng_sum;
const double N_setpoint = 570;
const double CarLine_W_offset = 560;
const double CarLine_A_offset = 1.43;

PID pid(&pid_input, &pid_output, &setpoint, consKp, consKi, consKd, DIRECT);  // PID 객체 생성
PID W_pid(&W_pid_input, &W_chng_output, &W_setpoint, W_consKp, W_consKi, W_consKd, DIRECT);  // 차선과 폭 제어 PID 객체 생성
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
  W_pid.SetMode(AUTOMATIC);
  W_pid.SetOutputLimits(-chng_max, chng_max);  // 출력 제한 설정
  A_pid.SetMode(AUTOMATIC);
  A_pid.SetOutputLimits(-chng_max, chng_max);  // 출력 제한 설정
}

void loop() {
  if (Serial.available()) {
    String receivedString = Serial.readStringUntil('\n');
    int commaIndex = receivedString.indexOf(',');
    String line_angle_input_str = receivedString.substring(0, commaIndex);
    String width_input_str = receivedString.substring(commaIndex + 1);

    CarLine_A = line_angle_input_str.toFloat();
    CarLine_W = width_input_str.toFloat();

    // #### 폭 차에 의한 PID 제어
    W_pid_input = CarLine_W;
    if (abs(CarLine_W - CarLine_W_offset) < 8) {
      W_pid.SetTunings(W_consKp, W_consKi, W_consKd);
    } else {
      W_pid.SetTunings(W_aggKp, W_aggKi, W_aggKd);
    }
    W_pid.Compute();

    // #### 각도 차에 의한 PID 제어
    A_pid_input = CarLine_A;
    if (abs(CarLine_A - CarLine_A_offset) < 8) {
      A_pid.SetTunings(A_consKp, A_consKi, A_consKd);
    } else {
      A_pid.SetTunings(A_aggKp, A_aggKi, A_aggKd);
    }
    A_pid.Compute();

    // #### 핸들 PID 제어
    chng_sum = W_chng_output + A_chng_output;
    chng_sum = constrain(chng_sum, -chng_max, chng_max);
    pid_input = analogRead(potentiometerPin);
    setpoint = N_setpoint - chng_sum;

    double gap = abs(setpoint - pid_input);
    if (gap > 8) {
      pid.SetTunings(consKp, consKi, consKd);
    } else {
      pid.SetTunings(aggKp, aggKi, aggKd);
    }
    pid.Compute();

    // DC 모터 회전 제어
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
    if (abs(pid_output) >= 20){
      speed = speed_offset;
    } else {
      speed = speed_offset/3;
    }

    analogWrite(motor_b_l_1, speed); 
    analogWrite(motor_b_l_2, 0);
    analogWrite(motor_b_r_1, speed); 
    analogWrite(motor_b_r_2, 0);

    //##### 디버깅 정보 출력 #####//
    // Serial.println("potentiometerValue: " + String(potentiometerValue));
    // Serial.println("W_pid_input: " + String(W_pid_input));
    Serial.println("A_chng_output: " + String(A_chng_output));
    Serial.println("W_chng_output: " + String(W_chng_output));
    // Serial.println("chng_sum: " + String(chng_sum));
    // Serial.println("setpoint: " + String(setpoint));
    Serial.println("pid_output: " + String(pid_output));
    // Serial.println();
    delay(50);  // 딜레이
    //##### 디버깅 정보 출력 #####//

  }
}
