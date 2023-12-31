#include <PID_v1.h>

//pin
const int potentiometerPin = A8;  // 가변저항 연결 핀
int motor_f_1 = 2;
int motor_f_2 = 3;

//pid
//앞바퀴
double setpoint;  // 목표값(튜닝 완료)
double pid_input, pid_output;     // PID 제어에 사용할 변수
double consKp = 4.0;// 보수적인 제어에 사용할 하이퍼 파라미터
double consKi = 0.011;
double consKd = 0.125;
double aggKp = 10;// 공격적인 제어에 사용할 하이퍼 파라미터
double aggKi = 0.0075;
double aggKd = 0.6;
double chng_max = 20; // (튜닝 필요)

//폭에 의한 제어값 변화량 (튜닝 필요)
double W_setpoint = 100.0;  // 이상적인 차폭
double W_pid_input, W_chng_output;     // 차선과의 폭 PID 제어에 사용할 변수
double W_consKp = 1;// 보수적인 제어에 사용할 하이퍼 파라미터: 멀리 있을 때
double W_consKi = 0.01;
double W_consKd = 0.1;
double W_aggKp = 1;// 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 때
double W_aggKi = 0.01;
double W_aggKd = 0.1;
//폭에 의한 제어값 변화량 (튜닝 필요)
double A_setpoint = 0.0;  // 이상적인 차폭
double A_pid_input, A_chng_output;     // 차선과의 폭 PID 제어에 사용할 변수
double A_consKp = 1.0;// 보수적인 제어에 사용할 하이퍼 파라미터: 멀리 있을 때
double A_consKi = 0.01;
double A_consKd = 0.1;
double A_aggKp = 1;// 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 때
double A_aggKi = 0.01;
double A_aggKd = 0.1;

double CarLine_W, CarLine_A;
double chng_sum;
double N_setpoint = 570;
double CarLine_W_offset = 100;
double CarLine_A_offset = 0;


PID pid(&pid_input, &pid_output, &setpoint, consKp, consKi, consKd, DIRECT);  // PID 객체 생성
PID W_pid(&W_pid_input, &W_chng_output, &W_setpoint, W_consKp, W_consKi, W_consKd, DIRECT);  // 차선과 폭 제어 PID 객체 생성
PID A_pid(&A_pid_input, &A_chng_output, &A_setpoint, A_consKp, A_consKi, A_consKd, DIRECT);  // 차선과 각도 PID 객체 생성


void setup() {
  // 모터 설정
  pinMode(motor_f_1, OUTPUT);
  pinMode(motor_f_2, OUTPUT);
  Serial.begin(9600);
  
  // PID 제어 설정
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);  // 출력 제한 설정
  W_pid.SetMode(AUTOMATIC);
  W_pid.SetOutputLimits(-20, 20);  // 출력 제한 설정
  A_pid.SetMode(AUTOMATIC);
  A_pid.SetOutputLimits(-20, 20);  // 출력 제한 설정
}

void loop() {
  // 뒤로 감 int potentiometerValue = analogRead(potentiometerPin);  // 가변저항 값 읽기
  if (Serial.available()) {
    String receivedString = Serial.readStringUntil('\n');
    int commaIndex = receivedString.indexOf(',');
    String line_angle_input_str = receivedString.substring(0, commaIndex);
    String width_input_str = receivedString.substring(commaIndex + 1);

    CarLine_A = atof(line_angle_input_str.c_str());  // 문자열을 실수형으로 변환
    CarLine_W = atof(width_input_str.c_str());  // 문자열을 실수형으로 변환

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
    if (chng_sum > chng_max){
      chng_sum = chng_max;
    }
    else if (chng_sum < -(chng_max)){
      chng_sum = -(chng_max);
    }
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
  }
}
