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
double W_consKp = 1;// 보수적인 제어에 사용할 하이퍼 파라미터: 멀리 있을 떄
double W_consKi = 0.01;
double W_consKd = 0.1;
double W_aggKp = 1;// 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 떄
double W_aggKi = 0.01;
double W_aggKd = 0.1;
//폭에 의한 제어값 변화량 (튜닝 필요)
double A_setpoint = 0.0;  // 이상적인 차폭
double A_pid_input, A_chng_output;     // 차선과의 폭 PID 제어에 사용할 변수
double A_consKp = 1.0;// 보수적인 제어에 사용할 하이퍼 파라미터: 멀리 있을 떄
double A_consKi = 0.01;
double A_consKd = 0.1;
double A_aggKp = 1;// 공격적인 제어에 사용할 하이퍼 파라미터: 가까이 있을 떄
double A_aggKi = 0.01;
double A_aggKd = 0.1;

//double CarLine_W, CarLine_A;
double CarLine_W = 150; // 테스트 시 
double CarLine_A = 1; // 테스트 시 
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
  int potentiometerValue = analogRead(potentiometerPin);  // 가변저항 값 읽기
  //debug
  // Serial.print("potentiometerValue: ");  // 시리얼 모니터로 값 출력
  // Serial.println(potentiometerValue);  // 시리얼 모니터로 값 출력

  // #### 폭 차에 의한 PID 제어
  W_pid_input = CarLine_W; // +가 왼쪽으로 간 상황(우회전 필요)
  if (abs(CarLine_W - CarLine_W_offset) < 8) {  // 폭 목표값과의 오차 계산 (튜닝 필요)
    W_pid.SetTunings(W_consKp, W_consKi, W_consKd); // 폭이 offset과 가까우면 보수적으로
  } else { // 각이 offset과 멀면 보수적으로
    W_pid.SetTunings(W_aggKp, W_aggKi, W_aggKd);
  }
  W_pid.Compute();  // PID 제어 계산 -> W_chng_output +가 왼쪽으로 간 상황(우회전 필요)

  //debug
  // Serial.print("W_pid_input: ");  // 시리얼 모니터로 값 출력
  // Serial.println(W_pid_input);  // 시리얼 모니터로 값 출력
  // Serial.print("W_chng_output: ");  // 시리얼 모니터로 값 출력
  // Serial.println(W_chng_output);  // 시리얼 모니터로 값 출력


  // #### 각도 차에 의한 PID 제어
  A_pid_input = CarLine_A; // +가 왼쪽으로 간 상황(우회전 필요)
  // 폭 목표값과의 오차 계산 (튜닝 필요)
  if (abs(CarLine_A - CarLine_A_offset) < 8) { // 각이 offset과 가까우면 보수적으로
    A_pid.SetTunings(A_consKp, A_consKi, A_consKd);
  } else { // 각이 offset과 멀면 보수적으로
    A_pid.SetTunings(A_aggKp, A_aggKi, A_aggKd);
  }
  A_pid.Compute();  // PID 제어 계산 -> A_chng_output

  //debug
  Serial.print("A_chng_output: ");  // 시리얼 모니터로 값 출력
  Serial.println(A_chng_output);  // 시리얼 모니터로 값 출력


  // #### 핸들 PID 제어
  chng_sum = W_chng_output + A_chng_output;
  if (chng_sum > chng_max){
    chng_sum = chng_max;
  }
  else if (chng_sum < -(chng_max)){
    chng_sum = -(chng_max); // +가 왼쪽으로 간 상황(우회전:'-' 필요)
  }
  pid_input = potentiometerValue; // 가변저항 값: 핸들 위치가 인풋
  setpoint = N_setpoint - chng_sum; // 셋 포인트: 기본 셋포인트(N_setpoint)에서 변화시킬 값(chng_sum)을 마이너스(좌로 틀어지면 -> 우로 틀어지도록)

  // 핸들 목표값과의 오차 계산 (튜닝 완료)
  double gap = abs(setpoint - pid_input);  
  if (gap > 8) {
    // 보수적인 제어 파라미터 적용: 멀리 있을 떄 공격적이면 모터 출력이 너무 커짐 -> 진동 일어남. 출력값이 너무 처지지 않도록 보수적인 제어 필요.
    pid.SetTunings(consKp, consKi, consKd);
  } else {
    // 공격적인 제어 파라미터 적용: 가까울때 공격적이어야 모터 출력이 충분히 나옴
    pid.SetTunings(aggKp, aggKi, aggKd);
  }
  pid.Compute();  // PID 제어 계산 -> pid_output (핸들 돌릴 모터의 값이 나옴)
  //debug
  // Serial.print("chng_sum: ");  // 시리얼 모니터로 값 출력
  // Serial.println(chng_sum);  // 시리얼 모니터로 값 출력
  // Serial.print("setpoint: ");  // 시리얼 모니터로 값 출력
  // Serial.println(setpoint);  // 시리얼 모니터로 값 출력
  // Serial.print("pid_output: ");  // 시리얼 모니터로 값 출력
  // Serial.println(pid_output);  // 시리얼 모니터로 값 출력








  // DC 모터 회전 제어
  if (pid_output > 0) {
    analogWrite(motor_f_1, pid_output);  // 양방향 회전
    analogWrite(motor_f_2, 0);
  } else if (pid_output < 0) {
    analogWrite(motor_f_1, 0);
    analogWrite(motor_f_2, -pid_output);  // 역방향 회전
  } else {
    analogWrite(motor_f_1, 0);  // 정지
    analogWrite(motor_f_2, 0);
  }
  Serial.print("pid_output: ");  // 시리얼 모니터로 값 출력
  Serial.println(pid_output);  // 시리얼 모니터로 값 출력
  Serial.println();  // 시리얼 모니터로 값 출력
  delay(5 000);
}
