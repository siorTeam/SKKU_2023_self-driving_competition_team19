//차선의 각도, 차선과 자동차의 폭을 받아 주행하는 코드입니다.

#include <PID_v1.h>
// double line_angle_input;
// double width_input;
double pid_input, pid_output;     // PID 제어에 사용할 변수

/////Pins/////
const int potentiometerPin = A8; 

int motor_f_1 = 2;
int motor_f_2 = 3;

int motor_b_l_1 = 4; 
int motor_b_l_2 = 5; 
int motor_b_r_1 = 7; 
int motor_b_r_2 = 6; 


/////Hyper Parameters/////
//angle
int angleMax = 70;
int angleMin = -70;
int angle_forWeakRot = 10;
int angle_forStrongRot = 40;

//width
int widthOffset = 10;
//int w_gap = input width - widthOffset
int w_gap_forWeakRot = 2;
int w_gap_forStrongRot = 5;

//potentiometer

//motor
int rotation_speed = 255/3;
int speed = 255;

//PID (pid 제어는 비트값으로)
double setpoint = 570;
double N_setpoint = 570; //정상상태 셋포인트
double A_setpoint_chng = 20; // 각도차에 의한 회전을 위한 셋 포인트
double W_setpoint_chng= 20;// 차선과의 간격에 의한 회전을 위한 셋 포인트

double cons_agg = 3; // conservative aggressive 전환점

double consKp = 2.0; //conservative control
double consKi = 0.011;
double consKd = 0.125;

double aggKp = 10; // aggressive control
double aggKi = 0.0075;
double aggKd = 0.6;


PID pid(&pid_input, &pid_output, &setpoint, consKp, consKi, consKd, DIRECT);  // PID 객체 생성

void setup() {
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
}

void loop() {
  int potentiometerValue = analogRead(potentiometerPin);  // 가변저항 값 읽기
  double front_wheel_angle = (double)((potentiometerValue - setpoint) * 270.0 / 1023.0);  // 0부터 1023을 0부터 270으로 비례적으로 변환
  Serial.print(front_wheel_angle);  // 가변저항에서 읽은 각도를 모니터로 값 출력
  Serial.print(" degrees\n");

  //직진
  analogWrite(motor_b_l_1, speed); 
  analogWrite(motor_b_l_2, 0);
  analogWrite(motor_b_r_1, speed); 
  analogWrite(motor_b_r_2, 0);

  // 컴퓨터로부터 시리얼 통신이 전송되면, 한 줄씩 읽어와서 cmd 변수에 입력
  if (Serial.available()) {
    String receivedString = Serial.readStringUntil('\n');
    int commaIndex = receivedString.indexOf(',');
    String line_angle_input_str = receivedString.substring(0, commaIndex);
    String width_input_str = receivedString.substring(commaIndex + 1);

    double line_angle_input = atof(line_angle_input_str.c_str());  // 문자열을 실수형으로 변환
    double width_input = atof(width_input_str.c_str());  // 문자열을 실수형으로 변환

    Serial.println(line_angle_input); // 첫 번째 부분 출력
    Serial.println(width_input);     // 두 번째 부분 출력
    // 차선에 의한 회전

    // int angleGap = front_wheel_angle - line_angle_input;  //+가 왼쪽으로 간 상황
    // if (angleGap >= 15) {
    //   analogWrite(motor_f_1, rotation_speed);
    //   analogWrite(motor_f_2, 0);
    // } else if (angleGap <= -15) {
    //   analogWrite(motor_f_1, 0);
    //   analogWrite(motor_f_2, rotation_speed);
    // } else {
    //   setpoint = N_setpoint;  // 원래 setpoint 값으로 복원
    //   analogWrite(motor_f_1, 0);
    //   analogWrite(motor_f_2, 0);    }
    int angleGap = front_wheel_angle - line_angle_input;  //+가 왼쪽으로 간 상황
    if (angleGap >= 15) {
      setpoint = N_setpoint - A_setpoint_chng;

    } else if (angleGap <= -15) {
      setpoint = N_setpoint + A_setpoint_chng;

    } else {
      setpoint = N_setpoint;  // 원래 setpoint 값으로 복원
    }
    int potentiometerValue = analogRead(potentiometerPin);  // 가변저항 값 읽기
    Serial.println(potentiometerValue);  // 시리얼 모니터로 값 출력
    
    pid_input = potentiometerValue;  // PID 입력값 설정

    double gap = abs(setpoint - pid_input);  // 목표값과의 오차 계산
    if (gap < 3) {
      // 보수적인 제어 파라미터 적용
      pid.SetTunings(consKp, consKi, consKd);
    } else {
      // 공격적인 제어 파라미터 적용
      pid.SetTunings(aggKp, aggKi, aggKd);
    }

    pid.Compute();  // PID 제어 계산

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
    // pid.Compute();

    // // PID 제어 결과를 이용하여 앞바퀴를 제어
    // if (pid_output > 0) {
    //   analogWrite(motor_f_1, pid_output);  // 양방향 회전
    //   analogWrite(motor_f_2, 0);
    // } else if (pid_output < 0) {
    //   analogWrite(motor_f_1, 0);
    //   analogWrite(motor_f_2, -pid_output);  // 역방향 회전
    // } else {
    //   analogWrite(motor_f_1, 0);  // 정지
    //   analogWrite(motor_f_2, 0);
    // }
    
    // output을 사용하여 원하는 작업을 수행


  }  
}
