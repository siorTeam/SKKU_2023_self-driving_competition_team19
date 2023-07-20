#include <PID_v1.h>

// Pin
const int potentiometerPin = A8;
const int motor_f_1 = 2;
const int motor_f_2 = 3;
const int motor_b_l_1 = 4;
const int motor_b_l_2 = 5;
const int motor_b_r_1 = 7;
const int motor_b_r_2 = 6;

const int NUM_SENSORS = 6;
const int echoPins[NUM_SENSORS] = {54, 55, 56, 57, 58, 59};
const int trigPins[NUM_SENSORS] = {23, 25, 27, 29, 31, 33};

// const int NUM_SENSORS = 6;
// const int echoPins[NUM_SENSORS] = {A0, A1, A2, A3, A4, A5};
// const int trigPins[NUM_SENSORS] = {23, 25, 27, 29, 31, 33};
// 속도
int Deceleration_threshold = 20;
int speed;

// PID
double setpoint;  
double pid_input, pid_output;
const double consKp = 4.0;  
const double consKi = 0.011;
const double consKd = 0.125;
const double aggKp = 10;  
const double aggKi = 0.0075;
const double aggKd = 0.6;
const double A_chng_max = 30;
const double chng_max = 30;

// 폭 제어
double W_setpoint = 500; 
double A_setpoint = 50;  
double A_pid_input, A_chng_output;
const double A_consKp = 2;
const double A_consKi = 0.01;
const double A_consKd = 0.1;
double chng_sum;
const double N_setpoint = 570;

// 신호등
long distance1;
long distance2;
int avoide_num = 0;

PID pid(&pid_input, &pid_output, &setpoint, consKp, consKi, consKd, DIRECT);
PID A_pid(&A_pid_input, &A_chng_output, &A_setpoint, A_consKp, A_consKi, A_consKd, DIRECT);

// 회피 구동 변수
bool avoiding = false; // 회피 구동 중인지 여부
unsigned long avoidStartTime1 = 0; // 회피 구동 시작 시간 기록
unsigned long avoidStartTime2 = 0; // 회피 구동 시작 시간 기록

void setup() {
  pinMode(motor_f_1, OUTPUT);
  pinMode(motor_f_2, OUTPUT);
  pinMode(motor_b_l_1, OUTPUT);
  pinMode(motor_b_l_2, OUTPUT);
  pinMode(motor_b_r_1, OUTPUT);
  pinMode(motor_b_r_2, OUTPUT);
  // 초음파 센서 설정
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  } 
  avoide_num = 1; // avoide_num 변수 초기화

  Serial.begin(9600);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  A_pid.SetMode(AUTOMATIC);
  A_pid.SetOutputLimits(-A_chng_max, A_chng_max);
}

void loop() {
  // ##### 파이시리얼 통신
  if (Serial.available()) {
    // String receivedString = Serial.readStringUntil('\n');
    // int firstCommaIndex = receivedString.indexOf(',');
    // int secondCommaIndex = receivedString.indexOf(',', firstCommaIndex + 1);
    // String line_angle_input_str = receivedString.substring(0, firstCommaIndex);
    // String width_input_str = receivedString.substring(firstCommaIndex + 1, secondCommaIndex);
    // String traffic_light_str = receivedString.substring(secondCommaIndex + 1);

    // A_pid_input = line_angle_input_str.toFloat();
    // double W_input = width_input_str.toFloat();
    // String traffic_light_color = traffic_light_str;

    // ##### 파이시리얼 통신 신호등 x
    String receivedString = Serial.readStringUntil('\n');
    int commaIndex = receivedString.indexOf(',');
    String line_angle_input_str = receivedString.substring(0, commaIndex);
    String width_input_str = receivedString.substring(commaIndex + 1);
    A_pid_input = line_angle_input_str.toFloat();
    double W_input = width_input_str.toFloat();

    String traffic_light_color = "N";

    // 거리 측정
    distance1 = getDistanceFromUltrasonicSensor(0);
    Serial.print("Distance1: ");
    Serial.println(distance1);

    distance2 = getDistanceFromUltrasonicSensor(1);
    Serial.print("Distance2: ");
    Serial.println(distance2);

    Serial.print("avoide_num: ");
    Serial.println(avoide_num);

    Serial.print("setpoint: ");
    Serial.println(setpoint);

      // 회피 구동중에는 아얘 다른 함수
    if (!avoiding) { // 회피중이 아닐 때
      if (distance1 < 150 and avoide_num == 1) { // 장애물 1 만남 -> 커맨드 1 생성, 기본 넘버는 2
        speed = 0;
        HandleBackMotor();
        delay(500);
        avoiding = true; //회피 시작
        avoidStartTime1 = millis(); //타이머 시작
      }
      if (distance1 < 90 and avoide_num == 2) { // 장애물 2 만남 -> 커맨드 2 생성
        if (distance2 > 40) {  // 장애물 1을 통과해야지만 작동(통과 안하면 무시)
          speed = 0;
          HandleBackMotor();
          delay(500);
          avoiding = true; //회피 시작
          avoidStartTime2 = millis(); //타이머 시작
        }
      }
    }

    // 회피 1번
    if (avoiding == true and avoide_num == 1) {
      unsigned long currentTime1 = millis();
      unsigned long avoidingTime1 = currentTime1 - avoidStartTime1;
      if (avoidingTime1 < 10000) {    // 10초 동안 회피 구동을 수행
        // 왼쪽으로 회피
        setpoint = (N_setpoint + chng_max); 
        HandleFrontMotor();
        speed = 50;
        HandleBackMotor();

        Serial.print("avoidingTime1: ");
        Serial.println(avoidingTime1);
      } else if (avoidingTime1 < 18000) { // 18초간 직진. 회피 구동 시간이 지나면, 다시 원래 주행 방향으로 복귀하는 동작 수행
        setpoint = CalculateSetpoint(W_input);
        HandleFrontMotor();
        SpeedControl();
        HandleBackMotor();
        Serial.print("avoidingTime1: ");
        Serial.println(avoidingTime1);
      } else {
        avoide_num += 1; // 어보이드 넘버 = 2
        avoiding = false; // 6초 지나고 나서 어보이딩 초기화, -> 다음 루프에서 커맨드 2 될 예정
      }        
    }

    // 회피 2번
    if (avoiding == true and avoide_num == 2) {
      unsigned long currentTime2 = millis();
      unsigned long avoidingTime2 = currentTime2 - avoidStartTime2;
      if (avoidingTime2 < 10000) {    // 10초 동안 회피 구동을 수행
        // 오른쪽으로 회피
        setpoint = (N_setpoint - chng_max); 
        HandleFrontMotor();
        speed = 50;
        HandleBackMotor();
        Serial.print("avoidingTime2: ");
        Serial.println(avoidingTime2);
      } else { // 회피 구동 시간이 지나면, 다시 원래 주행 방향으로 복귀
        avoide_num += 1; // 어보이드 넘버 = 3
        avoiding = false; // 5초 지나고 나서 어보이딩 초기화, -> 다음 루프에서 커맨드 2 될 예정
      }        
    }


    // 회피 구동 중이 아닐 때, 기본 주행 로직 수행
    if (!avoiding) {
      // 신호등 감지
      if (traffic_light_color == "R" && distance1 < 140) { //빨강 나오면 정지
        speed = 0;
        HandleBackMotor();
      } else if (traffic_light_color == "G" && distance1 < 140) { //초록 나오면 일반 주행
        setpoint = CalculateSetpoint(W_input);
        HandleFrontMotor();
        SpeedControl();
        HandleBackMotor();
      } else { //기본 주행
        setpoint = CalculateSetpoint(W_input);
        HandleFrontMotor();
        SpeedControl();
        HandleBackMotor();
        // PrintDebugInfo(W_input);
      }
    }
  }
}

long getDistanceFromUltrasonicSensor(int sensorIndex) {
  digitalWrite(trigPins[sensorIndex], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[sensorIndex], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[sensorIndex], LOW);

  long duration = pulseIn(echoPins[sensorIndex], HIGH);
  long distance = duration * 0.034 / 2;

  return distance;
}

double stop(int new_setpoint) { //정지 및 셋포인트 할당
  speed = 0;
  return new_setpoint;
}

double CalculateSetpoint(double W_input) {
  if (W_input < 300) {
    W_input = 0;
  }
  if (A_pid_input < 25) {
    A_pid_input = 50;
  }

  if (!W_input || !A_pid_input) {
    return N_setpoint;
  } else if (W_input >= 1000) {
    return N_setpoint;
  } else if (distance2 < 15) { //우측 장애물 감지 코드
    return N_setpoint + chng_max;
  }else if (W_input - W_setpoint < -30 && W_input != 0) {
    return N_setpoint + chng_max;
  } else if (W_input - W_setpoint > 30 && W_input != 0) {
    return N_setpoint - chng_max;
  } else {
    A_pid.SetTunings(A_consKp, A_consKi, A_consKd);
    A_pid.Compute();
    return N_setpoint + A_chng_output;
  }
}

void HandleFrontMotor() {
  pid_input = analogRead(potentiometerPin);
  double gap = abs(setpoint - pid_input);
  if (gap > 8) {
    pid.SetTunings(consKp, consKi, consKd);
  } else {
    pid.SetTunings(aggKp, aggKi, aggKd);
  }
  pid.Compute();

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
void SpeedControl() {
if (abs(A_pid_input - A_setpoint) >=(A_chng_max - 2) || A_pid_input == 0){
    speed = 50;
  } else {
    speed = 65;
  }
}
void HandleBackMotor() {
   
  analogWrite(motor_b_l_1, speed);
  analogWrite(motor_b_l_2, 0);
  analogWrite(motor_b_r_1, speed);
  analogWrite(motor_b_r_2, 0);
}

// Optional: 디버깅 정보 출력 함수
// void PrintDebugInfo(double W_input) {
//   Serial.println("W_input: " + String(W_input));
//   Serial.println("setpoint: " + String(setpoint));
// }
