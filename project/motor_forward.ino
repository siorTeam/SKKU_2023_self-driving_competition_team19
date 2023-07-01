// 모터 핀
int motor_f_1 = 2;
int motor_f_2 = 3;
int motor_b_l_1 = 4; 
int motor_b_l_2 = 5; 
int motor_b_r_1 = 7; 
int motor_b_r_2 = 6; 

int serial_vuff; // 전역 변수
int speed = 255/2;
int rotation_speed = 255/4;

void setup() {
  // 모터 핀 셋업 실제 핀 연결(OUTPUT 1, 2)과 상이하니 주의
  //1이 정방향(앞으로 가는거, 시계 반대방향)
  pinMode(motor_f_1, OUTPUT); 
  pinMode(motor_f_2, OUTPUT); 
  pinMode(motor_b_l_1, OUTPUT); 
  pinMode(motor_b_l_2, OUTPUT); 
  pinMode(motor_b_r_1, OUTPUT); 
  pinMode(motor_b_r_2, OUTPUT); 
}

void loop() {
  analogWrite(motor_f_1, 0); 
  analogWrite(motor_f_1, 0); 

  analogWrite(motor_b_l_1, speed); 
  analogWrite(motor_b_l_2, 0);

  analogWrite(motor_b_r_1, speed); 
  analogWrite(motor_b_r_2, 0);
}
