import serial
import time

# 아두이노와의 시리얼 통신을 설정합니다.
arduino = serial.Serial('COM8', 9600)  # COM 포트 번호와 보레이트를 적절히 설정합니다.
time.sleep(2)  # 아두이노와의 연결을 위해 잠시 대기합니다.

while True:
    message = input("아두이노에게 보낼 문자열을 입력하세요: ")
    message = message.strip()  # 입력 문자열의 양쪽 공백과 개행 문자를 제거합니다.
    arduino.write(message.encode())  # 문자열을 아두이노로 보냅니다.
    time.sleep(0.1)  # 0.1초 대기합니다.
