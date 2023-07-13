import serial
import time

arduino = serial.Serial('COM8', 9600)
time.sleep(2)

while True:
    line_angle_input = "0"
    width_input = "100"

    # 음수 값이면 '-'를 붙여서 전송
    if float(line_angle_input) < 0:
        line_angle_input = "-" + line_angle_input
    if float(width_input) < 0:
        width_input = "-" + width_input

    message = f"{line_angle_input},{width_input}\n"
    arduino.write(message.encode())
    time.sleep(0.05)

    # 아두이노로부터 입력 받은 응답을 버퍼를 비우기 위해 읽어옴
    while arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print(response)


    #time.sleep(2)

