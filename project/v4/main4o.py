import cv2
import numpy as np
import time
import serial

# 파이 시리얼 포트 설정 및 연결 대기
arduino = serial.Serial('COM8', 9600)

def py_serial(line_angle_input, width_input):
    # 음수 값이면 '-'를 붙여서 전송
    if float(line_angle_input) < 0:
        line_angle_input = "-" + line_angle_input
    if width_input is None or float(width_input) < 0:
        width_input = "0"
    # 숫자 두개를 하나의 문자열로 변환하여 통신(오버헤드 줄이려면 한 문자열로 하는게 좋음)
    message = f"{line_angle_input},{width_input}\n"
    arduino.write(message.encode())
    # 아두이노로부터 입력 받은 응답을 버퍼를 비우기 위해 읽어옴
    while arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print(response)

def process_frame(frame):
    # 이미지 전처리
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = np.array([0, 0, 180], dtype=np.uint8)
    upper_white = np.array([179, 30, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    # ROI 영역 설정
    height, width = mask.shape
    roi_vertices = np.array([[
        (0, height),
        (width // 2 - 15, height // 2 + 50),
        (width - 30, height // 2 + 50),
        (width - 30, height)
    ]], dtype=np.int32)
    roi_mask = np.zeros_like(mask)
    cv2.fillPoly(roi_mask, roi_vertices, 255)
    masked_image = cv2.bitwise_and(mask, roi_mask)
    # 허프 변환을 사용하여 직선 검출
    lines = cv2.HoughLinesP(masked_image, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=50)
    angle = 0  # 초기화
    intersection_x = 0  # 초기화
    if lines is not None:
        ideal_line = None
        rightmost_x = 0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            line_width = abs(x2 - x1)  # 선의 좌우 길이 계산
            minWidth = 50  # 좌우 길이의 최소값 설정
            if line_width >= minWidth:  # 좌우 길이가 최소값 이상인 선만 처리
                # 직선의 기울기 계산
                if x1 != x2 and y1 != y2 and (x2 - x1) != 0:
                    slope = (y2 - y1) / (x2 - x1)
                    # 평행 직선 무시
                    angle_threshold = 10  # 각도 기준 값
                    current_angle = np.degrees(np.arctan(slope))
                    if -angle_threshold < current_angle < angle_threshold:
                        continue
                    # 직선이 화면 하단과 만나는 지점의 x 좌표 계산
                    try:
                        current_intersection_x = int(x1 + (height - y1) * (x2 - x1) / (y2 - y1))
                    except ZeroDivisionError:
                        current_angle = 0
                        current_intersection_x = 0
                        continue
                    # 이상적인 직선 선택
                    if ideal_line is None or current_intersection_x > rightmost_x:
                        ideal_line = line
                        rightmost_x = current_intersection_x
                        angle = current_angle
                        intersection_x = current_intersection_x
        if ideal_line is not None:
            x1, y1, x2, y2 = ideal_line[0]  # ideal_line은 2차원 배열이므로 인덱스를 추가
            # 직선 표시
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 3)
            # 파이시리얼 통신 (현장에서 angle, distance_to_right_line 변수 확인 및 튜닝 필요)
            py_serial(str(angle), str(intersection_x))
            return frame, angle, intersection_x
    # lines가 None인 경우에는 기본 값을 반환
    return frame, 0, 0  # ideal_line이 None인 경우, 기본 값 반환
# 동영상 또는 카메라 입력을 사용하여 프레임을 읽어옴
video_capture = cv2.VideoCapture(0)
display_interval = 50  # 값을 출력하는 주기 (300 밀리초로 설정)
start = False
while True:
    ret, frame = video_capture.read()
    if not ret:
        break
    if start:
        processed_frame, angle, intersection_x = process_frame(frame)
        cv2.putText(processed_frame, f"Angle: {angle:.2f}, Intersection: ({intersection_x}, {frame.shape[0]})", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Lane Detection", processed_frame)
        # 프레임 처리 속도를 조절하기 위해 대기 시간 지정
        if cv2.waitKey(display_interval) & 0xFF == ord('q'):
            break
    else:
        start = input('input')
video_capture.release()
cv2.destroyAllWindows()
