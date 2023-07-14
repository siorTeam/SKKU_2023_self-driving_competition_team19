import cv2
import numpy as np
from scipy.stats import linregress
import serial
import time

# 웹캠 해상도 설정
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 파이 시리얼 포트 설정 및 연결 대기
arduino = serial.Serial('COM8', 9600)
time.sleep(2)

#파이 시리얼 통신 코드
def py_serial(line_angle_input, width_input):

    # 음수 값이면 '-'를 붙여서 전송
    if float(line_angle_input) < 0:
        line_angle_input = "-" + line_angle_input
    if float(width_input) < 0:
        width_input = "-" + width_input
    # 숫자 두개를 하나의 문자열로 변환하여 통신(오버헤드 줄이려면 한 문자열로 하는게 좋음)
    message = f"{line_angle_input},{width_input}\n"
    arduino.write(message.encode())
    time.sleep(0.05)

    # 아두이노로부터 입력 받은 응답을 버퍼를 비우기 위해 읽어옴
    while arduino.in_waiting > 0:
        response = arduino.readline().decode().strip()
        print(response)


def py_serial(line_angle_input, width_input):

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


def undistort_frame(frame):
    # 카메라 캘리브레이션 매개 변수
    camera_matrix = np.array([[1.0, 0, FRAME_WIDTH / 2],
                              [0, 1.0, FRAME_HEIGHT / 2],
                              [0, 0, 1.0]], dtype=np.float32)
    distortion_coefficients = np.zeros((4, 1), dtype=np.float32)

    # 왜곡 보정
    undistorted_frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)
    return undistorted_frame

def detect_dotted_line():
    # 웹캠 열기
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    # 웹캠 영상 읽기
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 영상 보정
        frame = undistort_frame(frame)

        # 영상 크기
        height, width = frame.shape[:2]

        # 원의 중심 좌표와 반지름 설정
        center_x = int(width / 2)
        center_y = int(height / 2)
        radius = int(min(center_x, center_y) * 0.6)

        # ROI (원 내부 영역) 생성
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, (center_x, center_y), radius, (255, 255, 255), -1)
        roi = cv2.bitwise_and(frame, frame, mask=mask)

        # 이미지를 그레이스케일로 변환
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # 이미지 이진화
        _, binary = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)

        # 추세선을 위한 점 리스트 초기화
        dotted_points = []
        solid_points = []
        solid_rect_lines = []  # 실선의 긴 변을 저장하는 리스트

        # 점선과 실선 검출을 위한 컨투어 찾기
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 점선과 실선 그리기
        for contour in contours:
            area = cv2.contourArea(contour)
            if 300 < area < 1000:  # 점선으로 인식할 면적 조건 설정
                # 직사각형의 꼭지점 좌표 구하기
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.intp(box)

                # 점선 그리기
                cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)

                # 중심점 좌표 수집
                cx = int(rect[0][0])
                cy = int(rect[0][1])
                dotted_points.append((cx, cy))
            elif area >= 1000:  # 실선으로 인식할 면적 조건 설정
                # 직사각형의 꼭지점 좌표 구하기
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.intp(box)

                # 실선 그리기
                cv2.drawContours(frame, [box], 0, (255, 255, 0), 2)

                # 중심점 좌표 수집
                cx = int(rect[0][0])
                cy = int(rect[0][1])
                solid_points.append((cx, cy))

                # 가장 긴 변의 직선 그리기
                rect_lines = cv2.boxPoints(rect)
                rect_lines = np.intp(rect_lines)
                longest_line = np.argmax(np.linalg.norm(rect_lines[:-1] - rect_lines[1:], axis=1))

                # 가장 긴 변의 두 점
                p1 = rect_lines[longest_line]
                p2 = rect_lines[(longest_line + 1) % 4]

                # 직선의 방정식 계산
                dx = p2[0] - p1[0]
                dy = p2[1] - p1[1]
                if dx != 0:
                    slope = dy / dx
                else:
                    slope = np.inf

                # 직선을 화면 밖까지 그리기
                x1 = p1[0] - int((p1[1] - 0) / slope) if slope != 0 else p1[0]
                y1 = 0
                x2 = p2[0] + int((height - p2[1]) / slope) if slope != 0 else p2[0]
                y2 = height - 1

                cv2.line(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                solid_rect_lines.append(rect_lines[longest_line])

        # 점선 추세선 그리기
        if len(dotted_points) > 1:
            xs, ys = zip(*dotted_points)
            unique_xs = np.unique(xs)
            if len(unique_xs) == 1:
                continue
            slope, intercept, _, _, _ = linregress(xs, ys)
            if np.isnan(slope) or np.isnan(intercept):
                continue

            # 점선 추세선 그리기
            x1 = 0
            x2 = width
            y1 = int(slope * x1 + intercept)
            y2 = int(slope * x2 + intercept)

            cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # 가장 긴 변의 직선과 점선 추세선의 교점 계산
            dx = x2 - x1
            dy = y2 - y1
            if dx != 0:
                line_slope = dy / dx
                line_intercept = y1 - line_slope * x1
                if line_slope != slope:
                    intersection_x = (intercept - line_intercept) / (line_slope - slope)
                    intersection_y = slope * intersection_x + intercept

                    # 화면에 교점 표시
                    if np.isfinite(intersection_x) and np.isfinite(intersection_y) and 0 <= intersection_x < width:
                        cv2.circle(frame, (int(intersection_x), int(intersection_y)), 5, (0, 255, 255), -1)

                        # 교점의 x 좌표에 따라 동작 결정 (아예 여유를 안줬는데 straight로 갔으면 하는 정도의 여유줘야함)
                        if intersection_x > center_x:
                            cv2.putText(frame, 'Turn right', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        elif intersection_x < center_x:
                            cv2.putText(frame, 'Turn left', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        else:
                            cv2.putText(frame, 'Go straight', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    print("No intersection")
            else:
                print("No intersection")

        # 원의 영역 표시
        cv2.circle(frame, (center_x, center_y), radius, (255, 0, 0), 2)

        # Calculate the angle between the dotted line and the horizontal axis
        angle = None
        if len(dotted_points) > 1:
            xs, ys = zip(*dotted_points)
            unique_xs = np.unique(xs)
            if len(unique_xs) == 1:
                continue
            slope, intercept, _, _, _ = linregress(xs, ys)
            if np.isnan(slope) or np.isnan(intercept):
                continue

            x1 = 0
            x2 = width
            y1 = int(slope * x1 + intercept)
            y2 = int(slope * x2 + intercept)

            vec1 = np.array([1, 0])  # Vector along the horizontal axis
            vec2 = np.array([x2 - x1, y2 - y1])  # Vector along the dotted line
            vec2_normalized = vec2 / np.linalg.norm(vec2)  # Normalize the dotted line vector
            angle = np.degrees(np.arccos(np.dot(vec1, vec2_normalized)))

        # Calculate the midpoint of the right solid line
        midpoint_x = None
        distance_to_right_line = None
        if len(solid_points) > 0:
            p1 = rect_lines[longest_line]
            p2 = rect_lines[(longest_line + 1) % 4]
            midpoint_x = int((p1[0] + p2[0]) / 2)

            # 차의 위치 좌표 (예시로 가운데 좌표 사용)
            cx = int(width / 2)
            cy = int(height / 2)

            # 오른쪽 실선과 차의 거리 계산
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            if dx != 0:
                line_slope = dy / dx
                line_intercept = p1[1] - line_slope * p1[0]
                distance_to_right_line = abs(line_slope * cx - cy + line_intercept) / np.sqrt(line_slope ** 2 + 1)

        # Display the angle, midpoint x-coordinate, and distance to the right line on the frame
        if angle is not None:
            cv2.putText(frame, f"Angle: {angle:.2f} degrees", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if midpoint_x is not None:
            cv2.putText(frame, f"Midpoint X: {midpoint_x}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        if distance_to_right_line is not None:
            cv2.putText(frame, f"Distance to Right Line: {distance_to_right_line:.2f} units", (10, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 파이시리얼 통신 (현장에서 angle, distance_to_right_line 변수 확인 및 튜닝 필요)
        py_serial(angle, distance_to_right_line)
        
        # 결과 프레임 출력
        cv2.imshow('Detected Lines', frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 파이 시리얼 통신


    # 웹캠 종료
    cap.release()
    cv2.destroyAllWindows()

# 웹캠으로 특정 색상 점선 및 흰색 실선 검출 및 추세선, 가장 긴 변 표시 함수 호출
detect_dotted_line()
