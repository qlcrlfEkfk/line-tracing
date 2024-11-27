import cv2
import socket
import struct
import pickle
import numpy as np
# 카메라 초기화
cap = cv2.VideoCapture(0)
# 소켓 초기화 (다른 PC에서 데이터 수신)
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('172.30.1.35', 8485))  # 송신 측의 IP와 Port 사용
payload_size = struct.calcsize(">L")
data = b""
while True:
    # 1. 현재 PC 카메라에서 프레임 읽기
    ret, frame_local = cap.read()
    if not ret:
        break
    # 2. 소켓으로부터 중앙 영역용 데이터 수신
    while len(data) < payload_size:
        data += client_socket.recv(4096)
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    while len(data) < msg_size:
        data += client_socket.recv(4096)
    frame_remote_data = data[:msg_size]
    data = data[msg_size:]
    frame_remote = pickle.loads(frame_remote_data)
    # 3. 중앙 영역에 다른 PC의 영상 삽입
    h, w, _ = frame_local.shape
    center_h, center_w = h // 2, w // 2
    remote_resized = cv2.resize(frame_remote, (w // 3, h // 3))  # 크기 조정
    remote_h, remote_w, _ = remote_resized.shape
    # 중앙 영역 설정
    start_y = center_h - remote_h // 2
    start_x = center_w - remote_w // 2
    frame_local[start_y:start_y+remote_h, start_x:start_x+remote_w] = remote_resized
    # 4. 결과 디스플레이
    cv2.imshow("Combined Feed", frame_local)
    # 'q' 키로 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
client_socket.close()
cv2.destroyAllWindows()







