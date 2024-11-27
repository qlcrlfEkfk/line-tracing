import cv2
import socket
import struct
import pickle

# 소켓 초기화
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 8485))  # IP, Port 설정
server_socket.listen(5)
print("연결 대기 중...")

# 클라이언트 연결 대기
conn, addr = server_socket.accept()
print(f"클라이언트 연결: {addr}")

# 카메라 초기화
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 프레임 직렬화
    data = pickle.dumps(frame)
    size = len(data)

    # 소켓으로 데이터 전송 (길이 + 데이터)
    conn.sendall(struct.pack(">L", size) + data)

cap.release()
conn.close()
server_socket.close()