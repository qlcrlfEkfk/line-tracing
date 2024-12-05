# #!/usr/bin/env python
# from pymycobot import myagv
# import time

# # AGV 초기화
# def initialize_agv(port="/dev/ttyAMA2", baud_rate=115200):
#     """
#     AGV 초기화
#     Args:
#         port (str): AGV 연결 포트
#         baud_rate (int): 통신 속도
#     Returns:
#         object: AGV 객체
#     """
#     agv = myagv.MyAgv(port, baud_rate)
#     # agv.connect()  # 연결 필요 시 활성화
#     return agv

# # AGV 동작 제어
# def move_control(agv, direction_1, direction_2, direction_3, duration):
#     """
#     AGV 동작 제어 함수
#     Args:
#         agv (object): AGV 객체
#         direction_1 (int): 전진/후진 제어 값
#         direction_2 (int): 좌/우 이동 제어 값
#         direction_3 (int): 회전 제어 값
#         duration (int): 동작 지속 시간 (초)
#     """
#     print(f"Moving AGV with directions: {direction_1}, {direction_2}, {direction_3}")
#     agv.move_control(direction_1, direction_2, direction_3)
#     time.sleep(duration)
#     agv.move_control(128, 128, 128)  # 정지
#     # time.sleep(1)


# # 메인 제어 루프
# if __name__ == "__main__":
#     try:
#         # AGV 초기화
#         agv = initialize_agv()
#         time.sleep(1)

#         # 1. 전진 (direction_1: 200 → 전진, direction_2, direction_3: 128 → 정지)
#         move_control(agv, direction_1=148, direction_2=128, direction_3=128, duration=4)
#         time.sleep(1)
#         move_control(agv, direction_1=128, direction_2=128, direction_3=170, duration=8) # 180도 회전 중요!!!
#         time.sleep(1)

#         move_control(agv, direction_1=148, direction_2=128, direction_3=128, duration=2)
#         time.sleep(1)

#         move_control(agv, direction_1=128, direction_2=128, direction_3=86, duration=8) 
#         time.sleep(1)

#         move_control(agv, direction_1=108, direction_2=128, direction_3=128, duration=2)
#         time.sleep(1)

#     except KeyboardInterrupt:
#         print("Stopping AGV...")
#     finally:
#         agv.move_control(128, 128, 128)  # AGV 정지
#         print("AGV disconnected.")
#!/usr/bin/env python
from pymycobot import myagv
import time

# AGV 초기화
def initialize_agv(port="/dev/ttyAMA2", baud_rate=115200):
    """
    AGV 초기화
    Args:
        port (str): AGV 연결 포트
        baud_rate (int): 통신 속도
    Returns:
        object: AGV 객체
    """
    agv = myagv.MyAgv(port, baud_rate)
    return agv

# 전진
def go_ahead(agv, speed, duration):
    """
    AGV 전진 동작
    Args:
        agv (object): AGV 객체
        speed (int): 전진 속도 (129~255)
        duration (int): 동작 지속 시간 (초)
    """
    print(f"AGV is moving forward at speed {speed}")
    agv.go_ahead(speed)  # pymycobot의 go_ahead 메서드 사용
    time.sleep(duration)
    # agv.stop()  # 정지 메서드로 이동 멈춤

# 후진
def retreat(agv, speed, duration):
    """
    AGV 후진 동작
    Args:
        agv (object): AGV 객체
        speed (int): 후진 속도 (0~127)
        duration (int): 동작 지속 시간 (초)
    """
    print(f"AGV is moving backward at speed {speed}")
    agv.retreat(speed)  # pymycobot의 retreat 메서드 사용
    time.sleep(duration)
    # agv.stop()  # 정지 메서드로 이동 멈춤

# 회전
def clockwise_rotate(agv, rotation_speed, duration):
    """
    AGV 회전 동작
    Args:
        agv (object): AGV 객체
        rotation_speed (int): 회전 속도 (0~127: 시계방향, 129~255: 반시계방향)
        duration (int): 동작 지속 시간 (초)
    """
    print(f"AGV is rotating at speed {rotation_speed}")
    agv.clockwise_rotation(rotation_speed)  # pymycobot의 rotate 메서드 사용
    time.sleep(duration)
    # agv.stop()  # 정지 메서드로 이동 멈춤

def counter_clockwise_rotate(agv, rotation_speed, duration):
    """
    AGV 회전 동작
    Args:
        agv (object): AGV 객체
        rotation_speed (int): 회전 속도 (0~127: 시계방향, 129~255: 반시계방향)
        duration (int): 동작 지속 시간 (초)
    """
    print(f"AGV is rotating at speed {rotation_speed}")
    agv.counterclockwise_rotation(rotation_speed)  # pymycobot의 rotate 메서드 사용
    time.sleep(duration)
    # agv.stop()  # 정지 메서드로 이동 멈춤

# 메인 제어 루프
if __name__ == "__main__":
    try:
        # AGV 초기화
        agv = initialize_agv()
        time.sleep(1)

        # 1. 전진
        go_ahead(agv, speed=15, duration=4)
        time.sleep(1)

        # 2. 180도 회전
        clockwise_rotate(agv, rotation_speed=45, duration=8) # 중요!!!!!!!!!!!!!!
        time.sleep(1)

        # 3. 전진
        go_ahead(agv, speed=5, duration=1)
        time.sleep(1)

        # 4. 회전
        clockwise_rotate(agv, rotation_speed=45, duration=8)
        time.sleep(1)

        # 5. 후진
        retreat(agv, speed=5, duration=2)
        time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping AGV...")
    finally:
        agv.stop()  # AGV 정지
        print("AGV disconnected.")
