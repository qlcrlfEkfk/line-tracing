#!/usr/bin/env python
import threading
import rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math
from select import select
import sys
if sys.platform != 'win32':
    import termios
    import tty

# 메시지 출력
msg = """
Reading from the keyboard and publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
v : toggle camera on/off
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    ',': (-1, 0, 0, 0),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (0, 0, 0, 1),
    'o': (0, 0, 0, -1),
    '.': (-1, 0, 0, -1),
    'm': (-1, 0, 0, 1)
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9)
}

# 픽셀-미터 비율 설정
ym_per_pix = 0.56 / 480
xm_per_pix = 0.37 / 640

class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("Failed to open camera.")
        self.frame = None
        self.running = True

    def run(self):
        while self.running:
            ret, self.frame = self.cap.read()
            if not ret:
                print("Failed to read from camera.")
                self.running = False

    def get_frame(self):
        return self.frame

    def stop(self):
        self.running = False
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

class CameraProcessingThread(threading.Thread):
    def __init__(self, camera_thread, pub_thread): 
        super(CameraProcessingThread, self).__init__()
        self.camera_thread = camera_thread
        self.pub_thread = pub_thread
        self.running = True

    def run(self):
        while self.running:
            frame = self.camera_thread.get_frame()
            if frame is not None:
                self.process_frame(frame)

    def stop(self):
        self.running = False
        cv2.destroyAllWindows()

    def process_frame(self, frame):
        filtered_image = self.color_filter(frame)
        roi_image = self.region_of_interest(filtered_image)
        warped_img, minverse = self.perspective_transform(roi_image)

        gray_img = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, binary_img = cv2.threshold(gray_img, 1, 255, cv2.THRESH_BINARY)
        left_base, right_base = self.calculate_histogram(binary_img)
        draw_info, out_img = self.sliding_window(binary_img, left_base, right_base)

        result, offset = self.calculate_offset_and_draw(frame, binary_img, minverse, draw_info)
        self.pub_thread.update_offset(offset)

        cv2.imshow("Processed Frame", result)
        cv2.waitKey(1)

    def color_filter(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        white_mask = cv2.inRange(hls, np.array([0, 230, 0]), np.array([255, 255, 255]))
        yellow_mask = cv2.inRange(hls, np.array([15, 30, 100]), np.array([35, 204, 255]))
        black_mask = cv2.inRange(hls, np.array([0,0,0]),np.array([180,255,50]))
        black_mask = cv2.bitwise_not(black_mask)
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        combined_mask = cv2.bitwise_and(combined_mask, black_mask)
        return cv2.bitwise_and(image, image, mask=combined_mask)

    def region_of_interest(self, image):
        height, width = image.shape[:2]
        mask = np.zeros_like(image)
        roi_corners = np.array([[(0, height), (width, height), (width, 0), (0, 0)]], dtype=np.int32)
        cv2.fillPoly(mask, roi_corners, (255,) * image.shape[2])
        return cv2.bitwise_and(image, mask)

    def perspective_transform(self, image):
        h, w = image.shape[:2]
        src = np.float32([[0, h], [w, h], [40, 140], [w - 40, 140]])
        dst = np.float32([[0, h], [w, h], [0, 0], [w, 0]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        inverse_matrix = cv2.getPerspectiveTransform(dst, src)
        return cv2.warpPerspective(image, matrix, (w, h)), inverse_matrix

    def calculate_histogram(self, binary_image):
        histogram = np.sum(binary_image[binary_image.shape[0] // 2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        return np.argmax(histogram[:midpoint]), np.argmax(histogram[midpoint:]) + midpoint

    def sliding_window(self, binary_image, left_base, right_base):
        nwindows = 4
        margin = 60
        minpix = 60
        window_height = binary_image.shape[0] // nwindows
        nonzero = binary_image.nonzero()
        nonzero_y, nonzero_x = np.array(nonzero[0]), np.array(nonzero[1])
        left_lane_inds, right_lane_inds = [], []

        for window in range(nwindows):
            win_y_low = binary_image.shape[0] - (window + 1) * window_height
            win_y_high = binary_image.shape[0] - window * window_height
            win_xleft_low = left_base - margin
            win_xleft_high = left_base + margin
            win_xright_low = right_base - margin
            win_xright_high = right_base + margin

            good_left_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                              (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                               (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                left_base = int(np.mean(nonzero_x[good_left_inds]))
            if len(good_right_inds) > minpix:
                right_base = int(np.mean(nonzero_x[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        left_fit = np.polyfit(nonzero_y[left_lane_inds], nonzero_x[left_lane_inds], 2)
        right_fit = np.polyfit(nonzero_y[right_lane_inds], nonzero_x[right_lane_inds], 2)
        ploty = np.linspace(0, binary_image.shape[0] - 1, binary_image.shape[0])

        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        return {'left_fitx': left_fitx, 'right_fitx': right_fitx, 'ploty': ploty}, None

    def calculate_offset_and_draw(self, original_image, binary_image, minverse, draw_info):
        mean_x = (draw_info['left_fitx'] + draw_info['right_fitx']) / 2
        center_x = binary_image.shape[1] // 2
        offset = (mean_x[-1] - center_x) * xm_per_pix
        return original_image, offset

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.offset = 0.0
        self.running = True
        self.start()

    def update_offset(self, offset):
        """라인 트레이싱에서 계산된 offset 값을 업데이트"""
        self.offset = offset

    def stop(self):
        """스레드 정지"""
        self.running = False

    def run(self):
        twist = Twist()
        while self.running:
            # AGV의 속도와 회전을 결정
            twist.linear.x = 0.2  # 직진 속도 (고정값 또는 동적으로 변경 가능)
            twist.angular.z = -0.1 * self.offset  # 오프셋에 따라 회전 속도 결정
            self.publisher.publish(twist)
            rospy.sleep(0.1)  # 0.1초 간격으로 명령 퍼블리시

def getKey(timeout):
    """키보드 입력 읽기 함수"""
    if sys.platform == 'win32':
        import msvcrt
        return msvcrt.getwch()
    else:
        import termios, tty
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key

def saveTerminalSettings():
    """터미널 설정 저장"""
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    """터미널 설정 복원"""
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.tcgetattr(sys.stdin))

def main():
    rospy.init_node('teleop_twist_keyboard')
    settings = saveTerminalSettings()
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.1)
    key_timeout = rospy.get_param("~key_timeout", 0.5)

    # 스레드 초기화
    pub_thread = PublishThread(repeat)
    camera_thread = CameraThread()
    processing_thread = CameraProcessingThread(camera_thread, pub_thread)

    camera_on = False  # 카메라 상태 (켜짐/꺼짐)
    x = y = z = th = 0

    try:
        pub_thread.update_offset(0)  # 초기 오프셋 설정
        print(msg)

        while not rospy.is_shutdown():
            key = getKey(key_timeout)

            if key == 'v':
                # 카메라 활성화/비활성화 토글
                camera_on = not camera_on
                if camera_on:
                    print("Starting camera...")
                    camera_thread.start()
                    processing_thread.start()
                else:
                    print("Stopping camera...")
                    camera_thread.stop()
                    processing_thread.stop()
            elif key in moveBindings:
                # 키 입력에 따라 이동 방향 변경
                x, y, z, th = moveBindings[key]
            elif key in speedBindings:
                # 속도 조정
                speed *= speedBindings[key][0]
                turn *= speedBindings[key][1]
            else:
                x = y = z = th = 0
                if key == '\x03':  # CTRL+C 종료
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    finally:
        # 스레드 정리
        camera_thread.stop()
        processing_thread.stop()
        pub_thread.stop()
        restoreTerminalSettings(settings)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

