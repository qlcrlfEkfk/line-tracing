#!/usr/bin/env python
from __future__ import print_function
import threading
import rospy
from geometry_msgs.msg import Twist
import time
import sys
from select import select
import cv2
import numpy as np
import math
if sys.platform == 'win32':
    import msvcrt
else:
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
    'o': (0, 0, 0, -1),
    'j': (0, 1, 0, 0),
    'l': (0, -1, 0, 0),
    'u': (0, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, -1),
    'm': (-1, 0, 0, 1)
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9)
}

# 픽셀당 실제 길이 비율 설정
ym_per_pix = 0.56 / 480  # 세로 픽셀당 미터
xm_per_pix = 0.37 / 640  # 가로 픽셀당 미터
previous_turn = 0.5  # 초기 회전 값
previous_speed = 0.25  # 초기 속도 값

# 카메라 캡처 스레드
class CameraThread(threading.Thread):
    def __init__(self):
        super(CameraThread, self).__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("카메라를 열 수 없습니다.")
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

# 영상 처리 스레드
class CameraProcessingThread(threading.Thread):
    def __init__(self, camera_thread, pub_thread): 
        super(CameraProcessingThread, self).__init__()
        self.camera_thread = camera_thread
        self.pub_thread = pub_thread  # pub_thread 저장
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
        if frame is None:
            print("Received None frame. Skipping processing.")
            return
        filtered_image = self.color_filter(frame)
        roi_image = self.roi(filtered_image)
        warped_img, minverse = self.wrapping(roi_image)

        _gray = cv2.cvtColor(warped_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(_gray, 1, 255, cv2.THRESH_BINARY)
        leftbase, rightbase = self.plothistogram(thresh)
        draw_info, out_img = self.slide_window_search(thresh, leftbase, rightbase)

        result, offset = self.draw_lane_lines_with_offset(frame, thresh, minverse, draw_info)
        
        # 전달된 offset을 사용하여 pub_thread 업데이트
        self.pub_thread.update_offset(offset)
        
        cv2.imshow("(Unwarped) Sliding Window Search", out_img)
        cv2.imshow("Result", result)
        cv2.waitKey(1)

    # 필요한 영상 처리 함수들 추가
    def color_filter(self, image):
        hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
        
        # 흰색 범위 설정
        lower_white = np.array([0, 230, 0])
        upper_white = np.array([255, 255, 255])
        white_mask = cv2.inRange(hls, lower_white, upper_white)
        
        # 노란색 범위 설정
        yellow_lower = np.array([15, 30, 100])
        yellow_upper = np.array([35, 204, 255])
        yellow_mask = cv2.inRange(hls, yellow_lower, yellow_upper)
        
        # 검은색 제외
        black_lower = np.array([0, 0, 0])
        black_upper = np.array([180, 255, 50])
        black_mask = cv2.inRange(hls, black_lower, black_upper)
        black_mask = cv2.bitwise_not(black_mask)  # 검은색 영역을 제외하기 위해 반전
        
        # 흰색과 노란색 마스크 결합
        combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
        final_mask = cv2.bitwise_and(combined_mask, black_mask)  # 검은색 제외
        
        # 최종 필터 적용
        return cv2.bitwise_and(image, image, mask=final_mask)


    def roi(self, image):
        x, y = image.shape[1], image.shape[0]
        _shape = np.array([[(0, y), (x, y), (x, 0), (0,0)]], dtype=np.int32)
        mask = np.zeros_like(image)
        ignore_mask_color = (255,) * image.shape[2] if len(image.shape) > 2 else 255
        cv2.fillPoly(mask, np.int32([_shape]), ignore_mask_color)
        return cv2.bitwise_and(image, mask)

    def wrapping(self, image):
        h, w = image.shape[:2]
        source = np.float32([[0, h], [w-0, h], [40, 140], [w - 40, 140]])
        destination = np.float32([[0, h], [w, h], [0, 0], [w, 0]])
        transform_matrix = cv2.getPerspectiveTransform(source, destination)
        minv = cv2.getPerspectiveTransform(destination, source)
        return cv2.warpPerspective(image, transform_matrix, (w, h)), minv

    def plothistogram(self, image):
        histogram = np.sum(image[image.shape[0] // 2:, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        return np.argmax(histogram[:midpoint]), np.argmax(histogram[midpoint:]) + midpoint

    def slide_window_search(self, binary_warped, left_current, right_current):
        out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
        nwindows = 4
        window_height = int(binary_warped.shape[0] / nwindows)
        nonzero = binary_warped.nonzero()
        nonzero_y, nonzero_x = np.array(nonzero[0]), np.array(nonzero[1])
        margin, minpix = 60, 60
        left_lane, right_lane = [], []

        for w in range(nwindows):
            win_y_low = binary_warped.shape[0] - (w + 1) * window_height
            win_y_high = binary_warped.shape[0] - w * window_height
            win_xleft_low, win_xleft_high = left_current - margin, left_current + margin
            win_xright_low, win_xright_high = right_current - margin, right_current + margin

            # 왼쪽과 오른쪽 윈도우에서 검출된 픽셀 찾기
            good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                        (nonzero_x >= win_xleft_low) & (nonzero_x < win_xleft_high)).nonzero()[0]
            good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                        (nonzero_x >= win_xright_low) & (nonzero_x < win_xright_high)).nonzero()[0]
            
            # 픽셀이 있는 윈도우만 추가
            if len(good_left) > 0:
                left_lane.append(good_left)
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            if len(good_right) > 0:
                right_lane.append(good_right)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)

            # 업데이트: 검출된 픽셀이 minpix 이상일 때 윈도우 위치 갱신
            if len(good_left) > minpix:
                left_current = int(np.mean(nonzero_x[good_left]))
            if len(good_right) > minpix:
                right_current = int(np.mean(nonzero_x[good_right]))

        left_lane, right_lane = np.concatenate(left_lane), np.concatenate(right_lane)
        leftx, lefty = nonzero_x[left_lane], nonzero_y[left_lane]
        rightx, righty = nonzero_x[right_lane], nonzero_y[right_lane]

        if len(leftx) == 0 or len(lefty) == 0 or len(rightx) == 0 or len(righty) == 0:
            return {'left_fitx': np.array([]), 'right_fitx': np.array([]), 'ploty': np.array([])}, out_img

        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

        out_img[nonzero_y[left_lane], nonzero_x[left_lane]] = [255, 0, 0]
        out_img[nonzero_y[right_lane], nonzero_x[right_lane]] = [0, 0, 255]

        return {'left_fitx': np.trunc(left_fitx), 'right_fitx': np.trunc(right_fitx), 'ploty': ploty}, out_img


    # def calculate_curvature(self, left_fitx, right_fitx, ploty):
    #     L_ARC_M_Y = len(ploty) // 2
    #     R_ARC_M_Y = len(ploty) // 2

    #     # L_H와 R_H 계산
    #     L_H = math.sqrt((left_fitx[L_ARC_M_Y] * xm_per_pix - (left_fitx[0] * xm_per_pix + left_fitx[-1] * xm_per_pix) / 2) ** 2 +
    #                     (L_ARC_M_Y * ym_per_pix - 240 * ym_per_pix) ** 2)
    #     L_W = math.sqrt((left_fitx[0] * xm_per_pix - left_fitx[-1] * xm_per_pix) ** 2 + (480 * ym_per_pix) ** 2)

    #     R_H = math.sqrt((right_fitx[R_ARC_M_Y] * xm_per_pix - (right_fitx[0] * xm_per_pix + right_fitx[-1] * xm_per_pix) / 2) ** 2 +
    #                     (R_ARC_M_Y * ym_per_pix - 240 * ym_per_pix) ** 2)
    #     R_W = math.sqrt((right_fitx[0] * xm_per_pix - right_fitx[-1] * xm_per_pix) ** 2 + (480 * ym_per_pix) ** 2)

    #     # L_H와 R_H가 0일 때 곡률 계산을 생략하여 에러 방지
    #     L_RAD = ((L_H / 2) + (L_W ** 2 / (8 * L_H))) if L_H != 0 else float('inf')
    #     R_RAD = ((R_H / 2) + (R_W ** 2 / (8 * R_H))) if R_H != 0 else float('inf')

    #     return L_RAD, R_RAD

    # def calculate_center_curvature(self, L_RAD, R_RAD):
    #     return (L_RAD + R_RAD) / 2 if L_RAD and R_RAD else None

    # def calculate_lane_angle(self, left_fitx, right_fitx, ploty):
    #     mean_x = (left_fitx + right_fitx) / 2
    #     top_y = ploty[0]
    #     bottom_y = ploty[-1]
    #     top_x = mean_x[0]
    #     bottom_x = mean_x[-1]
    #     angle_rad = math.atan2(abs(bottom_x - top_x), abs(bottom_y - top_y))
    #     return math.degrees(angle_rad)

    # def detect_lane_direction(self, left_fitx, right_fitx):
    #     start_x = (left_fitx[0] + right_fitx[0]) / 2
    #     end_x = (left_fitx[-1] + right_fitx[-1]) / 2
    #     return "Left" if end_x > start_x else "Right" if end_x < start_x else "Straight"

    def draw_lane_lines_with_offset(self, original_image, warped_image, Minv, draw_info):
        left_fitx, right_fitx, ploty = draw_info['left_fitx'], draw_info['right_fitx'], draw_info['ploty']
        warp_zero = np.zeros_like(warped_image).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]).astype(np.int32)
        pts = np.hstack((pts_left, pts_right))

        mean_x = (left_fitx + right_fitx) / 2
        # pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))], dtype=np.int32)
        
        cv2.fillPoly(color_warp, np.int_([pts]), (216, 168, 74))

        newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
        result = cv2.addWeighted(original_image, 1, newwarp, 0.4, 0)

        # 초록색 점으로 중앙선을 그림
        for y, x in zip(ploty.astype(int), mean_x.astype(int)):
            cv2.circle(result, (x, y), 3, (0, 255, 0), -1)

        # 빨간색 기준점 추가 (320, 430)
        red_point_x, red_point_y = 320, 330
        cv2.circle(result, (red_point_x, red_point_y), 5, (0, 0, 255), -1)

        # offset 계산
        if len(mean_x) > 0 and red_point_y in ploty:
            center_x_at_430 = int(mean_x[ploty.astype(int) == red_point_y][0])
            offset = (center_x_at_430 - red_point_x) * xm_per_pix * 100

        print(f"Calculated offset: {offset:.2f} cm")
        return result, offset
        
class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.offset = 0.0
        self.integral_offset = 0.0  # 적분 누적값
        self.Kp = 0.7  # 비례 계수
        self.Ki = 0.1  # 적분 계수, 필요에 따라 조정
        self.condition = threading.Condition()
        self.done = False
        self.timeout = 0.05 / rate if rate != 0.0 else None
        self.camera_off_printed = False
        self.start()

    def reset_twist(self, twist):
        """
        twist 객체를 초기화하고 즉시 초기화된 상태를 발행합니다.
        """
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)  # 초기화된 명령 즉시 발행

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            self.condition.wait(self.timeout)

            if camera_on:
                # 카메라가 켜져 있을 때만 방향과 속도를 계산
                self.update_movement_based_on_offset()
                self.reset_twist(twist)
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.z = self.th * self.turn
                self.camera_off_printed = False
            else:
                # 카메라가 꺼져 있을 때는 이동 멈춤
                twist.linear.x = twist.linear.y = twist.linear.z = 0
                twist.angular.x = twist.angular.y = twist.angular.z = 0

                if not self.camera_off_printed:
                    print("Camera is off, stopping movement.")
                    self.camera_off_printed = True  # 메시지를 한 번 출력했음을 표시    

            self.condition.release()
            self.publisher.publish(twist)
            
            if camera_on:
                line = "=" * 10
                print(line)
                print(f"turn: {self.turn:.2f},\n speed: {self.speed:.2f}")

        twist.linear.x = twist.linear.y = twist.linear.z = 0
        twist.angular.x = twist.angular.y = twist.angular.z = 0
        self.publisher.publish(twist)

    def update_offset(self, offset):
        self.offset = offset

    def update_movement_based_on_offset(self):
        """
        offset 값에 따라 이동 방향, 회전 및 속도를 설정하고 즉시 pub_thread에 반영합니다.
        """
        # 초기 설정
        if self.offset > 1.5:
            x, y, z, th = 1, 0, 0, -1  # 왼쪽 회전
            turn = 0.15
            speed = 0.1

        elif self.offset < -1.5:
            x, y, z, th = 1, 0, 0, 1  # 오른쪽 회전
            turn = 0.15
            speed = 0.1

        elif -1.5 <= self.offset <= 1.5:
            x, y, z, th = 1, 0, 0, 0  # 직진
            turn = 0
            speed = 0.15

        # PI 제어를 사용해 추가 회전 보정
        # self.integral_offset += self.offset
        # offset_ratio = (self.Kp * self.offset) + (self.Ki * self.integral_offset)
        # turn = max(-1.0, min(1.0, turn + offset_ratio))

        # 계산된 값들을 즉시 pub_thread에 업데이트하여 반영
        self.update(x, y, z, th, speed, turn)

    
def getKey(timeout):
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
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = saveTerminalSettings()
    rospy.init_node('teleop_twist_keyboard')
    speed = rospy.get_param("~speed", 0)
    turn = rospy.get_param("~turn", 0)
    speed_limit = rospy.get_param("~speed_limit", 1.0)
    turn_limit = rospy.get_param("~turn_limit", 1.0)
    repeat = rospy.get_param("~repeat_rate", 0.05)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    
    pub_thread = PublishThread(repeat)
    camera_thread = CameraThread()
    processing_thread = CameraProcessingThread(camera_thread, pub_thread)

    camera_on = False
    x = y = z = th = 0
    status = 0

    try:
        pub_thread.update(x, y, z, th, speed, turn)
        print(msg)
        print(vels(speed,turn))

        while not rospy.is_shutdown():
            key = getKey(key_timeout)
            if key == 'v':
                camera_on = not camera_on
                if camera_on:
                    try:
                        camera_thread.start()
                        processing_thread.start()
                    except Exception as e:
                        print("Error starting camera or processing thread:", e)
                        camera_on = False
                else:
                    camera_thread.stop()
                    processing_thread.stop()
            elif key in moveBindings:
                x, y, z, th = moveBindings[key]
            elif key in speedBindings:
                x *= speedBindings[key][0]
                turn *= speedBindings[key][1]
            else:
                x = y = z = th = 0
                if key == '\x03':
                    break

            # 업데이트된 방향과 속도로 명령 전송
            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print("An unexpected error occurred:", e)

                                
    finally:
        try:
            camera_thread.stop()
            processing_thread.stop()
            pub_thread.stop()
        except Exception as e:
            print("Error while stopping threads:", e)
        restoreTerminalSettings(settings)
        cv2.destroyAllWindows()
