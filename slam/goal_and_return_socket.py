#!/usr/bin/env python
import rospy
import threading
import actionlib
import socket
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

# 전역 변수
initial_pose = None
current_amcl_pose = None
shutdown_event = threading.Event()  # 쓰레드 종료 플래그

# /amcl_pose 콜백 함수
def amcl_pose_callback(msg):
    global current_amcl_pose, initial_pose
    current_amcl_pose = msg.pose.pose
    if initial_pose is None:
        initial_pose = current_amcl_pose
        rospy.loginfo(f"Initial Pose saved: x={initial_pose.position.x}, y={initial_pose.position.y}")

# 목표 지점으로 이동하는 함수
def move_to_goal(goal_x, goal_y, goal_yaw):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

    rospy.loginfo(f"Sending goal: x={goal_x}, y={goal_y}, yaw={goal_yaw}")
    client.send_goal(goal)

    client.wait_for_result()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        return True
    else:
        rospy.logwarn("Failed to reach goal.")
        return False

# Yaw 값 계산 함수
def calculate_yaw(from_x, from_y, to_x, to_y):
    return math.atan2(to_y - from_y, to_x - from_x)

# 소켓 신호 전송 함수
def send_signal_to_pc():
    try:
        rospy.loginfo("Connecting to PC for signal transmission...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect(('192.168.1.100', 5000))  # PC의 IP와 포트
            s.sendall(b"AGV has reached the initial position.")
            rospy.loginfo("Signal sent to PC.")
    except Exception as e:
        rospy.logerr(f"Failed to send signal to PC: {e}")

# AMCL 위치 업데이트 쓰레드
def amcl_listener_thread():
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.spin()

# 목표 이동 쓰레드
def move_base_thread(targets):
    global initial_pose
    for idx, (x, y, yaw) in enumerate(targets):
        if shutdown_event.is_set():
            break

        rospy.loginfo(f"Moving to target {idx + 1}: x={x}, y={y}, yaw={yaw}")
        if not move_to_goal(x, y, yaw):
            rospy.logwarn(f"Failed to reach target {idx + 1}.")
            break

    if initial_pose:
        rospy.loginfo("Returning to initial position...")
        reverse_yaw = calculate_yaw(initial_pose.position.x, initial_pose.position.y,
                                    current_amcl_pose.position.x, current_amcl_pose.position.y)
        if move_to_goal(initial_pose.position.x, initial_pose.position.y, reverse_yaw):
            rospy.loginfo("Returned to initial position successfully.")
            send_signal_to_pc()
        else:
            rospy.logwarn("Failed to return to initial position.")

def main():
    rospy.init_node('move_base_client', anonymous=False)
    targets = [(2.4019315242767334, -0.3599012494087219, 0.0)]

    amcl_thread = threading.Thread(target=amcl_listener_thread)
    move_thread = threading.Thread(target=move_base_thread, args=(targets,))

    amcl_thread.start()
    move_thread.start()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt detected. Shutting down.")
    finally:
        shutdown_event.set()
        amcl_thread.join()
        move_thread.join()
        rospy.loginfo("All threads terminated.")

if __name__ == '__main__':
    main()
