#!/usr/bin/env python
import rospy
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

# 전역 변수
initial_pose = None
current_amcl_pose = None

# 쓰레드 종료 플래그
shutdown_event = threading.Event()

# /amcl_pose 콜백 함수
def amcl_pose_callback(msg):
    """AMCL 위치 업데이트 콜백 함수"""
    global current_amcl_pose, initial_pose
    current_amcl_pose = msg.pose.pose
    if initial_pose is None:
        initial_pose = current_amcl_pose
        rospy.loginfo(f"Initial Pose saved: x={initial_pose.position.x}, y={initial_pose.position.y}")

# 목표 지점으로 이동하는 함수
def move_to_goal(goal_x, goal_y, goal_yaw):
    """move_base 액션 클라이언트를 사용하여 목표 지점으로 이동"""
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # MoveBaseGoal 메시지 생성
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # 목표 위치와 방향 설정
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)

    rospy.loginfo(f"Sending goal: x={goal_x}, y={goal_y}, yaw={goal_yaw}")
    client.send_goal(goal)

    # 결과 대기
    client.wait_for_result()
    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
        return True
    else:
        rospy.logwarn("Failed to reach goal.")
        return False

# Yaw 값 계산 함수
def calculate_yaw(from_x, from_y, to_x, to_y):
    """두 점 사이의 Yaw 값 계산"""
    return math.atan2(to_y - from_y, to_x - from_x)

# 쓰레드: AMCL 위치 업데이트
def amcl_listener_thread():
    """AMCL 위치 업데이트를 위한 쓰레드"""
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.loginfo("AMCL Listener thread started.")
    rospy.spin()  # ROS 콜백 실행

# 쓰레드: 목표 이동
def move_base_thread(targets):
    """목표 지점으로 이동"""
    global initial_pose
    for idx, (x, y, yaw) in enumerate(targets):
        if shutdown_event.is_set():
            rospy.loginfo("Shutdown signal received. Exiting move_base_thread.")
            break

        rospy.loginfo(f"Moving to target {idx + 1}: x={x}, y={y}, yaw={yaw}")
        if not move_to_goal(x, y, yaw):
            rospy.logwarn(f"Failed to reach target {idx + 1}.")
            break

    # 초기 위치로 복귀
    if initial_pose:
        rospy.loginfo("Returning to initial position...")
        reverse_yaw = calculate_yaw(initial_pose.position.x, initial_pose.position.y,
                                    current_amcl_pose.position.x, current_amcl_pose.position.y)
        if move_to_goal(initial_pose.position.x, initial_pose.position.y, reverse_yaw):
            rospy.loginfo("Returned to initial position successfully.")
        else:
            rospy.logwarn("Failed to return to initial position.")

def main():
    global shutdown_event
    rospy.init_node('move_base_client', anonymous=False)

    # 목표 지점 설정
    targets = [
        (2.4019315242767334, -0.3599012494087219, 0.0),  # 첫 번째 목표 지점
    ]

    # 쓰레드 시작
    amcl_thread = threading.Thread(target=amcl_listener_thread)
    move_thread = threading.Thread(target=move_base_thread, args=(targets,))

    amcl_thread.start()
    move_thread.start()

    try:
        # 노드 실행 대기
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt detected. Shutting down.")
    finally:
        # 종료 플래그 설정 및 쓰레드 종료 대기
        shutdown_event.set()
        amcl_thread.join()
        move_thread.join()
        rospy.loginfo("All threads terminated.")

if __name__ == '__main__':
    main()
