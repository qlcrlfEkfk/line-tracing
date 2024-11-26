#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

# 초기 위치 저장
initial_pose = None
current_amcl_pose = None

# /amcl_pose 콜백 함수
def amcl_pose_callback(msg):
    global current_amcl_pose, initial_pose
    current_amcl_pose = msg.pose.pose
    if initial_pose is None:  # 처음 위치가 설정되지 않았으면 저장
        initial_pose = current_amcl_pose
        rospy.loginfo(f"Initial Pose saved: x={initial_pose.position.x}, y={initial_pose.position.y}")

# 목표 지점으로 이동하는 함수
def move_to_goal(goal_x, goal_y, goal_yaw):
    # move_base 액션 클라이언트 초기화
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base server")

    # MoveBaseGoal 메시지 생성
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # "map" 좌표계를 기준으로 설정
    goal.target_pose.header.stamp = rospy.Time.now()

    # 목표 위치와 방향 설정
    goal.target_pose.pose.position.x = goal_x
    goal.target_pose.pose.position.y = goal_y
    goal.target_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
    goal.target_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)  # 방향 조정

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

# Yaw 값 계산 함수 (목표 지점에서 초기 위치로의 방향)
def calculate_yaw(from_x, from_y, to_x, to_y):
    return math.atan2(to_y - from_y, to_x - from_x)

def main():
    global initial_pose

    rospy.init_node('move_base_client', anonymous=False)

    # AMCL 위치 구독
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    rospy.loginfo("Waiting for initial pose to be saved...")
    while initial_pose is None:  # 초기 위치가 저장될 때까지 대기
        rospy.sleep(0.1)

    rospy.loginfo(f"Initial Pose: x={initial_pose.position.x}, y={initial_pose.position.y}")

    # 목표 지점 리스트
    targets = [
        (2.4019315242767334, -0.3599012494087219, 0),  # 첫 번째 점
        (2.2925639152526855, -1.686120867729187, 0),  # 두 번째 점
        (3.5678048133850098, -1.715164065361023, 0)  # 세 번째 점
    ]

    # 각 목표 지점으로 이동
    for idx, (x, y, yaw) in enumerate(targets):
        rospy.loginfo(f"Moving to target {idx + 1}: x={x}, y={y}, yaw={yaw}")
        if not move_to_goal(x, y, yaw):
            rospy.logwarn(f"Failed to reach target {idx + 1}. Skipping remaining targets.")
            return

    rospy.loginfo("All targets reached. Returning to initial position via checkpoints...")

    # 돌아오는 경유 지점 리스트
    return_points = [
        (2.2925639152526855, -1.686120867729187, 0),  # 첫 번째 경유점
        (2.4019315242767334, -0.3599012494087219, 0)  # 두 번째 경유점
    ]

    # 경유 지점을 순차적으로 방문
    for idx, (x, y, yaw) in enumerate(return_points):
        rospy.loginfo(f"Returning via checkpoint {idx + 1}: x={x}, y={y}, yaw={yaw}")
        if not move_to_goal(x, y, yaw):
            rospy.logwarn(f"Failed to reach checkpoint {idx + 1}. Skipping remaining checkpoints.")
            return

    # 초기 위치로 복귀 (역방향으로 도착)
    rospy.loginfo("Returning to the initial position in reverse direction...")
    reverse_yaw = calculate_yaw(initial_pose.position.x, initial_pose.position.y, 
                                current_amcl_pose.position.x, current_amcl_pose.position.y)
    if move_to_goal(initial_pose.position.x, initial_pose.position.y, reverse_yaw):  # 역방향 Yaw 값 사용
        rospy.loginfo("Returned to the initial position successfully in reverse direction!")
    else:
        rospy.logwarn("Failed to return to the initial position.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node terminated.")
