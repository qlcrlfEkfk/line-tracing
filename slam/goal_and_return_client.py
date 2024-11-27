#!/usr/bin/env python
import rospy
import threading
import socket
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

# 전역 변수
initial_pose = None
current_amcl_pose = None
ip_address = '172.30.1.35'
port = 5000

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

# 소켓 통신 함수
def communicate_with_pc():
    try:
        rospy.loginfo("Connecting to PC for communication...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ip_address, port))  # PC의 IP와 포트
            s.sendall(b"AGV has reached the initial position.")
            rospy.loginfo("Signal sent to PC. Waiting for response...")

            # PC로부터 응답 수신
            response = s.recv(1024)
            if response:
                rospy.loginfo(f"Response from PC: {response.decode('utf-8')}")
                return True
            else:
                rospy.logwarn("No response from PC.")
                return False
    except Exception as e:
        rospy.logerr(f"Communication error with PC: {e}")
        return False

def main():
    rospy.init_node('move_base_client', anonymous=False)

    # AMCL 위치 구독
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)

    rospy.loginfo("Waiting for initial pose to be saved...")
    while initial_pose is None:
        rospy.sleep(0.1)

    rospy.loginfo(f"Initial Pose: x={initial_pose.position.x}, y={initial_pose.position.y}")

    # 목표 지점 설정
    target_x = 2.4019315242767334
    target_y = -0.3599012494087219
    target_yaw = 0.0

    # 목표 지점으로 이동
    if move_to_goal(target_x, target_y, target_yaw):
        rospy.loginfo("Reached the target. Communicating with PC...")
        if communicate_with_pc():
            rospy.loginfo("PC acknowledged. Returning to the initial position...")
            reverse_yaw = calculate_yaw(initial_pose.position.x, initial_pose.position.y,
                                        current_amcl_pose.position.x, current_amcl_pose.position.y)
            move_to_goal(initial_pose.position.x, initial_pose.position.y, reverse_yaw)
        else:
            rospy.logwarn("PC did not acknowledge. Aborting return.")
    else:
        rospy.logwarn("Failed to reach the target.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation node terminated.")
