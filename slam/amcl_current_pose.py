#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

# 현재 위치를 저장할 변수
current_pose = None

# 콜백 함수: /amcl_pose 토픽에서 데이터 수신
def callback(msg):
    global current_pose
    current_pose = msg.pose.pose
    # 위치 정보 출력
    rospy.loginfo(f"Current Pose: x={current_pose.position.x}, y={current_pose.position.y}, orientation_z={current_pose.orientation.z}, orientation_w={current_pose.orientation.w}")

def main():
    # ROS 노드 초기화
    rospy.init_node('current_pose_listener', anonymous=False)

    # /amcl_pose 구독 시작
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)

    # ROS 스핀 실행
    rospy.loginfo("Listening to /amcl_pose for current position...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated due to interruption.")
    except Exception as e:
        rospy.logerr(f"Unexpected error: {e}")
