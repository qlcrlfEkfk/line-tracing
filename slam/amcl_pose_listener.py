#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped

current_amcl_pose = None

def amcl_pose_callback(msg):
    global current_amcl_pose
    current_amcl_pose = msg.pose.pose

def clicked_point_callback(msg):
    global current_amcl_pose
    if current_amcl_pose:
        rospy.loginfo(f"Clicked Point: x={msg.point.x}, y={msg.point.y}")
        rospy.loginfo(f"AMCL Pose: x={current_amcl_pose.position.x}, y={current_amcl_pose.position.y}")

def main():
    rospy.init_node('amcl_pose_and_point_listener', anonymous=True)

    # /amcl_pose�� /clicked_point ����
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
    rospy.Subscriber('/clicked_point', PointStamped, clicked_point_callback)

    rospy.loginfo("Listening to /amcl_pose and /clicked_point...")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")



