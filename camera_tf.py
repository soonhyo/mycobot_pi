#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations as tft
import numpy as np


if __name__ == "__main__":
    rospy.init_node("tf_broadcaster")

    # tf broadcaster 생성
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # 변환(Transform) 생성
    transform = geometry_msgs.msg.TransformStamped()

    # 변환 시간 및 프레임 설정
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "link1"
    transform.child_frame_id = "camera_link"

    roll = np.deg2rad(0)  # roll in radians
    pitch = np.deg2rad(0)  # pitch in radians
    yaw = np.deg2rad(180)  # yaw in radians

    q = tft.quaternion_from_euler(roll, pitch, yaw)

    # 변환 파라미터 설정
    transform.transform.translation.x = 0.25
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.20
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]

    rate = rospy.Rate(10.0) # 10Hz
    while not rospy.is_shutdown():
        # 변환 시간 업데이트
        transform.header.stamp = rospy.Time.now()

        # 변환 broadcast
        tf_broadcaster.sendTransform(transform)

        rate.sleep()
