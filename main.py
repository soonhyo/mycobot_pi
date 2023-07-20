#!/usr/bin/env python3

from model_registration import align_point_clouds
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf_conversions
from geometry_msgs.msg import TransformStamped

def camera_point_cloud_callback(msg):
    # ROS PointCloud2를 numpy array로 변환
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    global camera_pointcloud
    camera_pointcloud = np.array([p for p in gen])

def robot_point_cloud_callback(msg):
    # ROS PointCloud2를 numpy array로 변환
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    robot_pointcloud = np.array([p for p in gen])

    # 카메라 포인트 클라우드와 로봇의 포인트 클라우드 정합
    transformation = align_point_clouds(camera_pointcloud, robot_pointcloud)

    # 결과 출력
    print("Transformation Matrix:")
    print(transformation)

    # tf 메시지 생성
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "link1"
    transform_stamped.child_frame_id = "camera_color_optical_frame"
    transform_stamped.transform.translation.x = transformation[0, 3]
    transform_stamped.transform.translation.y = transformation[1, 3]
    transform_stamped.transform.translation.z = transformation[2, 3]
    q = tf_conversions.transformations.quaternion_from_matrix(transformation)
    transform_stamped.transform.rotation.x = q[0]
    transform_stamped.transform.rotation.y = q[1]
    transform_stamped.transform.rotation.z = q[2]
    transform_stamped.transform.rotation.w = q[3]

    # tf 브로드캐스트
    br.sendTransform(transform_stamped)

if __name__ == "__main__":
    rospy.init_node('pointcloud_listener')

    # tf broadcaster 생성
    br = tf2_ros.TransformBroadcaster()

    # 카메라에서 얻은 point cloud를 받기 위한 subscriber 설정
    rospy.Subscriber("/filtered_points", PointCloud2, camera_point_cloud_callback)

    # 로봇의 point cloud 데이터를 받기 위한 subscriber 설정
    rospy.Subscriber("/robot_point_cloud", PointCloud2, robot_point_cloud_callback)

    rospy.spin()
