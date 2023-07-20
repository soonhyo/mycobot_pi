#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
from stl import mesh
import rospkg
def mesh_to_point_cloud(stl_mesh):
    # STL mesh에서 모든 vertices를 가져온다
    all_points = np.vstack([
        stl_mesh.v0,
        stl_mesh.v1,
        stl_mesh.v2
    ])

    # 중복되는 vertices를 제거한다
    unique_points = np.unique(all_points, axis=0)

    return unique_points

if __name__ == "__main__":
    rospy.init_node('stl_to_pointcloud_publisher')
    rospack = rospkg.RosPack()
    # STL 파일 로드
    stl_mesh = mesh.Mesh.from_file(rospack.get_path("mycobot_description")+"/urdf/mycobot/mesh/mycobot.stl")

    # STL 메시를 포인트 클라우드로 변환
    point_cloud = mesh_to_point_cloud(stl_mesh)

    # 포인트 클라우드를 ROS 메시지로 변환
    header = Header()
    header.frame_id = "link1"
    ros_point_cloud = pc2.create_cloud_xyz32(header, point_cloud)

    # 포인트 클라우드를 publish
    pub = rospy.Publisher("robot_point_cloud", PointCloud2, queue_size=10)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(ros_point_cloud)
        rate.sleep()
