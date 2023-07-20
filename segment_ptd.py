#!/usr/bin/env python3

import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, Image
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt

rospy.init_node("model_registration_node")

# CvBridge 객체 생성
bridge = CvBridge()

# depth 이미지와 point cloud 초기화
depth_image = None
point_cloud = None

# Selected pixel callback
def clicked_point_callback(msg):
    global depth_image, point_cloud
    selected_pixel = [int(msg.x), int(msg.y)]

    # Depth 이미지와 point cloud가 존재하고, 선택한 픽셀이 유효한 경우에만 실행
    if depth_image is not None and point_cloud is not None and selected_pixel is not None:
        # Depth 이미지에서 선택한 픽셀의 깊이 값 찾기
        selected_depth = depth_image[selected_pixel[1], selected_pixel[0]]

        # Point cloud에서 선택한 픽셀에 해당하는 포인트 찾기
        selected_point = point_cloud[selected_pixel[1], selected_pixel[0]]

        # Point cloud를 Open3D의 PointCloud 객체로 변환
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(point_cloud.reshape(-1, 3))

        # 선택한 포인트 주변의 포인트를 세그멘테이션
        if not np.isnan(selected_point).any():  # Add check for NaN values
            labels = np.array(cloud.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
            max_label = labels.max()
            print(f"point cloud has {max_label + 1} clusters")
            colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            colors[labels < 0] = 0
            cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
            o3d.visualization.draw_geometries([cloud])
# Point cloud callback
def point_cloud_callback(msg):
    global point_cloud
    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
    point_cloud = np.array(list(gen)).reshape(msg.height, msg.width, 3)

# Depth image callback
def depth_image_callback(msg):
    global depth_image
    depth_image = bridge.imgmsg_to_cv2(msg, "passthrough")

# Subscriber 설정
rospy.Subscriber("/camera/color/image_raw_mouse_left", Point, clicked_point_callback)
rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_image_callback)
rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_cloud_callback)

rospy.spin()
