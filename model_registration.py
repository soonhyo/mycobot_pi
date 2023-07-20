import open3d as o3d
import numpy as np

def align_point_clouds(cloud_source, cloud_target):
    # Numpy arrays를 Open3D 포인트 클라우드로 변환
    source = o3d.geometry.PointCloud()
    target = o3d.geometry.PointCloud()

    source.points = o3d.utility.Vector3dVector(cloud_source)
    target.points = o3d.utility.Vector3dVector(cloud_target)

    # Point clouds를 다운 샘플링
    source = source.voxel_down_sample(voxel_size=0.05)
    target = target.voxel_down_sample(voxel_size=0.05)

    # 각 포인트 클라우드의 중심을 계산
    centroid_source = source.get_center()
    centroid_target = target.get_center()

    # 두 포인트 클라우드가 같은 중심을 가지도록 전역 변환을 적용
    trans_init = np.eye(4)
    trans_init[0:3, 3] = centroid_target - centroid_source
    source.transform(trans_init)

    # ICP를 실행하여 최적의 변환 행렬을 찾음
    threshold = 0.02
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())

    # 전체 변환 행렬 출력 (초기 전역 변환 + ICP 변환)
    return np.dot(reg_p2p.transformation, trans_init)
