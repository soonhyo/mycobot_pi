#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np

def point_cloud_callback(msg):
    global point_cloud

    gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    point_cloud = np.array([p for p in gen if p[2] <= 0.5])  # p[2] is the 'z' field

    # Create a point_cloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id
    filtered_pc2 = pc2.create_cloud_xyz32(header, point_cloud.tolist())

    # Publish the PointCloud2 message
    pub.publish(filtered_pc2)

rospy.init_node("point_cloud_filter")
rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_cloud_callback)
pub = rospy.Publisher("/filtered_points", PointCloud2, queue_size=10)
rospy.spin()
