#!/usr/bin/env python3
import open3d as o3d
import copy
import rospy
import pcl





import numpy as np
from ctypes import * # convert float to uint32

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)



def convertCloudFromRosToOpen3d(ros_cloud):
    
    # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = o3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    if "rgb" in field_names:
        IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
        # Get xyz
        xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

        # Get rgb
        # Check whether int or float
        if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
            rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
        else:
            rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

        # combine
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))
        open3d_cloud.colors = o3d.utility.Vector3dVector(np.array(rgb)/255)
    else:
        xyz = [(x,y,z) for x,y,z in cloud_data ] # get xyz
        open3d_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # return
    return open3d_cloud










def detect_ball(pcd):


    # Downsample the point cloud to reduce computation time
    pcd_down = pcd.voxel_down_sample(voxel_size=0.005)


    print('sdfdfdd')
    # Estimate surface normals
    pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=30))
    o3d.geometry.PointCloud.orient_normals_towards_camera_location(pcd_down)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=100))
    # Segment the plane using RANSAC
    
    outlier_filter = pcd_down.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
    plane_model, inliers = pcd_down.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=1000)
    inlier_cloud = pcd_down.select_by_index(inliers)
    outlier_cloud = pcd_down.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([1, 0, 0])    

    # Extract the ball using Euclidean clustering
    labels = np.array(outlier_cloud.cluster_dbscan(eps=0.003, min_points=30, print_progress=False))
    max_label = labels.max()
    cluster_centers = []
    for i in range(max_label + 1):
        mask = labels == i
        cluster_points = outlier_cloud.select_by_index(np.where(mask)[0])
        cluster_center = np.asarray(cluster_points.get_center())
        cluster_centers.append(cluster_center)
        print('dsfdgdg')

    # Return the center of the largest cluster (assumed to be the ball)
    if len(cluster_centers) > 0:
        cluster_centers = np.asarray(cluster_centers)
        cluster_distances = np.linalg.norm(cluster_centers, axis=1)
        largest_cluster_idx = np.argmax(cluster_distances)
        ball_center = cluster_centers[largest_cluster_idx]
        rospy.loginfo("Ball Center: "+str(ball_center))
        return ball_center
    else:
        return None

    o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])




def callback(data):
    # Convert PointCloud2 message to pcl.PointCloud object
    cloud = convertCloudFromRosToOpen3d(data)
    detect_ball(cloud)
  

rospy.init_node('pixel_to_point', anonymous=True)
rospy.Subscriber('/rs_plugin/depth/points', PointCloud2, callback)
rospy.spin()

    