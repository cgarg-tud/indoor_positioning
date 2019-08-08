from functools import partial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pycpd import rigid_registration
import numpy as np
import time


# based on  examples/Python/Advanced/global_registration.py
# http://www.open3d.org/docs/tutorial/Advanced/global_registration.html

from open3d import *

import copy




def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([0.9, 0.1, 0.1])
    target_temp.paint_uniform_color([0.1, 0.1, 0.7])
    
#    
#    source_temp.paint_uniform_color([1, 0.706, 0])
#    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

# reads a source point cloud and a target point cloud from two files. They are misaligned with an identity matrix as transformation.
def prepare_dataset(voxel_size,inp,db, cam_pos,jparams):
   
    print(":: Load two point clouds and disturb initial pose.")
        
    source = inp
    target = db
    draw_registration_result(source, target, np.identity(4))

    start = time.time()
    
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    
   
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size, cam_pos,jparams)
    
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size, 0,jparams)

    end=time.time()
    pre_time = float(round(end-start,2))
    print ("prepare_dataset Runtime: " + str(round(end-start,2))+" sec\n\n")
    return source, target, source_down, target_down, source_fpfh, target_fpfh, pre_time

def remove_outliers(pcd,jparams):

    inlier_cloud = pcd
    
    if (jparams['add_ceiling']["check"] == "no"):    
    
        print("Statistical oulier removal")
        cl,ind = statistical_outlier_removal(inlier_cloud,
                nb_neighbors= jparams["stats_clean"]["nb_neighbors"], std_ratio= jparams["stats_clean"]["std_ratio"])
        display_inlier_outlier(inlier_cloud, ind)
        print(cl)
        print(inlier_cloud.points)
        inlier_cloud = select_down_sample(inlier_cloud, ind)
        
        print("Radius oulier removal")
        cl,ind = radius_outlier_removal( inlier_cloud ,
                nb_points= jparams["radius_clean" ]["nb_points"], radius=jparams["radius_clean" ]["radius"])
        display_inlier_outlier( inlier_cloud , ind)
        print(inlier_cloud.points)
        print(cl)
        inlier_cloud = select_down_sample(inlier_cloud, ind)
        draw_geometries([inlier_cloud])
        
    else:        
        print("Statistical oulier removal")
        cl,ind = statistical_outlier_removal(inlier_cloud,
                nb_neighbors= jparams["add_ceiling"]["stats_clean"]["nb_neighbors"], std_ratio= jparams["add_ceiling"]["stats_clean"]["std_ratio"])
        display_inlier_outlier(inlier_cloud, ind)
        print(cl)
        print(inlier_cloud.points)
        inlier_cloud = select_down_sample(inlier_cloud, ind)
        
        print("Radius oulier removal")
        cl,ind = radius_outlier_removal( inlier_cloud ,
                nb_points= jparams["add_ceiling"]["radius_clean" ]["nb_points"], radius=jparams["add_ceiling"]["radius_clean" ]["radius"])
        display_inlier_outlier( inlier_cloud , ind)
        print(inlier_cloud.points)
        print(cl)
        inlier_cloud = select_down_sample(inlier_cloud, ind)
        draw_geometries([inlier_cloud])      
        
    
    return inlier_cloud
    
    
def display_inlier_outlier(cloud, ind):
    inlier_cloud = select_down_sample(cloud, ind)
    outlier_cloud = select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    draw_geometries([inlier_cloud, outlier_cloud])


def preprocess_point_cloud(pcd, voxel_size, cam_pos,jparams):
    start=time.time()
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = voxel_down_sample(pcd, voxel_size)


    radius_normal = voxel_size * jparams['features']["radius_normal" ]
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = int(jparams['features']["max_nn"])))   
    
    
    print(":: Reorient  normals with search radius %.3f." % radius_normal)
    if(type(cam_pos) != int):
        orient_normals_towards_camera_location(pcd_down,cam_pos)

    
    radius_feature = voxel_size * jparams['features']["radius_feature"]
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = jparams['features']["max_nn_f"]))
    end=time.time()
    
    print ("pre_process Runtime: " + str(round(end-start,2))+" sec\n\n")
    return pcd_down, pcd_fpfh




def execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size,jparams):
    start=time.time()
    distance_threshold = voxel_size * float(jparams["matching"]["distance_threshold"])
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)

    
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(True), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(float(jparams["matching"]["edge_threshold"])),
             CorrespondenceCheckerBasedOnDistance(distance_threshold), CorrespondenceCheckerBasedOnNormal(int(jparams["matching"]["normal_threshold"]))],
            RANSACConvergenceCriteria(jparams["matching"]["convergenceCriteria"][0],jparams["matching"]["convergenceCriteria"][1]))
    end=time.time()
    print ("global registration Runtime: " + str(round(end-start,2))+" sec\n\n")
    return result



def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    start=time.time()
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = registration_icp(source, target, distance_threshold,
            result_ransac.transformation,
            TransformationEstimationPointToPlane())
    end=time.time()
    print ("refineregistration Runtime: " + str(round(end-start,2))+" sec\n\n")
    return result



        
def improve_alignment(source, target, trans_init, jparams):   
    
    threshold = jparams["optimize"]["refine_distance_threshold"]
    draw_registration_result(source, target, trans_init)
    print("Initial alignment")
    evaluation = evaluate_registration(source, target,
            threshold, trans_init)
    print(evaluation)
    print("Apply point-to-point ICP")
    reg_p2p = registration_icp(source, target, threshold, trans_init,
            TransformationEstimationPointToPoint())
    print(reg_p2p)
    print("Transformation is:")
    print(reg_p2p.transformation)
    print("")
    draw_registration_result(source, target, reg_p2p.transformation)

    return reg_p2p
