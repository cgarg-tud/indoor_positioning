# -*- coding: utf-8 -*-
"""
Created on Thu Jun 27 08:59:49 2019

@author: chira
"""

from first_alignment1 import *
from plane_extract import *
from creating_tekstfiles import *
from find_hist_features import *
from test_pose_optimization import *


# based on  examples/Python/Advanced/global_registration.py
# http://www.open3d.org/docs/tutorial/Advanced/global_registration.html

from open3d import *

import copy

def do_rigid_alignment(dirName,database_name, user_name, pcd_user, pcd_database,cam_pos, jparams) :
    
  
    inp = pcd_user

    Y = textfile_user(pcd_user, user_name, dirName,jparams)
    X = textfile_database(pcd_database, database_name, dirName,jparams)
    
    parameters = rigid_alignment(X, Y)
    print(parameters) 
    
    file = open(dirName + '/' + database_name + user_name + '.xyz', 'w')
    index = 0 
    for point in inp.points:
        index = index + 1 
        transformed_point = parameters[0] * np.dot(point, parameters[2]) + np.tile(parameters[1], 1)
        file.write(str(transformed_point[0]) + ' ' + str(transformed_point[1]) + ' ' + str(transformed_point[2]) + '\n') 

    file.close()
    print('finished')
    print(index)    
    cam_pos = parameters[0] * np.dot(cam_pos, parameters[2]) + np.tile(parameters[1], 1)
    
    return cam_pos



def find_similarity(db, inp, voxel_size,cam_pos,jparams):     
    
    source, target, source_down, target_down, source_fpfh, target_fpfh, pre_time = \
            prepare_dataset(voxel_size, inp,db,cam_pos,jparams)

    start1 = time.time()
    result_ransac = execute_global_registration(source_down, target_down,
            source_fpfh, target_fpfh, voxel_size,jparams)
    
    
    fitness = result_ransac.fitness
    rmse = result_ransac.inlier_rmse    
    corr = result_ransac.correspondence_set
    target_match = len(result_ransac.correspondence_set)/target_fpfh.data.shape[1]    
    fitness_matching = [fitness, rmse, target_match]   
    end1 = time.time()
    draw_registration_result(source_down, target_down,result_ransac.transformation)      
    
    if(jparams["optimize"]["refine"]== "yes"):    
        reg_p2p = improve_alignment(source_down,target_down,result_ransac.transformation, jparams)    
        fitness = reg_p2p.fitness
        rmse = reg_p2p.inlier_rmse        
        corr = reg_p2p.correspondence_set
        target_match = len(reg_p2p.correspondence_set)/target_fpfh.data.shape[1]    
        fitness_matching = [fitness, rmse, target_match]     
        
     ## testing performed with pose optiization not implemented fully 
    
    if (jparams["optimize"]["pose_optimize"] == "yes"):      
        pcds = []
        pcds.append(target)
        pcds.append(source)    
        draw_geometries(pcds)   
        voxel_size = jparams["optimize"]["voxel_size"] 
        max_coarse = voxel_size * jparams["optimize"]["max_coarse"]
        max_fine = voxel_size * jparams["optimize"]["max_fine"]
        do_optimization(pcds,voxel_size,max_coarse, max_fine)  
        
    m_time = float(round(end1-start1,2))
    t_time = m_time + pre_time
    return corr, source, target,fitness_matching, t_time
        
