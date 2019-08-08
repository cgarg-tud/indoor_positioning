###creating same dimension###
from open3d import *
from functools import partial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pycpd import rigid_registration
import numpy as np
import time

def textfile_database(pcd, name, dirName,jparams):
    #Creating KD tree and voxels 
    pcd_tree = KDTreeFlann(pcd)
    pcd = voxel_down_sample(pcd,  voxel_size = jparams["db_voxel_size"])
#    draw_geometries([pcd])

    #Creating new file 
    file = open(dirName + '/' + 'new_' + name +'.xyz', 'w')
    index = 0 
    for point in pcd.points:
        index = index + 1 
        file.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n') #It also possible to obtain normals and other information about the points 
    file.close()

    X = np.loadtxt(dirName + '/'+ 'new_' + name +'.xyz')
    print("the number of database points " + str(index))
    return X 

def textfile_user(pcd, name, dirName,jparams):
    #Creating KD tree and voxels 
    pcd_tree = KDTreeFlann(pcd)
    pcd = voxel_down_sample(pcd, voxel_size = jparams["user_voxel_size"])
#    draw_geometries([pcd])

    #Creating new file 
    file = open(dirName + '/' + 'new_' + name +'.xyz', 'w')
    index = 0 
    for point in pcd.points:
        index = index + 1 
        file.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n') #It also possible to obtain normals and other information about the points 
    file.close()
    Y = np.loadtxt(dirName + '/' + 'new_' + name +'.xyz') #synthetic data, equaivalent to X + 1  
    print("the number of user points " + str(index))
    return Y



#def textfile_user_pose(pcd, name, dirName):
#    #Creating KD tree and voxels 
#    pcd_tree = KDTreeFlann(pcd)
#    pcd = voxel_down_sample(pcd, voxel_size = 8)
#    draw_geometries([pcd])
#
#    #Creating new file 
#    file = open(dirName + '/' + 'new_' + name +'.xyz', 'w')
#    index = 0 
#    for point in pcd.points:
#        index = index + 1 
#        file.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n') #It also possible to obtain normals and other information about the points 
#    file.close()
#    Y = np.loadtxt(dirName + '/' + 'new_' + name +'.xyz') #synthetic data, equaivalent to X + 1  
#    print("the number of user points " + str(index))
#    return Y

