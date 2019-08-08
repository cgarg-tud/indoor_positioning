import math
import numpy as np
from scipy.linalg import eigh
import numpy as np 
import os
import json, sys
import time
from open3d import *
from functools import partial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pycpd import rigid_registration
import shutil


from find_hist_features import *
    
def get_camera_points(name, file):
    index = 0 
    positions_camera = []
    
    for line in file:
        if index != 0:
            split_line = line.split()
            positions_camera.append((float(split_line[1]), float(split_line[2]), float(split_line[3])))
        index = index + 1 
        
    return positions_camera

#### Find closest camera point; from Celine 
def fit_plane(pts):
    # shift points to mean
    mean = np.mean(pts, axis = 0)  
    pts -= mean
    # compute covariance matrix and eigenvalues and eignevectors
    cov = np.cov(pts, rowvar = False)
    evals, evecs = eigh(cov)
    # find smallest eigenvalue, the corresponging eigenvector is our normal n
    idx = np.argsort(evals)[::-1]
    evecs = evecs[:,idx]
    evals = evals[idx]

    n = evecs[:,-1]
    c = mean
    return c, n

def add_ceiling(jparams, cam_name):
    
    cam_dir = jparams['cam_dir']
    cam_file = open(cam_dir + cam_name, 'r')
    
    positions_camera = get_camera_points(cam_name, cam_file)    
    c,n = fit_plane(positions_camera)    
    
    if (jparams['add_ceiling']["check"] == "no"):        
        return c
    else:    
    
        #    print(c)
        #    print(n)
        user_name = jparams['user_name']
        dirName = user_name     
        # The point pcd_user and pcd_database     
        pcd_user = read_point_cloud(jparams['user_dir'] + user_name + '.xyz')      
        pcd_user = remove_outliers(pcd_user,jparams)  
        draw_geometries([pcd_user])
        #voxel_grid = create_surface_voxel_grid_from_point_cloud(pcd_user, 8)
        #
        #draw_geometries([voxel_grid])
          
        points = pcd_user.points
        sortedx=sorted(points, key = lambda l:  l[0])
        sortedy = sorted(points, key = lambda l:  l[1])
        sortedz = sorted(points, key = lambda l:  l[2])   
        
        plane_points = []
        
        minx,maxx = sortedx[0][0],sortedx[-1][0]
        miny,maxy = sortedy[0][1],sortedy[-1][1]
        minz,maxz = sortedz[0][2],sortedz[-1][2]
        
        Dist =[]
        count = 0
        for point in points:
            dist = np.dot((c-point),n)
            Dist.append([dist, point[2]])
            count = count + 1    
        
        pt_cov = abs((maxx -minx)*(maxy-miny)*(maxz-minz))/count
        
        Dist = np.array(Dist)
        
        max_th = Dist[:,0].max()
        min_th = Dist[:,0].min()
        
        tol = 0.01* (max_th - min_th)
        
        indices = np.where(Dist[:,0] > (max_th - tol))
        top_points = Dist[indices]
        plane_z = np.average(top_points[:,1])        
        plane_points = []
        
        for cnt in range(int(pt_cov)):
            for cnt2 in range(int(pt_cov)):
                plane_points.append([minx + cnt*((maxx-minx)/pt_cov),miny + cnt2*((maxy-miny)/pt_cov),plane_z])
        
          # Create directory
        dirName = user_name 
        try:
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ") 
        except FileExistsError:
            print("Directory " , dirName ,  " already exists")
        
        
        file = open(dirName + '/' + 'plane_' + user_name +'.xyz', 'w')
        index = 0 
        for point in plane_points:
            index = index + 1 
            file.write(str(point[0]) + ' ' + str(point[1]) + ' ' + str(point[2]) + '\n') 
        file.close()
        
        plane = read_point_cloud(dirName + '/' + 'plane_' + user_name +'.xyz')
       
            
        pcds = []
        pcds.append(pcd_user)
        pcds.append(plane)
        
        
        draw_geometries(pcds)

