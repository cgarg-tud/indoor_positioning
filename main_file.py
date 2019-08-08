### main file ###

from feature_matching import *


import numpy as np 
import os
import time
import json,sys
import csv

import xmltodict
import operator
from operator import itemgetter, attrgetter




def main():
 
     #-- read the needed parameters from the file 'params.json' (must be in same folder)
    try:       
        jparams = json.load(open('params.json'))
       
    except:
        print("ERROR: something is wrong with the params.json file.")
        sys.exit()        
    
    start = time.time()       
       
    
    # Input all database files.
    p1db = "DB1"
    p2db = "DB2"
    p3db = "DB3"
    p4db = "DB4"
    p5db = "DB5"
    p6db = "DB6"
    p7db = "DB7"
    p8db = "DB8"
    p9db = "DB9"
    p10db = "DB10"
    
    # Dictionary for all database files.
    
    room_db = {"1": p1db, "2": p2db, "3": p3db, "4": p4db, "5": p5db, "6": p6db, "7": p7db,\
               "8": p8db, "10": p10db}
    
    db_dir = jparams["db_dir"]           
     
    #Directories for DIM input
    
    if(jparams["DIM"] == "yes"): 
        
        user_dir = jparams["dim_user_dir"]
        
        filenames = ["Results_avg_UC1a", "Results_avg_UC1b" ,"Results_avg_UC2a","Results_avg_UC2b", "Results_avg_UC3a",\
                     "Results_avg_UC3b","Results_avg_UC4a", "Results_avg_UC4b","Results_avg_UC5a","Results_avg_UC5b" ,"Results_avg_UC6a",\
                     "Results_avg_UC6b","Results_avg_UC7a","Results_avg_UC7b","Results_avg_UC8a", "Results_avg_UC8b", "Results_avg_UC9a",\
                     "Results_avg_UC9b","Results_avg_UC10a", "Results_avg_UC10b"]           
        
        cam_db = { "Results_avg_UC1a" : "cam1a.txt", "Results_avg_UC1b" :"cam1b.txt" ,"Results_avg_UC2a" : "cam2a.txt","Results_avg_UC2b": "cam2b.txt", "Results_avg_UC3a": "cam3a.txt",\
                     "Results_avg_UC3b" : "cam3b.txt","Results_avg_UC4a" : "cam4.txt", "Results_avg_UC4b":"cam4.txt","Results_avg_UC5a" : "cam5.txt","Results_avg_UC5b":"cam5.txt" ,"Results_avg_UC6a":"cam6.txt",\
                     "Results_avg_UC6b" : "cam6.txt","Results_avg_UC7a" : "cam7a.txt","Results_avg_UC7b" :"cam7b.txt","Results_avg_UC8a" : "cam8.txt", "Results_avg_UC8b" :"cam8.txt", "Results_avg_UC9a":"cam9.txt",\
                     "Results_avg_UC9b":"cam9.txt","Results_avg_UC10a":"cam10.txt", "Results_avg_UC10b" :"cam10.txt"}
        
     #Directories for LIDAR Input
    
    if(jparams["LIDAR"] == "yes"): 
        
        user_dir = jparams["lidar_user_dir"]        
        
        filenames = ["UL1a", "UL1b", "UL2a", "UL2b", "UL2c", "UL3a", "UL3b", "UL4a", "UL4b", "UL4c",\
             "UL5a", "UL5b", "UL5c", "UL6a", "UL6b", "UL6c", "UL6d", "UL6e", "UL6f", "UL6g", "UL7a", "UL7b",\
             "UL8a", "UL8b", "UL9a", "UL9b", "UL9c", "UL9d", "UL9e", "UL10a", "UL10b", "UL10c"]
        
     
    total_result = []
    voxel_size = jparams["matching"]["voxel_size"]
    
    for filename in filenames:     
         
        output_file = []            
        user_name = filename  
        dirName =  jparams["output_dir"]  + user_name        
        
        #Create a directory for every input in output directory 
         
        try:
            os.mkdir(dirName)
            print("Directory " , dirName ,  " Created ") 
        except FileExistsError:
            print("Directory " , dirName ,  " already exists")
            
        print("You tested for the following room:", user_name)
        start2 = time.time()      
    
        #do rigid alignment for DIM input before feature matching and for any other inout if it is known that it needs to be rigidly aligned first
        
        if(jparams["DIM"] == "yes" and jparams["rigid_align"] == "yes") :
            
            pcd_user = read_point_cloud(user_dir + user_name + '.xyz')   
            
             #  Remove outliers if not preprocessed input 
            if(jparams["pre_process"] == "yes"):
                pcd_user = remove_outliers(pcd_user,jparams)  
          
            cam_file = add_ceiling(jparams,cam_db[filename])   
            cam_R={}  
                     
            count = 1        
            for r in room_db:   
                startr = time.time()
                print(r)
                database_name = room_db[r]
                pcd_database = read_point_cloud(db_dir + database_name + '.xyz')                   
                cam_pos = do_rigid_alignment(dirName, database_name, user_name,  pcd_user, pcd_database,cam_file, jparams)
                file = open(dirName + '/' + 'cam_pos_' + user_name +'.xyz', 'a+')
                file.write(str(cam_pos[0]) + ' ' + str(cam_pos[1]) + ' ' + str(cam_pos[2]) + ' ' + r + '\n') #It also possible to obtain normals and other information about the points 
                file.close()
                cam_R.update({r : cam_pos})               
                endr=time.time()
                print (" rigid Alignment Runtime: " + str(round(endr-startr,2))+" sec\n\n")
      
          
        #do feature matching 
         
         #if DIM input is used make sure, rigid alignment is done before matching
         
        if(jparams["similarity"] == "yes"):   
            
            if(jparams["DIM"] == "yes"):            
                cam_R={}
                cam = open(filename + '/' + 'cam_pos_' + user_name +'.xyz')
                for line in cam:
                    x,y,z,db_id = line.split()
                    cam_R.update({int(db_id) : [float(x), float(y), float(z)]})  
                cam.close() 
            
            voxel_size = jparams["matching"]["voxel_size"]
            
            #test each input with each database room and store result in csv file 
            if(jparams["LIDAR"] == "yes"):
                cam_pos = 0
                inp = read_point_cloud(user_dir + user_name + '.xyz')
                
            for r in room_db:
        #            
                database_name = room_db[r]    
                print(database_name)   
                db = read_point_cloud(db_dir + database_name + '.xyz')    
                
                if(jparams["DIM"] == "yes"):
                    cam_pos = cam_R[int(r)]
                    inp =  read_point_cloud(filename + '/' + database_name + filename + '.xyz')               

                    
                #matching function to get room signature               
                corr, source, target, fitness_ransac, step_time = find_similarity(db, inp, voxel_size,cam_pos,jparams)  
                print(fitness_ransac)                
         
                if (fitness_ransac[1] == 0.0):
                    fitness_ransac[1] = 1.0                   
                output= (r,fitness_ransac[0],fitness_ransac[1], step_time, len(db.points))               
                output_file.append(output)     
            
    #        Get the top n rooms based on fitness, inliers and corresponding setsize.
            
            top_n = 10
            rooms_final = sorted(output_file , key=itemgetter(1), reverse = True )[:top_n] 
                
            end2 = time.time()
            with open( dirName + "/" + filename  + "output_v_" +  str(voxel_size) + "_" + str(time.time())+ ".csv", 'a+', newline='') as csv_file:
                            print("NOTE\t> Started writing to the output file.")
                            writer = csv.writer(csv_file, delimiter=';')
                            writer.writerow(['Method','Input', 'Input_size','Voxel_size','DB_size','DB_Location','fitness','rmse','time'])
                            for item in rooms_final :                              
                                writer.writerow(['ransac', user_name, len(inp.points), voxel_size, item[4], item[0],item[1],item[2],item[3]])      
            i_room = ""
            for s in list(user_name):            
                if s.isdigit():
                    i_room = i_room + s 
                    
            rooms_final = np.array(rooms_final)
            actual_position = rooms_final[:,0].tolist().index(i_room) + 1
            actual_output = rooms_final[actual_position-1]          
            total_result.append(['ransac', user_name, voxel_size, rooms_final[0][0],actual_position,rooms_final[0][1],rooms_final[0][2],actual_output[1],actual_output[2],round(end2-start2,2)])
     
        
    # Write localization results of all inputs to csv file for confusion matrix and analysis    
    with open(jparams["output_dir"] + "10_output_v_" +  str(voxel_size) + ".csv", 'a+', newline='') as csv_file:
            print("NOTE\t> Sample final output file created.")
            writer = csv.writer(csv_file, delimiter=';')
            writer.writerow(['Method','Input','Voxel_size','Top_Location','Actual_Position','Top_fitness','Top_rmse','Actual_fitness','Actual_rmse','Actual_time'])
            for result in total_result:                
                writer.writerow(result)    
                
    end=time.time()
    print (" Full Runtime: " + str(round(end-start,2))+" sec\n\n")
                        
            


if __name__ == '__main__':
    main()
