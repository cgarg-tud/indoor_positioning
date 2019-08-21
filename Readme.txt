CONTENTS OF THIS FILE
---------------------
 * Introduction
 * Necessary additional files
 * Requirements
 * Additional information
 * Creators

INTRODUCTION
------------
Different python programs used for the Synthesis project 2019 in which indoor localisation using point clouds was attempted.
The files indicated with "TEST_" are not used in our final implementation and are not thoroughly tested.
______________
params.json :
This provides one stop file to include all directories as indicated for inputs, database and output.

_____________________________________
main.py: and feature_matching.py
Through this script, it is possible to load in a point cloud and find the location for several inputs. If DIM input is used, rigid alignment is needed first and then feature matching is done so "DIM":"yes", "rigid_align" : "yes", "LIDAR" : "no" . After that feature matching is done by using "DIM":"yes" and "rigid_align" : "no", "similarity" : "yes" , "LIDAR" : "no".

If LIDAR input is used, no preprocessing is required as such and feature matching is done by using "LIDAR":"yes, "similarity" : "yes", "DIM": "no", "rigid_align" : "no".

_____________________
find_hist_features.py
This contains all main functions that use Open3D functions and examples to perform all operations of feature matching 

_____________________________________________
first_alignment1.py and creating_testfiles.py
This script contains core functions for rigid alignment initailly made by Karin 

_________________
plane_extract.py 
This script is used to utilize camera position parameters to automate ceiling extraction. Only a part of this script has been included in implementation and no ceiling is added. However, individually using this script, further processing can be performed.

____________________________
TEST_pose_optimization.py
This script is for testing the optimization of registration done fnd_hist_features. It was not possible to completely implement it in the final workflow for full results
NOTE: WE DID NOT USE THIS SCRIPT AND METHOD FOR OUR FINAL IMPLEMENTATION
NOTE: BECAUSE IT REQUIRES FURTHER INVESTIGATION, THIS FILE HAS NOT YET BEEN FULLY TESTED.

__________________________
TEST_raster folder:
Experiments to apply DEM using nearest neighbor and TIN interpolation are performed in this script.
NOTE: WE DID NOT USE THIS SCRIPT AND METHOD FOR OUR FINAL IMPLEMENTATION


NECESSARY ADDITIONAL FILES
------------

"USER_PC_ALL": contains the user input point clouds (in .xyz format) from the laser scanner
"USER_PC_ALL_DIM": contains the pre-processed user input point clouds from the dense image matching (DIM) 
"USER_LDB_ALL":A change in database would require a change in the code.
"USER_PC_ALL_CAM": contains the camera external parameters of DIM input point clouds
 

MAIN REQUIREMENTS
------------
*csv (standard library, import csv) https://docs.python.org/3/library/csv.html
*cv2 (pip install opencv-python) https://pypi.org/project/opencv-python/
*math (standard library, import math) https://docs.python.org/3/library/math.html
*matplotlib (pip install matplotlib) https://matplotlib.org/
*numpy (pip install numpy) https://www.numpy.org/
*scipy (pip install scipy) https://www.scipy.org/
*statistics (pip install statistics) https://pypi.org/project/statistics/
*imutils (pip install imutils) https://pypi.org/project/imutils/
*string (standard library, import string) https://docs.python.org/3/library/string.html
*json (pip install json) 
*pycpd (pip install pycpd) https://github.com/siavashk/pycpd
*open3d(pip install open-3d) https://pypi.org/project/open-3d/

ADDITIONAL INFORMATION
------------
The files are created with the following system specifications:
* Python Version: Python 3.7.x
* Operating system: Windows


CREATORS
-----------
Chirag Garg 
There are many libraries and examples used, all referenced in the code.
