import sys
import os
import numpy as np
import argparse
#import pymesh
#import ShapeStatistics as shp

#def normalize_points(pts):
#    N = pts.shape[1]
#    centroid = shp.getCentroid(pts)
#    #Demean
#    pts = pts - centroid
#    #Normalize by its root mean square distance to the origin
#    pts = pts*np.sqrt(N/np.sum(pts**2))
#    #Scale
#    scale = 1 / (np.sqrt(np.sum(np.square(pts)) / N))
#    pts = np.multiply(scale, pts)
#
#    return pts

def normalizeFolder(folder_path, new_folder_path):

    for subdir, dirs, files in os.walk(folder_path):
    
        for file in files:
            current_path = os.path.join(subdir, file)
            if current_path.endswith('.ply'):
                if 'view' in file:
                    sub = subdir.split('/')
                    aux_path = os.path.join(new_folder_path, sub[len(sub)-2] + '/views/')
                    new_path = os.path.join(aux_path, file)
                    #print('view file : {}'.format(new_path))
                    if not os.path.exists(aux_path):
                        os.makedirs(aux_path)
                else:
                    sub = subdir.split('/')
                    aux_path = os.path.join(new_folder_path, sub[len(sub)-1] + '/full/')
                    new_path = os.path.join(aux_path, file)
                    #print('full file : {}'.format(new_path))
                    if not os.path.exists(aux_path):
                        os.makedirs(aux_path)	
                #print(new_path)
                cmd = './normalize_pointcloud_main ' + current_path + ' ' + new_path
                os.system(cmd) # returns the exit status

				
				
				
				#mesh = pymesh.load_mesh( current_path)
                #pts = mesh.vertices[:, 0:3]
                #pts = normalize_points(pts)
                #new_mesh = pymesh.form_mesh(pts, mesh.faces)
                #pymesh.save_mesh(new_path, new_mesh)
                

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-folder", "--folder", required=True, help="Dataset for normalizing point cloud")
    ap.add_argument("-new-folder", "--new-folder", required=True, help="Dataset for saving the normalization")
    args = vars(ap.parse_args())

    normalizeFolder(args["folder"], args["new_folder"])
