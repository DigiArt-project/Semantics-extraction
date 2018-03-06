import sys
import os
import matplotlib
matplotlib.use('Agg')
here = os.path.dirname(os.path.realpath(__file__))
sys.path.append("S3DGLPy")
from PolyMesh import *
import numpy as np
import matplotlib.pyplot as plt
import ShapeStatistics as shp
import pypcd
import argparse
from sklearn.preprocessing import StandardScaler,MinMaxScaler,Normalizer


dataset_descriptor_path = "/Users/lironesamoun/digiArt/Tools/structuresensor_descriptors/"
resolution = 2
cmap = plt.get_cmap('jet') # color ramp
#If no have views folder
have_views_folder = False
output_dataset_save = "/Users/lironesamoun/digiArt/Tools/structuresensor_full_descriptors/"

def scale_list(data, range_min = 0, range_max = 1):
    results = list(map(float, data))
    results = MinMaxScaler(feature_range=(range_min,range_max)).fit_transform(results)

    return results



def load_pcd(path):
    pc = pypcd.PointCloud.from_path(path)

    x = np.array(pc.pc_data['x'])
    y = np.array(pc.pc_data['y'])
    z = np.array(pc.pc_data['z'])

    pts = [x, y, z]
    pts = np.asarray(pts)

    if len(pc.pc_data.dtype.names) > 6:
        normal_x = np.array(pc.pc_data['normal_x'])
        normal_y = np.array(pc.pc_data['normal_y'])
        normal_z = np.array(pc.pc_data['normal_z'])

        normals = [normal_x, normal_y, normal_z]
        normals = np.asarray(normals)

        return pts,normals
    else:
        normals = None
        return pts, normals


def normalize_points(pts):
    N = pts.shape[1]
    centroid = shp.getCentroid(pts)
    #Demean
    pts = pts - centroid
    #Normalize by its root mean square distance to the origin
    pts = pts*np.sqrt(N/np.sum(pts**2))
    #Scale
    scale = 1 / (np.sqrt(np.sum(np.square(pts)) / N))
    pts = np.multiply(scale, pts)

    return pts

def write_vector_to_file(data,output):
    file = open(output, 'w')
    for item in data:
        file.write("%s " % item)

def main(dataset_path):
    for subdir, dirs, files in os.walk(dataset_path):
    
        for file in files:
            #print os.path.join(dirs, file) 
            current_path = os.path.join(subdir, file)
            if current_path.endswith('.pcd'):
                

                parent_dir = os.path.abspath(os.path.join(subdir, os.pardir))
                #category_folder = os.path.basename(parent_dir)
                category_folder = os.path.basename(os.path.dirname(os.path.abspath(current_path)))
                #descriptor_folder_path_spin = output_dataset_save + category_folder + "/descriptors/spin/"
                #descriptor_folder_path_egi = output_dataset_save + category_folder + "/descriptors/egi/"
                descriptor_folder_path_spin = output_dataset_save + category_folder + "/descriptors/spin/"
                descriptor_folder_path_egi = output_dataset_save  + category_folder +"/descriptors/egi/"

                #Create folder for storing EGI descriptor and Spin descriptor
                
                if not os.path.exists(descriptor_folder_path_spin):
                    os.makedirs(descriptor_folder_path_spin)
                if not os.path.exists(descriptor_folder_path_egi):
                    os.makedirs(descriptor_folder_path_egi)
                
                base = os.path.basename(current_path)
                filename =  os.path.splitext(base)[0]
                

                            
                name_descriptor = "desc_"+filename+".txt"

                output_spin =  descriptor_folder_path_spin + name_descriptor
                output_egi =  descriptor_folder_path_egi + name_descriptor
                
                if not os.path.exists(output_spin):
                    print("Processing : {}".format(current_path))
                    pts,normals = load_pcd(current_path)

                 
                       
                    pts = normalize_points(pts)
                    #Get spin image
                    NAngles = 100#720
                    Extent = 2#2
                    Dim = 40
                    #More bins give better resolution but increase comparison computations by a factor of N2
                    #Support distance : smaller distance finds only locals features / Larger distance give global features
                    histogram = shp.getSpinImage(pts, normals, NAngles, Extent, Dim, False)
                    histogram_scaled = scale_list(histogram)

                    write_vector_to_file(histogram_scaled,output_spin)
                    #print histogram

                    #Get Extended Gaussian Image
                    sphere = getSphereMesh(1, resolution)
                    hist = shp.getEGIHistogram(pts, normals, sphere.VPos.T)
                    hist = hist / np.max(hist)
                    hist_scaled = scale_list(hist)
                    write_vector_to_file(hist_scaled,output_egi)
                    #print hist
                    
                



def main_views(dataset_descriptor_path):
    for subdir, dirs, files in os.walk(dataset_descriptor_path):
    
        for file in files:

            #print os.path.join(dirs, file) 
            current_path = os.path.join(subdir, file)
            #print "Processing : " + current_path
            if ("/views/" in current_path and current_path.endswith('.pcd')):
                print("Processing : {}".format(current_path))
                parent_dir = os.path.abspath(os.path.join(subdir, os.pardir))
                descriptor_folder_path_spin = parent_dir + "/descriptors/spin/"
                descriptor_folder_path_egi = parent_dir + "/descriptors/egi/"
                #Create folder for storing EGI descriptor and Spin descriptor
                if not os.path.exists(descriptor_folder_path_spin):
                    os.makedirs(descriptor_folder_path_spin)
                if not os.path.exists(descriptor_folder_path_egi):
                    os.makedirs(descriptor_folder_path_egi)
                
                base = os.path.basename(current_path)
                filename =  os.path.splitext(base)[0]
                filename = filename.split("_",1)[1]
                
                name_descriptor = "desc_"+filename+".txt"
                output_spin =  descriptor_folder_path_spin + name_descriptor
                output_egi =  descriptor_folder_path_egi + name_descriptor
                if not os.path.exists(output_spin):
                    
                    pts,normals = load_pcd(current_path)
                    if not normals == None:
                       
                        pts = normalize_points(pts)
                        #Get spin image
                        NAngles = 100#720
                        Extent = 2#2
                        Dim = 40
                        #More bins give better resolution but increase comparison computations by a factor of N2
                        #Support distance : smaller distance finds only locals features / Larger distance give global features
                        histogram = shp.getSpinImage(pts, normals, NAngles, Extent, Dim, False)
                        histogram_scaled = scale_list(histogram)

                        write_vector_to_file(histogram_scaled,output_spin)
                        #print histogram

                        #Get Extended Gaussian Image
                        sphere = getSphereMesh(1, resolution)
                        hist = shp.getEGIHistogram(pts, normals, sphere.VPos.T)
                        hist = hist / np.max(hist)
                        hist_scaled = scale_list(hist)
                        write_vector_to_file(hist_scaled,output_egi)
                        #print hist
                    
                



if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-dataset", "--dataset", required=True, help="Dataset for computing the spin and EGI descriptor")
    args = vars(ap.parse_args())

    if args["dataset"] is not None:
        dataset_descriptor_path = args["dataset"]

    if (have_views_folder):
        main_views(dataset_descriptor_path)
    else:
        main(dataset_descriptor_path)


