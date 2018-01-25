import sys
import os
import matplotlib
matplotlib.use('Agg')
import pypcd
import argparse
import numpy as np
import matplotlib.pyplot as plt




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

def write_vector_to_file(data,output):
    file = open(output, 'w')
    for item in data:
        file.write("%s " % item)

def main(dataset_path,threshold):
    for subdir, dirs, files in os.walk(dataset_path):
    
        for file in files:
            #print os.path.join(dirs, file) 
            current_path = os.path.join(subdir, file)
            if current_path.endswith('.pcd'):
                print "Processing : " + current_path
                pts,normals = load_pcd(current_path)
                
                nbPoints = pts.shape[1]
                if (nbPoints < threshold ):
                    print("Nb points: {} & threshold {}".format(nbPoints,threshold))
                    print("Point cloud is removed | Size : {} \n".format(nbPoints))
                    os.remove(current_path)
        

if __name__ == '__main__':
    print("\n==> Remove cloud inside a folder where its size in inferior to a certain threshold")
    ap = argparse.ArgumentParser()
    ap.add_argument("-dataset", "--dataset", required=True, help="Dataset for removing cloud")
    ap.add_argument("-size", "--size", required=True, help="Threshold for number of points (If cloud.size < size, it will be deleted")
    args = vars(ap.parse_args())

    if args["dataset"] is not None:
        dataset_descriptor_path = args["dataset"]
    if args["size"] is not None:
        threshold = int(args["size"])

    main(dataset_descriptor_path,threshold)


